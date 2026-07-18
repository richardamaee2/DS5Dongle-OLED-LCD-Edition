//
// src/lcd.cpp — Waveshare Pico-LCD-1.3 backend (ST7789VW, 240×240 IPS, RGB565).
//
// Compile-time alternative to src/oled.cpp, selected with -DDISPLAY_LCD13=ON
// (CMakeLists.txt compiles exactly ONE of the two files; both export the same
// oled_init()/oled_loop() entry points and the g_lightbar_override global, so
// nothing outside the display layer changes). Screen list, navigation
// semantics, input handling and all display-independent service logic (charge
// ETA, IMU calibration, lightbar engine, diag rates) are ported 1:1 from
// oled.cpp — a change of panel must never change behaviour.
//
// ── Latency budget (the load-bearing design constraint) ─────────────────────
// Everything here runs cooperatively on core 0 next to the BT→USB input path,
// so any blocking work in this file directly delays input processing. A
// 240×240×16-bit frame is 115,200 bytes ≈ 10× the OLED's SPI traffic; sending
// it blocking at 40 MHz would stall the loop for ~23 ms — unacceptable.
// Instead the flush is fully DMA-driven and the CPU never waits for it:
//
//   render pass (~10 Hz): draw the screen into the RAM framebuffer, then
//     start_flush(): write the 11-byte CASET/RASET/RAMWR window preamble
//     (≈2 µs of blocking SPI at 40 MHz) and start one background DMA that
//     streams the whole framebuffer into the SPI TX FIFO, paced by DREQ.
//   every loop iteration: flush_service() — two non-blocking status reads;
//     when the DMA and the SPI shifter are both done it raises CS. If a
//     render falls due while a flush is still in flight, the render is
//     deferred to the next loop iteration (no tearing, no waiting).
//
// The render pass itself (fb_clear + text/rects into RAM) costs on the order
// of the OLED's blocking flush (~1 ms every 100 ms), which is the agreed
// budget. Both backends self-measure: the loop wraps the render in two
// timestamps and publishes the worst case via latency_note_display_busy()
// (Latency screen footer, 0xFD bytes [70..73]) — so the "LCD must not block
// longer than the OLED" requirement is checkable on real hardware, not vibes.
//
// Headless rule: unchanged from the OLED backend. No init checks, no probing —
// with no panel attached the SPI writes and the DMA go nowhere, GPIO pull-ups
// read "not pressed", and the firmware boots and runs identically.
//
// Panel/driver references: Waveshare Pico-LCD-1.3 wiki (pinout below) and the
// vendor LCD_1in3.c init sequence (gamma/porch/power constants kept verbatim).
//

#include "oled.h"
#include "oled_font.h"
#include "bt.h"
#include "slots.h"
#include "audio.h"
#include "config.h"
#include "state_mgr.h"
#include "latency.h"

#include <cstdio>
#include <cstring>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "cmd.h"
#include "pico/time.h"

extern uint8_t interrupt_in_data[63]; // defined in main.cpp

// Mic diagnostic counters (defined in main.cpp).
extern uint32_t bt_31_packet_count();
extern uint32_t host_out02_total();
extern uint32_t host_out02_trig_allow();
extern uint32_t host_out02_to_bt();
extern uint32_t host_out02_trig_folded();
extern uint8_t  bt_31_last_byte2();
extern uint8_t  bt_31_b2_or_mask();
extern uint16_t bt_31_len_min();
extern uint16_t bt_31_len_max();
extern void     bt_31_mic_prefix(uint8_t out[6]);
extern bool     spk_active; // main.cpp: true while host USB speaker stream is open

// Global (not in the anon namespace below) so state_mgr.cpp can extern it:
// true while a lightbar mode or the charging pulse owns the LED, which
// tells state_update() to ignore the host's AllowLedColor writes.
bool g_lightbar_override = false;

namespace {

// ── Pico-LCD-1.3 pinout (Waveshare wiki, verified 2026-07) ──────────────────
// Display shares the Pico-OLED-1.3's SPI1 pins; backlight is new on GP13.
// Buttons A/B sit on the same GPIOs as the OLED's KEY0/KEY1 → mapped 1:1.
constexpr uint kPinDC   = 8;
constexpr uint kPinCS   = 9;
constexpr uint kPinCLK  = 10;
constexpr uint kPinMOSI = 11;
constexpr uint kPinRST  = 12;
constexpr uint kPinBL   = 13;  // backlight, PWM-dimmed (idle power ladder)
constexpr uint kPinKey0 = 15;  // button A  (KEY0 semantics: next screen)
constexpr uint kPinKey1 = 17;  // button B  (KEY1 semantics: back / brightness)
constexpr uint kPinKeyX = 19;  // button X  (unused — reserved)
constexpr uint kPinKeyY = 21;  // button Y  (unused — reserved)
// 5-way joystick — optional extra: D-pad-equivalent navigation on the
// Settings / Slots / Diagnostics screens, CTRL press = Triangle-equivalent.
constexpr uint kPinJoyUp    = 2;
constexpr uint kPinJoyCtrl  = 3;
constexpr uint kPinJoyLeft  = 16;
constexpr uint kPinJoyDown  = 18;
constexpr uint kPinJoyRight = 20;

constexpr int kW = 240;
constexpr int kH = 240;
constexpr int kFbPixels = kW * kH;
constexpr int kFbBytes  = kFbPixels * 2;

// Set to true if your unit shows everything upside-down (two board revisions
// mount the panel 180° apart). Flips MADCTL and applies the GRAM row offset
// the flipped orientation needs on this 240×240-of-320 controller.
constexpr bool kRotate180 = false;
constexpr uint8_t kMadctl    = kRotate180 ? 0xB0 : 0x70;
constexpr int     kRowOffset = kRotate180 ? 80   : 0;

// RGB565 framebuffer, stored BYTE-SWAPPED (ST7789 wants MSB first on the wire;
// pre-swapping at draw time lets the DMA stream raw bytes with zero per-flush
// CPU work). 115,200 bytes of .bss.
uint16_t fb[kFbPixels];

// ── Colours ─────────────────────────────────────────────────────────────────
// All colour constants go through rgb565(), which byte-swaps for the wire.
constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    const uint16_t c = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
    return (uint16_t)((uint16_t)(c << 8) | (uint16_t)(c >> 8));
}
constexpr uint16_t kBlack  = rgb565(0, 0, 0);
constexpr uint16_t kWhite  = rgb565(255, 255, 255);
constexpr uint16_t kGrey   = rgb565(140, 140, 150);
constexpr uint16_t kDGrey  = rgb565(70, 70, 78);
constexpr uint16_t kRed    = rgb565(255, 60, 50);
constexpr uint16_t kGreen  = rgb565(60, 220, 90);
constexpr uint16_t kYellow = rgb565(240, 200, 40);
constexpr uint16_t kAmber  = rgb565(255, 160, 0);
constexpr uint16_t kPureR  = rgb565(255, 0, 0);
constexpr uint16_t kPureG  = rgb565(0, 255, 0);
constexpr uint16_t kPureB  = rgb565(64, 100, 255); // readable blue, not #0000FF-on-black

// Per-player accent — P1..P4 are EXACTLY the PS5 player colours the 4-Player
// Edition stamps on the lightbar (state_mgr.cpp kPlayerColor): blue, red,
// green, pink. P5..P8 are this fork's 8-Player extension colours (orange,
// cyan, purple, yellow), kept in sync with kPlayerColor there.
// player_id == 0 (off) → white/neutral chrome.
constexpr uint16_t kPlayerAccent[9] = {
    kWhite,
    rgb565(0, 90, 255),    // P1 blue (brightened for text legibility)
    rgb565(255, 60, 50),   // P2 red
    rgb565(60, 220, 90),   // P3 green
    rgb565(255, 70, 160),  // P4 pink
    rgb565(255, 140, 30),  // P5 orange
    rgb565(40, 210, 230),  // P6 cyan
    rgb565(170, 90, 255),  // P7 purple
    rgb565(255, 210, 40),  // P8 yellow
};
inline uint16_t theme_accent() {
    const uint8_t pid = get_config().player_id;
    return kPlayerAccent[(pid >= 1 && pid <= 8) ? pid : 0];
}

// ── ST7789VW low-level ──────────────────────────────────────────────────────
void cmd(uint8_t c) {
    gpio_put(kPinDC, 0);
    gpio_put(kPinCS, 0);
    spi_write_blocking(spi1, &c, 1);
    gpio_put(kPinCS, 1);
}

void data_byte(uint8_t d) {
    gpio_put(kPinDC, 1);
    gpio_put(kPinCS, 0);
    spi_write_blocking(spi1, &d, 1);
    gpio_put(kPinCS, 1);
}

void hw_reset() {
    gpio_put(kPinRST, 1); sleep_ms(50);
    gpio_put(kPinRST, 0); sleep_ms(50);
    gpio_put(kPinRST, 1); sleep_ms(50);
}

// Vendor init sequence (Waveshare LCD_1in3.c), constants verbatim: porch,
// gate/VCOM/LCM power trims and the two gamma tables are panel-tuned.
void st7789_init() {
    cmd(0x36); data_byte(kMadctl);            // MADCTL — orientation
    cmd(0x3A); data_byte(0x05);               // COLMOD — 16 bpp RGB565
    cmd(0xB2);                                // PORCTRL
    data_byte(0x0C); data_byte(0x0C); data_byte(0x00); data_byte(0x33); data_byte(0x33);
    cmd(0xB7); data_byte(0x35);               // GCTRL
    cmd(0xBB); data_byte(0x19);               // VCOMS
    cmd(0xC0); data_byte(0x2C);               // LCMCTRL
    cmd(0xC2); data_byte(0x01);               // VDVVRHEN
    cmd(0xC3); data_byte(0x12);               // VRHS
    cmd(0xC4); data_byte(0x20);               // VDVS
    cmd(0xC6); data_byte(0x0F);               // FRCTRL2 — 60 Hz
    cmd(0xD0); data_byte(0xA4); data_byte(0xA1); // PWCTRL1
    cmd(0xE0);                                // PVGAMCTRL
    data_byte(0xD0); data_byte(0x04); data_byte(0x0D); data_byte(0x11);
    data_byte(0x13); data_byte(0x2B); data_byte(0x3F); data_byte(0x54);
    data_byte(0x4C); data_byte(0x18); data_byte(0x0D); data_byte(0x0B);
    data_byte(0x1F); data_byte(0x23);
    cmd(0xE1);                                // NVGAMCTRL
    data_byte(0xD0); data_byte(0x04); data_byte(0x0C); data_byte(0x11);
    data_byte(0x13); data_byte(0x2C); data_byte(0x3F); data_byte(0x44);
    data_byte(0x51); data_byte(0x2F); data_byte(0x1F); data_byte(0x1F);
    data_byte(0x20); data_byte(0x23);
    cmd(0x21);                                // INVON — IPS panel wants inversion
    cmd(0x11);                                // SLPOUT
    sleep_ms(120);                            // datasheet-mandated wake delay (boot only)
    cmd(0x29);                                // DISPON
}

// Panel on/off for the idle ladder's Off tier. DISPOFF/DISPON are immediate
// (no SLPIN — its 120 ms wake delay would block the loop on wake).
bool panel_display_on = true;
void st7789_display(bool on) {
    if (on == panel_display_on) return;
    panel_display_on = on;
    cmd(on ? 0x29 : 0x28);
}

// ── Backlight (PWM on GP13) ─────────────────────────────────────────────────
// LCDs glow on black — in a dark room the backlight IS the panel's power
// state. Brightness levels replace the OLED's contrast register; the idle
// ladder dims and then cuts the backlight entirely.
constexpr uint16_t kBlWrap = 999;             // 320 MHz / 16 / 1000 ≈ 20 kHz PWM
constexpr uint8_t  kBrightPct[] = {100, 55, 28, 10}; // KEY1 long-press cycle
constexpr int kNumBrightLevels = sizeof(kBrightPct) / sizeof(kBrightPct[0]);
int bright_idx = 0;
constexpr uint8_t kDimPct = 4;                // Dim tier: barely-there glow
int current_bl_pct = -1;

void bl_init() {
    gpio_set_function(kPinBL, GPIO_FUNC_PWM);
    const uint slice = pwm_gpio_to_slice_num(kPinBL);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 16.0f);
    pwm_config_set_wrap(&cfg, kBlWrap);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(kPinBL, 0);            // dark until the splash is ready
    current_bl_pct = 0;
}

void bl_set_pct(int pct) {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    if (pct == current_bl_pct) return;
    current_bl_pct = pct;
    pwm_set_gpio_level(kPinBL, (uint16_t)(((uint32_t)kBlWrap * (uint32_t)pct) / 100u));
}

// ── DMA flush engine ────────────────────────────────────────────────────────
int  dma_ch = -1;
bool flush_in_progress = false;

// Non-blocking: call every loop iteration. Two status reads; raises CS and
// retires the flush once the DMA and the SPI shifter have both drained.
void flush_service() {
    if (!flush_in_progress) return;
    if (dma_channel_is_busy(dma_ch)) return;
    if (spi_is_busy(spi1)) return;            // last FIFO bytes still shifting
    gpio_put(kPinCS, 1);
    flush_in_progress = false;
}

// Kick one full-frame flush. Blocking cost: the 11-byte window preamble at
// 40 MHz (~2.2 µs) plus DMA setup — the pixel stream itself is background DMA.
// Never called while a flush is in flight (the render gate guarantees it).
void start_flush() {
    // Column/row window. The ST7789's GRAM is 240×320; this 240×240 panel maps
    // straight (0,0) in the 0x70 orientation, +80 rows when rotated 180°.
    const uint16_t x0 = 0, x1 = kW - 1;
    const uint16_t y0 = kRowOffset, y1 = kRowOffset + kH - 1;
    cmd(0x2A); // CASET
    data_byte(x0 >> 8); data_byte(x0 & 0xFF);
    data_byte(x1 >> 8); data_byte(x1 & 0xFF);
    cmd(0x2B); // RASET
    data_byte(y0 >> 8); data_byte(y0 & 0xFF);
    data_byte(y1 >> 8); data_byte(y1 & 0xFF);
    cmd(0x2C); // RAMWR

    gpio_put(kPinDC, 1);
    gpio_put(kPinCS, 0);                      // CS stays low for the whole frame
    dma_channel_config c = dma_channel_get_default_config(dma_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, spi_get_dreq(spi1, true));
    dma_channel_configure(dma_ch, &c, &spi_get_hw(spi1)->dr,
                          (const uint8_t *)fb, kFbBytes, true);
    flush_in_progress = true;
}

// Boot-time only (splash): synchronous flush so the image is guaranteed on
// the glass before the backlight comes up. Never used after oled_init().
void flush_blocking() {
    start_flush();
    while (flush_in_progress) {
        flush_service();
        tight_loop_contents();
    }
}

// ── Drawing primitives ──────────────────────────────────────────────────────
void fb_clear() { memset(fb, 0, sizeof(fb)); }

inline void px(int x, int y, uint16_t color) {
    if (x < 0 || x >= kW || y < 0 || y >= kH) return;
    fb[y * kW + x] = color;
}

void rect_filled(int x, int y, int w, int h, uint16_t color) {
    if (w <= 0 || h <= 0) return;
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = x + w > kW ? kW : x + w;
    int y1 = y + h > kH ? kH : y + h;
    for (int yy = y0; yy < y1; yy++) {
        uint16_t *row = &fb[yy * kW];
        for (int xx = x0; xx < x1; xx++) row[xx] = color;
    }
}

void rect_outline(int x, int y, int w, int h, uint16_t color) {
    for (int i = 0; i < w; i++) { px(x + i, y, color); px(x + i, y + h - 1, color); }
    for (int i = 0; i < h; i++) { px(x, y + i, color); px(x + w - 1, y + i, color); }
}

// Bytewise NOT of the swapped RGB565 — same "flash pressed" role as the
// OLED's XOR-invert (visually: colour → its complement, black ↔ white).
void rect_invert(int x, int y, int w, int h) {
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = x + w > kW ? kW : x + w;
    int y1 = y + h > kH ? kH : y + h;
    for (int yy = y0; yy < y1; yy++) {
        uint16_t *row = &fb[yy * kW];
        for (int xx = x0; xx < x1; xx++) row[xx] = (uint16_t)~row[xx];
    }
}

// 5×7 base font (shared oled_font.h) scaled up in integer steps. scale 2
// (10×14 glyphs, 20 columns) is the body size; scale 3 headers.
void draw_char(int x, int y, char ch, uint16_t color, int scale) {
    if (ch < 0x20 || ch > 0x7E) return;
    const uint8_t *g = kFont5x7[ch - 0x20];
    for (int col = 0; col < kFontW; col++) {
        const uint8_t bits = g[col];
        for (int row = 0; row < kFontH; row++) {
            if (bits & (1 << row)) {
                rect_filled(x + col * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}

void draw_text(int x, int y, const char *s, uint16_t color, int scale) {
    while (*s) {
        draw_char(x, y, *s++, color, scale);
        x += 6 * scale;
    }
}

int text_width(const char *s, int scale) {
    int n = 0; while (s[n]) n++;
    return n * 6 * scale - scale; // last glyph has no trailing gap
}

// Row-major MSB-first bitmap, scaled — same layout as the OLED icon table.
void draw_icon(int x, int y, const uint8_t *bitmap, int w, int h,
               uint16_t color, int scale) {
    const int row_bytes = (w + 7) / 8;
    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            const uint8_t byte = bitmap[row * row_bytes + (col / 8)];
            const uint8_t mask = (uint8_t)(1u << (7 - (col % 8)));
            if (byte & mask) rect_filled(x + col * scale, y + row * scale, scale, scale, color);
        }
    }
}

// 8x8 "link active" filled circle (drawn when DS5 is paired)
const uint8_t kIconLinkOn[8] = {
    0b00111100, 0b01111110, 0b11111111, 0b11111111,
    0b11111111, 0b11111111, 0b01111110, 0b00111100,
};
// 8x8 "link inactive" hollow circle (waiting for DS5)
const uint8_t kIconLinkOff[8] = {
    0b00111100, 0b01000010, 0b10000001, 0b10000001,
    0b10000001, 0b10000001, 0b01000010, 0b00111100,
};

// Battery: 104×16 body + nub, fill colour by level (charging = amber).
void draw_battery_icon(int x, int y, int pct, bool charging) {
    rect_outline(x, y, 104, 16, kGrey);
    rect_filled(x + 104, y + 4, 5, 8, kGrey);
    int fill = pct;
    if (fill < 0) fill = 0;
    if (fill > 100) fill = 100;
    uint16_t c = kGreen;
    if (charging) c = kAmber;
    else if (pct <= 20) c = kRed;
    else if (pct <= 50) c = kYellow;
    if (fill > 0) rect_filled(x + 2, y + 2, fill, 12, c);
}

// ── Layout helpers ──────────────────────────────────────────────────────────
// 240×240 grid: header (scale-3 title + accent rule), content rows every
// 22 px from y=40, footer hint line at y=222. The right edge column
// [228..240) is reserved for the A/B button chrome.
constexpr int kContentX = 8;
constexpr int kRowY0    = 40;
constexpr int kRowH     = 22;
inline int row_y(int i) { return kRowY0 + i * kRowH; }
constexpr int kFooterY  = 222;

void draw_header(const char *title, const char *tag) {
    draw_text(kContentX, 6, title, theme_accent(), 3);
    if (tag && tag[0]) {
        const int w = text_width(tag, 2);
        draw_text(kW - 14 - w, 10, tag, kGrey, 2);
    }
    rect_filled(0, 32, kW, 2, theme_accent());
}

void draw_footer(const char *hint) {
    draw_text(kContentX, kFooterY, hint, kGrey, 2);
}

// A/B button chrome on the right edge (the LCD-1.3's A and B buttons sit to
// the right of the glass): A = next '>', B = back '<'. Painted by flush_fb()
// on top of the rendered frame, mirroring the OLED's left-edge chrome.
void draw_button_chrome() {
    draw_char(kW - 12, 40, '>', theme_accent(), 2);
    draw_char(kW - 12, 186, '<', theme_accent(), 2);
}

void flush_fb_raw() { start_flush(); }

void flush_fb() {
    draw_button_chrome();
    start_flush();
}

// ── Frame/pacing + input state (semantics identical to oled.cpp) ────────────
uint32_t last_render_us = 0;
constexpr uint32_t kFrameUs = 100000;
bool key0_prev = true;
bool key1_prev = true;
uint32_t key0_t_us = 0;
uint32_t key1_t_us = 0;
constexpr uint32_t kDebounceUs = 20000;

// Single-press latch — armed on rising edge, fired on release (KEY0 = A).
bool key0_armed = false;

// KEY1 (= B) long-press detection (brightness cycling)
uint32_t key1_press_us = 0;
bool key1_was_pressed = false;
constexpr uint32_t kLongPressUs = 1500000;

// A + B chord held >= 1 s → watchdog_reboot (same as the OLED's KEY0+KEY1).
uint32_t chord_held_since_us = 0;
constexpr uint32_t kChordHoldUs = 1000000;

// Idle power ladder (Active → Dim → Off), thresholds from config in minutes.
// On the LCD the tiers drive the BACKLIGHT: Active = user brightness, Dim =
// kDimPct glow + walking dot, Off = backlight 0 + panel DISPOFF. 64-bit idle
// timer for the full 0..250 min range (same rationale as oled.cpp).
uint64_t last_activity_us = 0;
uint32_t last_input_hash = 0;
enum DispPowerState { DISP_ACTIVE, DISP_DIM, DISP_OFF };
DispPowerState disp_power_state = DISP_ACTIVE;
bool prev_bt_connected = false;

// Screen ordering — single source of truth, mirrors src/oled.cpp exactly
// (same names, same order, same count). Reorder by editing this block only.
constexpr int kScreenStatus    = 0;
constexpr int kScreenSlots     = 1;
constexpr int kScreenLightbar  = 2;
constexpr int kScreenTriggers  = 3;
constexpr int kScreenGyro      = 4;
constexpr int kScreenTouchpad  = 5;
constexpr int kScreenDiag      = 6;
constexpr int kScreenLatency   = 7;
constexpr int kScreenCpu       = 8;
constexpr int kScreenRssi      = 9;
constexpr int kScreenVU        = 10;
constexpr int kScreenSettings  = 11;
constexpr int kNumScreens      = 12;
int current_screen = 0;

// Lightbar mode cycle — keep numbering in sync with Config_body::lightbar_mode.
constexpr int kLbModeHost = 8;
constexpr int kNumLbModes = 9;

// Settings screen state (4-Player Edition list: 19 items).
constexpr int kNumSettingsItems = 19; // 8 fields + 3 auto-haptic + 2 screen-timeout + BT mic + Ctrl-wake + Player + PairLock + Reset + Wipe
constexpr int kSettingsAutoHapEnaIdx  = 8;
constexpr int kSettingsAutoHapGainIdx = 9;
constexpr int kSettingsAutoHapLpIdx   = 10;
constexpr int kSettingsScrDimIdx      = 11;
constexpr int kSettingsScrOffIdx      = 12;
constexpr int kSettingsBtMicIdx       = 13;
constexpr int kSettingsCtrlWakeIdx    = 14;
constexpr int kSettingsResetIdx       = 17;
constexpr int kSettingsWipeSlotsIdx   = 18;
Config_body settings_local{};
int settings_sel = 0;
bool settings_dirty = false;
bool settings_init_done = false;
uint8_t settings_last_dpad = 8;  // 8 = released
uint8_t settings_last_face = 0;
const char* settings_save_status = "";

// Factory-reset hold-Triangle-2s state (same UX as oled.cpp).
uint32_t settings_tri_press_us = 0;
bool settings_reset_triggered = false;
constexpr uint32_t kResetHoldUs = 2000000;

uint8_t lb_r = 0, lb_g = 0, lb_b = 0;
int lb_mode = kLbModeHost;
uint8_t lb_fav_r[4] = {255, 0,   0,   255};
uint8_t lb_fav_g[4] = {0,   255, 0,   255};
uint8_t lb_fav_b[4] = {0,   0,   255, 255};
uint8_t lb_last_face = 0;
bool lb_dirty = false;

uint32_t rumble_off_at_us = 0;
bool rumble_active = false;
constexpr uint32_t kRumbleBurstUs = 250000;

int trigger_preset = 0;
const char* const kTrigPresetNames[] = {"Off", "Feedback", "Weapon", "Vibration", "Bow", "Gallop", "Machine"};

uint8_t triggers_last_face = 0;
uint8_t lb_last_buttons = 0;
constexpr int kNumTrigPresets = 7;

// ── 5-way joystick (LCD-1.3 extra) ──────────────────────────────────────────
// Adds panel-local navigation on the Settings / Slots / Diagnostics screens:
// stick = D-pad-equivalent, CTRL press = Triangle-equivalent (select/save).
// Wired as a *substitute source* feeding the SAME edge-tracked variables the
// controller D-pad feeds, so the handlers' semantics are untouched; when a
// controller D-pad direction is active it wins. Square-hold actions (slot
// wipe) remain controller-only — the stick has no second button.
//
// Returns the DS5 D-pad nibble convention: 0=N 2=E 4=S 6=W, 8=released.
uint8_t joy_dpad() {
    if (!gpio_get(kPinJoyUp))    return 0;
    if (!gpio_get(kPinJoyRight)) return 2;
    if (!gpio_get(kPinJoyDown))  return 4;
    if (!gpio_get(kPinJoyLeft))  return 6;
    return 8;
}
bool joy_ctrl() { return !gpio_get(kPinJoyCtrl); }

// Merged control sources for the interactive screens. Controller present →
// its D-pad/face bits, with the joystick only filling in when the pad is
// released; no controller → joystick alone drives navigation.
uint8_t merged_dpad() {
    uint8_t dpad = 8;
    if (bt_is_connected()) dpad = (uint8_t)(interrupt_in_data[7] & 0x0F);
    if (dpad == 8) dpad = joy_dpad();
    return dpad;
}
uint8_t merged_face() {
    uint8_t face = 0;
    if (bt_is_connected()) face = (uint8_t)(interrupt_in_data[7] & 0xF0);
    if (joy_ctrl()) face |= 0x80; // CTRL = Triangle-equivalent
    return face;
}

// ── Controller TX helpers (verbatim from oled.cpp) ──────────────────────────
void send_rumble(uint8_t amplitude) {
    uint8_t pkt[78] = {};
    pkt[0] = 0x31;
    pkt[1] = 0x00;
    pkt[2] = 0x10;
    pkt[3] = 0x03;
    pkt[5] = amplitude;
    pkt[6] = amplitude;
    bt_write(pkt, sizeof(pkt));
}

void rumble_burst_tick(uint32_t now) {
    if (rumble_active && (int32_t)(now - rumble_off_at_us) >= 0) {
        send_rumble(0);
        rumble_active = false;
    }
}

// Trigger effect param format follows dualsensectl's reverse-engineering.
void send_trigger_effect(int preset) {
    uint8_t pkt[78] = {};
    pkt[0] = 0x31;
    pkt[2] = 0x10;
    pkt[3] = 0x0C; // valid_flag0: RIGHT/LEFT_TRIGGER_MOTOR_ENABLE

    uint8_t mode = 0x05; // OFF
    uint8_t p[9] = {0};

    switch (preset) {
        case 0:
            mode = 0x05;
            break;
        case 1: { // Feedback — all 10 zones at max strength 8
            mode = 0x21;
            const uint16_t active = 0x03FF;
            uint32_t strength = 0;
            for (int i = 0; i < 10; i++) strength |= (uint32_t)(7u << (3 * i));
            p[0] = active & 0xFF;
            p[1] = (active >> 8) & 0xFF;
            p[2] = strength & 0xFF;
            p[3] = (strength >> 8) & 0xFF;
            p[4] = (strength >> 16) & 0xFF;
            p[5] = (strength >> 24) & 0xFF;
            break;
        }
        case 2: { // Weapon — snap between positions 3 and 5, force 8
            mode = 0x25;
            const uint16_t start_stop = (1u << 3) | (1u << 5);
            p[0] = start_stop & 0xFF;
            p[1] = (start_stop >> 8) & 0xFF;
            p[2] = 7;
            break;
        }
        case 3: { // Vibration — all 10 zones amplitude 8, 30 Hz
            mode = 0x26;
            const uint16_t active = 0x03FF;
            uint32_t strength = 0;
            for (int i = 0; i < 10; i++) strength |= (uint32_t)(7u << (3 * i));
            p[0] = active & 0xFF;
            p[1] = (active >> 8) & 0xFF;
            p[2] = strength & 0xFF;
            p[3] = (strength >> 8) & 0xFF;
            p[4] = (strength >> 16) & 0xFF;
            p[5] = (strength >> 24) & 0xFF;
            p[8] = 30;
            break;
        }
        case 4: { // Bow — drawing resistance + snap at position 6
            mode = 0x22;
            const uint16_t start_stop = (1u << 2) | (1u << 6);
            const uint8_t force_pair = 7u | (7u << 3);
            p[0] = start_stop & 0xFF;
            p[1] = (start_stop >> 8) & 0xFF;
            p[2] = force_pair;
            break;
        }
        case 5: { // Galloping
            mode = 0x23;
            const uint16_t start_stop = (1u << 0) | (1u << 9);
            const uint8_t ratio = (5u & 0x07) | ((1u & 0x07) << 3);
            p[0] = start_stop & 0xFF;
            p[1] = (start_stop >> 8) & 0xFF;
            p[2] = ratio;
            p[3] = 5;
            break;
        }
        case 6: { // Machine gun
            mode = 0x27;
            const uint16_t start_stop = (1u << 1) | (1u << 8);
            const uint8_t force_pair = 7u | (7u << 3);
            p[0] = start_stop & 0xFF;
            p[1] = (start_stop >> 8) & 0xFF;
            p[2] = force_pair;
            p[3] = 20;
            p[4] = 0;
            break;
        }
    }

    pkt[13] = mode;
    for (int i = 0; i < 9; i++) pkt[14 + i] = p[i];
    pkt[24] = mode;
    for (int i = 0; i < 9; i++) pkt[25 + i] = p[i];

    bt_write(pkt, sizeof(pkt));
}

void send_lightbar_color(uint8_t r, uint8_t g, uint8_t b);

// ── Panel buttons: A = next screen, B = back / long-press brightness ────────
// Identical navigation semantics to the OLED's KEY0/KEY1 (same GPIOs). The
// long-press brightness cycle drives the BACKLIGHT PWM instead of a contrast
// register, persisting the same config field. The joystick counts as user
// activity so poking it wakes the panel like the buttons do.
void handle_buttons() {
    const uint32_t now = time_us_32();
    const bool k0 = gpio_get(kPinKey0);
    const bool k1 = gpio_get(kPinKey1);

    // A + B chord — both held >= kChordHoldUs triggers watchdog_reboot.
    const bool chord = !k0 && !k1;
    if (chord) {
        if (chord_held_since_us == 0) chord_held_since_us = now;
        key0_armed = false;
        key1_was_pressed = false;
        if ((now - chord_held_since_us) >= kChordHoldUs) {
            watchdog_reboot(0, 0, 0);
        }
    } else {
        chord_held_since_us = 0;
    }

    // A: arm on debounced press, fire "next screen" on release.
    if (!k0 && key0_prev && (now - key0_t_us) > kDebounceUs) {
        key0_t_us = now;
        key0_armed = true;
        last_activity_us = time_us_64();
    }
    if (k0 && !key0_prev && key0_armed) {
        key0_armed = false;
        current_screen = (current_screen + 1) % kNumScreens;
        last_render_us = 0;
        last_activity_us = time_us_64();
    }

    // B: short press = back; long press = brightness cycle (persisted).
    if (!k1 && key1_prev && (now - key1_t_us) > kDebounceUs) {
        key1_t_us = now;
        key1_press_us = now;
        key1_was_pressed = true;
        last_activity_us = time_us_64();
    }
    if (k1 && !key1_prev && key1_was_pressed) {
        key1_was_pressed = false;
        const uint32_t held = now - key1_press_us;
        last_activity_us = time_us_64();
        if (held > kLongPressUs) {
            bright_idx = (bright_idx + 1) % kNumBrightLevels;
            // Persist (same field/flow as the OLED build — issue #9) and keep
            // settings_local in sync so a Settings save can't clobber it.
            Config_body b = get_config();
            b.screen_brightness = (uint8_t)bright_idx;
            set_config(b);
            config_save();
            settings_local.screen_brightness = (uint8_t)bright_idx;
        } else {
            current_screen = (current_screen - 1 + kNumScreens) % kNumScreens;
            last_render_us = 0;
        }
    }

    key0_prev = k0;
    key1_prev = k1;

    // Joystick activity wakes/keeps the panel awake (like the buttons; the
    // per-screen handlers do the actual navigation). Edge-ish check is free.
    static uint8_t joy_prev = 8;
    static bool ctrl_prev = false;
    const uint8_t jd = joy_dpad();
    const bool jc = joy_ctrl();
    if (jd != joy_prev || jc != ctrl_prev) {
        joy_prev = jd;
        ctrl_prev = jc;
        last_activity_us = time_us_64();
    }
}

// ── Charge ETA tracker (verbatim from oled.cpp) ─────────────────────────────
// Times each 10% battery step while charging, taper-corrected, median over
// the last 5 steps. See oled.cpp for the full derivation commentary.
struct ChargeEta {
    bool charging;
    bool valid;
    bool provisional;
    int  minutes;
};
ChargeEta g_charge_eta{};

constexpr float kDefaultStepUs = 15.0f * 60.0f * 1000000.0f;
constexpr float kMaxStepUs = 30.0f * 60.0f * 1000000.0f;

float charge_step_weight(int to_level) {
    if (to_level >= 10) return 2.2f;  // 90→100% (constant-voltage tail)
    if (to_level == 9)  return 1.5f;  // 80→90%  (taper begins)
    return 1.0f;                      // bulk constant-current region
}

void sample_charge_eta() {
    constexpr int kRing = 5;
    static float    ring[kRing] = {0};
    static int      ring_count = 0;
    static int      ring_head = 0;
    static int      cur_step = -1;
    static uint64_t step_start_us = 0;
    static bool     was_charging = false;
    static bool     first_step_pending = false;

    const uint8_t pwr   = interrupt_in_data[52];
    int           step  = pwr & 0x0F;
    if (step > 10) step = 10;
    const uint8_t pstate = pwr >> 4;
    const bool charging = bt_is_connected() && (pstate == 1);

    if (!charging) {
        g_charge_eta = ChargeEta{};
        ring_count = ring_head = 0;
        cur_step = -1;
        was_charging = false;
        return;
    }

    const uint64_t now = time_us_64();
    if (!was_charging) {
        cur_step = step;
        step_start_us = now;
        ring_count = ring_head = 0;
        first_step_pending = true;
        was_charging = true;
    } else if (step == cur_step + 1) {
        const float dur = (float)(now - step_start_us);
        if (first_step_pending) {
            first_step_pending = false;
        } else {
            float be = dur / charge_step_weight(step);
            if (be > kMaxStepUs) be = kMaxStepUs;
            ring[ring_head] = be;
            ring_head = (ring_head + 1) % kRing;
            if (ring_count < kRing) ring_count++;
        }
        cur_step = step;
        step_start_us = now;
    } else if (step != cur_step) {
        cur_step = step;
        step_start_us = now;
        first_step_pending = false;
    }

    g_charge_eta.charging = true;
    if (cur_step < 10) {
        const bool measured = (ring_count > 0);
        float bulk;
        if (measured) {
            float tmp[kRing];
            for (int i = 0; i < ring_count; i++) tmp[i] = ring[i];
            for (int i = 1; i < ring_count; i++) {
                const float v = tmp[i];
                int j = i - 1;
                while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
                tmp[j + 1] = v;
            }
            bulk = tmp[ring_count / 2];
        } else {
            bulk = kDefaultStepUs;
        }
        float rem_us = 0.0f;
        for (int L = cur_step + 1; L <= 10; L++) rem_us += bulk * charge_step_weight(L);
        int mins = (int)(rem_us / 60000000.0f + 0.5f);
        if (mins < 0)   mins = 0;
        if (mins > 999) mins = 999;
        g_charge_eta.valid = true;
        g_charge_eta.provisional = !measured;
        g_charge_eta.minutes = mins;
    } else {
        g_charge_eta.valid = true;
        g_charge_eta.provisional = false;
        g_charge_eta.minutes = 0;
    }
}

// ── IMU calibration (DS5 feature report 0x05; verbatim from oled.cpp) ───────
struct ImuCal { int16_t bias; float sens; };  // 0..2 gyro P/Y/R, 3..5 accel X/Y/Z
ImuCal g_imu_cal[6];
bool   g_imu_cal_valid = false;
bool   g_imu_cal_tried = false;

constexpr float kGyroResPerDeg = 1024.0f;
constexpr float kAccelResPerG  = 8192.0f;

inline int16_t cal_ld16(const std::vector<uint8_t>& d, int i) {
    return (int16_t)((uint16_t)d[i] | ((uint16_t)d[i + 1] << 8));
}

__attribute__((noinline))
void imu_cal_parse(const std::vector<uint8_t>& d) {
    g_imu_cal_valid = false;
    if (d.size() < 35) return;                 // SDL requires >= 35 calibration bytes

    const int16_t gPB = cal_ld16(d, 1),  gYB = cal_ld16(d, 3),  gRB = cal_ld16(d, 5);
    const int16_t gPp = cal_ld16(d, 7),  gPm = cal_ld16(d, 9);
    const int16_t gYp = cal_ld16(d, 11), gYm = cal_ld16(d, 13);
    const int16_t gRp = cal_ld16(d, 15), gRm = cal_ld16(d, 17);
    const int16_t gSp = cal_ld16(d, 19), gSm = cal_ld16(d, 21);
    const int16_t aXp = cal_ld16(d, 23), aXm = cal_ld16(d, 25);
    const int16_t aYp = cal_ld16(d, 27), aYm = cal_ld16(d, 29);
    const int16_t aZp = cal_ld16(d, 31), aZm = cal_ld16(d, 33);

    const float num = (float)(gSp + gSm) * kGyroResPerDeg;
    g_imu_cal[0] = { gPB, num / (float)(gPp - gPm) };
    g_imu_cal[1] = { gYB, num / (float)(gYp - gYm) };
    g_imu_cal[2] = { gRB, num / (float)(gRp - gRm) };

    int16_t r;
    r = aXp - aXm; g_imu_cal[3] = { (int16_t)(aXp - r / 2), 2.0f * kAccelResPerG / (float)r };
    r = aYp - aYm; g_imu_cal[4] = { (int16_t)(aYp - r / 2), 2.0f * kAccelResPerG / (float)r };
    r = aZp - aZm; g_imu_cal[5] = { (int16_t)(aZp - r / 2), 2.0f * kAccelResPerG / (float)r };

    for (int i = 0; i < 6; i++) {
        const float divisor = (i < 3) ? 64.0f : 1.0f;
        const int   ab      = g_imu_cal[i].bias < 0 ? -g_imu_cal[i].bias : g_imu_cal[i].bias;
        float       gain    = 1.0f - g_imu_cal[i].sens / divisor;
        if (gain < 0) gain = -gain;
        if (ab > 1024 || gain > 0.5f) return;  // leave g_imu_cal_valid = false
    }
    g_imu_cal_valid = true;
}

void imu_cal_service() {
    if (!bt_is_connected()) { g_imu_cal_valid = false; g_imu_cal_tried = false; return; }
    if (g_imu_cal_tried) return;
    auto d = bt_peek_feature(0x05);
    if (d.size() < 35) return;                 // not arrived yet — retry next frame
    imu_cal_parse(d);
    g_imu_cal_tried = true;
}

inline int16_t imu_apply(int index, int16_t raw) {
    if (!g_imu_cal_valid) return raw;
    return (int16_t)((float)(raw - g_imu_cal[index].bias) * g_imu_cal[index].sens);
}

// ── Lightbar engine (verbatim from oled.cpp) ────────────────────────────────
void send_lightbar_color(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t pkt[78] = {};
    pkt[0] = 0x31;
    pkt[2] = 0x10;
    pkt[4] = 0x04; // valid_flag1: LIGHTBAR_CONTROL_ENABLE (bit 2)
    pkt[47] = r;
    pkt[48] = g;
    pkt[49] = b;
    bt_write(pkt, sizeof(pkt));
}

static const int8_t kSine32[32] = {
    0,   24,   49,   70,   90,  106,  117,  125,  127,  125,  117,  106,   90,   70,   49,   24,
    0,  -24,  -49,  -70,  -90, -106, -117, -125, -127, -125, -117, -106,  -90,  -70,  -49,  -24,
};
int sin_lut(uint8_t a) { return kSine32[(a >> 3) & 0x1F]; }

void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (h >= 360) h %= 360;
    const uint8_t region = (uint8_t)(h / 60);
    const uint16_t remainder = (uint16_t)((h - region * 60u) * 256u / 60u);
    const uint8_t p = (uint8_t)(((uint16_t)v * (255u - s)) >> 8);
    const uint8_t q = (uint8_t)(((uint16_t)v * (255u - (((uint16_t)s * remainder) >> 8))) >> 8);
    const uint8_t t = (uint8_t)(((uint16_t)v * (255u - (((uint16_t)s * (255u - remainder)) >> 8))) >> 8);
    switch (region) {
        case 0:  *r = v; *g = t; *b = p; break;
        case 1:  *r = q; *g = v; *b = p; break;
        case 2:  *r = p; *g = v; *b = t; break;
        case 3:  *r = p; *g = q; *b = v; break;
        case 4:  *r = t; *g = p; *b = v; break;
        default: *r = v; *g = p; *b = q; break;
    }
}

const char* lb_mode_tag(int mode) {
    switch (mode) {
        case 0: return "[LIVE]";
        case 1: return "[FAV0]";
        case 2: return "[FAV1]";
        case 3: return "[FAV2]";
        case 4: return "[FAV3]";
        case 5: return "[BREA]";
        case 6: return "[RAIN]";
        case 7: return "[FADE]";
        case 8: return "[HOST]";
        default: return "[????]";
    }
}

void lightbar_handle_input() {
    if (!bt_is_connected()) { lb_last_buttons = 0; return; }
    const uint8_t btns   = interrupt_in_data[8];
    const bool r1_now    = (btns & 0x02) != 0;
    const bool r1_prev   = (lb_last_buttons & 0x02) != 0;
    if (r1_now && !r1_prev) {
        lb_mode = (lb_mode + 1) % kNumLbModes;
        lb_dirty = true; // persisted on leaving the Lightbar screen
    }
    lb_last_buttons = btns;
}

// noinline: keeps the float/HSV literals out of the loop's literal pool —
// same Thumb 4 KB reach constraint documented in oled.cpp / CLAUDE.md.
__attribute__((noinline))
void lightbar_compute_mode(int mode, uint32_t now_ms) {
    if (mode == 0) {
        // LIVE: tilt -> RGB
        int16_t ax, ay, az;
        memcpy(&ax, &interrupt_in_data[21], 2);
        memcpy(&ay, &interrupt_in_data[23], 2);
        memcpy(&az, &interrupt_in_data[25], 2);
        ax = imu_apply(3, ax);
        ay = imu_apply(4, ay);
        az = imu_apply(5, az);
        const int rr = ((int)ax + 8192) * 255 / 16384;
        const int gg = ((int)ay + 8192) * 255 / 16384;
        const int bb = ((int)az + 8192) * 255 / 16384;
        lb_r = (uint8_t)(rr < 0 ? 0 : rr > 255 ? 255 : rr);
        lb_g = (uint8_t)(gg < 0 ? 0 : gg > 255 ? 255 : gg);
        lb_b = (uint8_t)(bb < 0 ? 0 : bb > 255 ? 255 : bb);
    } else if (mode <= 4) {
        const int slot = mode - 1;
        lb_r = lb_fav_r[slot];
        lb_g = lb_fav_g[slot];
        lb_b = lb_fav_b[slot];
    } else if (mode == 5) {
        // BREATHING: FAV0 modulated by a ~3 s sine
        const uint8_t phase = (uint8_t)(now_ms / 12);
        const int s = sin_lut(phase);
        const uint16_t scale = (uint16_t)(32 + (s + 127) / 2);
        lb_r = (uint8_t)((lb_fav_r[0] * scale) / 255);
        lb_g = (uint8_t)((lb_fav_g[0] * scale) / 255);
        lb_b = (uint8_t)((lb_fav_b[0] * scale) / 255);
    } else if (mode == 6) {
        // RAINBOW: hue sweep over ~6 s
        const uint16_t hue = (uint16_t)((now_ms / 17) % 360);
        hsv_to_rgb(hue, 255, 255, &lb_r, &lb_g, &lb_b);
    } else {
        // FADE between FAV slots, 2 s per slot
        const uint32_t kSlotMs = 2000;
        const uint32_t total = now_ms % (4 * kSlotMs);
        const int slot = (int)(total / kSlotMs);
        const int next = (slot + 1) & 3;
        const uint16_t blend = (uint16_t)(((total - slot * kSlotMs) * 256u) / kSlotMs);
        lb_r = (uint8_t)((lb_fav_r[slot] * (255 - blend) + lb_fav_r[next] * blend) / 255);
        lb_g = (uint8_t)((lb_fav_g[slot] * (255 - blend) + lb_fav_g[next] * blend) / 255);
        lb_b = (uint8_t)((lb_fav_b[slot] * (255 - blend) + lb_fav_b[next] * blend) / 255);
    }
}

// Single owner of the controller LED — same priority ladder as oled.cpp:
// charging pulse > selected mode > HOST hand-off. See oled.cpp for rationale.
__attribute__((noinline))
void lightbar_service() {
    if (!bt_is_connected()) { g_lightbar_override = false; return; }
    const uint32_t now_ms = time_us_32() / 1000;

    if (g_charge_eta.charging) {
        const uint8_t  phase = (uint8_t)(now_ms / 18);
        const int      s     = sin_lut(phase);
        const uint16_t scale = (uint16_t)(24 + ((s + 127) * 216) / 254);
        lb_r = (uint8_t)((255u * scale) / 255u);
        lb_g = (uint8_t)((100u * scale) / 255u);
        lb_b = 0;
    } else if (lb_mode == kLbModeHost) {
        state_get_led(&lb_r, &lb_g, &lb_b);
        g_lightbar_override = false;
        return;
    } else {
        lightbar_compute_mode(lb_mode, now_ms);
    }

    g_lightbar_override = true;
    state_set_led(lb_r, lb_g, lb_b);
    if (!spk_active) {
        // Active push, skipped during audio — the 0x36 frames already carry
        // state[]'s LED and must not have 0x31s slipped between them.
        send_lightbar_color(lb_r, lb_g, lb_b);
    }
}

void lightbar_load_config() {
    const Config_body& c = get_config();
    lb_mode = c.lightbar_mode;
    if (lb_mode < 0 || lb_mode >= kNumLbModes) lb_mode = kLbModeHost;
    for (int i = 0; i < 4; i++) {
        lb_fav_r[i] = c.lb_fav_r[i];
        lb_fav_g[i] = c.lb_fav_g[i];
        lb_fav_b[i] = c.lb_fav_b[i];
    }
    lb_dirty = false;
}

void lightbar_save_config() {
    Config_body b = get_config();
    b.lightbar_mode = (uint8_t)lb_mode;
    for (int i = 0; i < 4; i++) {
        b.lb_fav_r[i] = lb_fav_r[i];
        b.lb_fav_g[i] = lb_fav_g[i];
        b.lb_fav_b[i] = lb_fav_b[i];
    }
    set_config(b);
    config_save();
    lb_dirty = false;
}

// ── Diagnostics rates (verbatim from oled.cpp) ──────────────────────────────
int   diag_scroll = 0;
uint8_t diag_last_dpad = 8;

struct DiagRates {
    uint32_t usb_rate;
    uint32_t bt_rate;
    uint32_t mic_rate;
    uint32_t bt31_rate;
};
DiagRates g_diag_rates{};

void sample_diag_rates() {
    static uint32_t prev_us_frames = 0, prev_bt_packets = 0, prev_mic_frames = 0, prev_bt31 = 0;
    static uint32_t prev_sample_us = 0;
    const uint32_t now_us = time_us_32();
    const uint32_t cur_us_frames  = audio_usb_frames();
    const uint32_t cur_bt_packets = audio_bt_packets();
    const uint32_t cur_mic_frames = audio_mic_frames();
    const uint32_t cur_bt31       = bt_31_packet_count();
    if (prev_sample_us != 0 && now_us > prev_sample_us) {
        const uint32_t dt_us = now_us - prev_sample_us;
        if (dt_us > 0) {
            g_diag_rates.usb_rate  = (uint32_t)(((uint64_t)(cur_us_frames  - prev_us_frames)  * 1000000u) / dt_us);
            g_diag_rates.bt_rate   = (uint32_t)(((uint64_t)(cur_bt_packets - prev_bt_packets) * 1000000u) / dt_us);
            g_diag_rates.mic_rate  = (uint32_t)(((uint64_t)(cur_mic_frames - prev_mic_frames) * 1000000u) / dt_us);
            g_diag_rates.bt31_rate = (uint32_t)(((uint64_t)(cur_bt31       - prev_bt31)       * 1000000u) / dt_us);
        }
    }
    prev_us_frames  = cur_us_frames;
    prev_bt_packets = cur_bt_packets;
    prev_mic_frames = cur_mic_frames;
    prev_bt31       = cur_bt31;
    prev_sample_us  = now_us;
}

constexpr int kNumDiagRows = 12;
__attribute__((noinline))
void format_diag_row(int idx, char* line, size_t n) {
    switch (idx) {
        case 0: {
            const uint32_t s = time_us_32() / 1000000u;
            snprintf(line, n, "Up:%luh %02lum %02lus",
                     (unsigned long)(s / 3600u),
                     (unsigned long)((s / 60u) % 60u),
                     (unsigned long)(s % 60u));
            break;
        }
        case 1:
            snprintf(line, n, "BT: %s", bt_is_connected() ? "connected" : "waiting");
            break;
        case 2:
            snprintf(line, n, "host02: %lu", (unsigned long)host_out02_total());
            break;
        case 3:
            snprintf(line, n, "trig %lu / tx %lu",
                     (unsigned long)host_out02_trig_allow(),
                     (unsigned long)host_out02_to_bt());
            break;
        case 4:
            snprintf(line, n, "trig fold: %lu", (unsigned long)host_out02_trig_folded());
            break;
        case 5:
            snprintf(line, n, "BT31 in: %lu/s", (unsigned long)g_diag_rates.bt31_rate);
            break;
        case 6:
            snprintf(line, n, "USB aud: %lu/s", (unsigned long)g_diag_rates.usb_rate);
            break;
        case 7:
            snprintf(line, n, "BT32 out: %lu/s", (unsigned long)g_diag_rates.bt_rate);
            break;
        case 8:
            snprintf(line, n, "Mic in: %lu/s", (unsigned long)g_diag_rates.mic_rate);
            break;
        case 9:
            snprintf(line, n, "Mic dec=%ld w=%u",
                     (long)audio_mic_last_decoded(),
                     (unsigned)audio_mic_last_wrote());
            break;
        case 10:
            snprintf(line, n, "Mic PLC: %lu", (unsigned long)audio_mic_plc_frames());
            break;
        case 11: {
            uint8_t pfx[6]; bt_31_mic_prefix(pfx);
            snprintf(line, n, "%02X %02X %02X %02X %02X %02X",
                     pfx[0], pfx[1], pfx[2], pfx[3], pfx[4], pfx[5]);
            break;
        }
        default:
            line[0] = '\0';
            break;
    }
}

// D-pad (or joystick) scrolls the diagnostics viewport. Same edge semantics
// as oled.cpp; merged_dpad() lets the panel joystick work controller-less.
void diag_handle_input(int visible) {
    const uint8_t dpad = merged_dpad();
    if (dpad != diag_last_dpad && dpad != 8) {
        if      (dpad == 0) diag_scroll--; // up
        else if (dpad == 4) diag_scroll++; // down
    }
    diag_last_dpad = dpad;
    const int max_top = (kNumDiagRows > visible) ? (kNumDiagRows - visible) : 0;
    if (diag_scroll < 0) diag_scroll = 0;
    if (diag_scroll > max_top) diag_scroll = max_top;
}

// ── Screens ─────────────────────────────────────────────────────────────────
// Every screen carries the same information as its OLED counterpart (plus
// colour), and every render_* stays noinline — the Thumb literal-pool linker
// constraint documented in CLAUDE.md bites harder with more code, not less.

// Status. Colour upgrades: player badge in the player's colour, battery fill
// by charge level, pressed controls light up in the accent colour.
__attribute__((noinline)) void render_screen() {
    fb_clear();
    const bool connected = bt_is_connected();
    const uint16_t accent = theme_accent();

    draw_header("DS5 Bridge", nullptr);
    // Version + link state + player badge share the first row.
    draw_text(kContentX, 42, FIRMWARE_VERSION, kGrey, 2);
    draw_icon(200, 40, connected ? kIconLinkOn : kIconLinkOff, 8, 8,
              connected ? kGreen : kGrey, 2);
    const uint8_t pid = get_config().player_id;
    if (pid >= 1 && pid <= 8) {
        char pbadge[4];
        snprintf(pbadge, sizeof(pbadge), "P%u", pid);
        rect_filled(150, 38, 34, 20, accent);
        draw_text(155, 41, pbadge, kBlack, 2);
    }

    if (connected) {
        uint8_t a[6];
        bt_get_addr(a);
        char buf[24];
        snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                 a[0], a[1], a[2], a[3], a[4], a[5]);
        draw_text(kContentX, 64, buf, kGrey, 2);

        const uint8_t pwr = interrupt_in_data[52];
        int pct = (pwr & 0x0F) * 10;
        if (pct > 100) pct = 100;
        const uint8_t pstate = pwr >> 4;
        char marker = ' ';
        if (pstate == 1) marker = '+';        // Charging
        else if (pstate == 2) marker = '*';   // Complete
        else if (pstate >= 0xA) marker = '!'; // Error
        char bbuf[16];
        snprintf(bbuf, sizeof(bbuf), "%3d%%%c", pct, marker);
        draw_text(kContentX, 88, bbuf, kWhite, 2);
        draw_battery_icon(78, 86, pct, pstate == 1);

        // Charge ETA to the right of the battery (same provisional-"?" logic
        // as the OLED build — see sample_charge_eta()).
        if (g_charge_eta.charging) {
            char ebuf[8];
            if (g_charge_eta.valid)
                snprintf(ebuf, sizeof(ebuf), "~%dm%s", g_charge_eta.minutes,
                         g_charge_eta.provisional ? "?" : "");
            else
                snprintf(ebuf, sizeof(ebuf), "~--m");
            draw_text(192, 88, ebuf, kAmber, 2);
        }

        // Shoulder bars above the stick boxes.
        const uint8_t b8 = interrupt_in_data[8];
        if (b8 & 0x01) rect_filled(kContentX, 116, 40, 6, accent);
        else           rect_outline(kContentX, 116, 40, 6, kGrey);   // L1
        if (b8 & 0x02) rect_filled(184, 116, 40, 6, accent);
        else           rect_outline(184, 116, 40, 6, kGrey);         // R1

        // Analog sticks: 64×64 boxes, live dot, L3/R3 click = invert.
        rect_outline(kContentX, 128, 64, 64, kGrey);
        const int lx = kContentX + 2 + (interrupt_in_data[0] * 59) / 255;
        const int ly = 130 + (interrupt_in_data[1] * 59) / 255;
        rect_filled(lx - 2, ly - 2, 5, 5, kWhite);
        if (b8 & 0x40) rect_invert(kContentX, 128, 64, 64);          // L3

        rect_outline(160, 128, 64, 64, kGrey);
        const int rx = 162 + (interrupt_in_data[2] * 59) / 255;
        const int ry = 130 + (interrupt_in_data[3] * 59) / 255;
        rect_filled(rx - 2, ry - 2, 5, 5, kWhite);
        if (b8 & 0x80) rect_invert(160, 128, 64, 64);                // R3

        // L2/R2 analog trigger columns (fill from bottom, accent).
        rect_outline(80, 128, 10, 64, kGrey);
        const int l2_fill = (interrupt_in_data[4] * 60) / 255;
        if (l2_fill > 0) rect_filled(82, 190 - l2_fill, 6, l2_fill, accent);
        rect_outline(150, 128, 10, 64, kGrey);
        const int r2_fill = (interrupt_in_data[5] * 60) / 255;
        if (r2_fill > 0) rect_filled(152, 190 - r2_fill, 6, r2_fill, accent);

        const uint8_t b7 = interrupt_in_data[7];

        // D-pad diamond (lit for primary + diagonals), centre column.
        const int dp = b7 & 0x0F;
        const bool dp_n = (dp == 7 || dp == 0 || dp == 1);
        const bool dp_e = (dp == 1 || dp == 2 || dp == 3);
        const bool dp_s = (dp == 3 || dp == 4 || dp == 5);
        const bool dp_w = (dp == 5 || dp == 6 || dp == 7);
        const int dcx = 120, dcy = 146;
        auto dot = [&](int dx, int dy, bool on) {
            if (on) rect_filled(dcx + dx - 4, dcy + dy - 4, 9, 9, accent);
            else    rect_outline(dcx + dx - 4, dcy + dy - 4, 9, 9, kGrey);
        };
        dot(0, -13, dp_n);
        dot(13,  0, dp_e);
        dot(0,  13, dp_s);
        dot(-13, 0, dp_w);

        // Face buttons diamond below the d-pad.
        const int fcx = 120, fcy = 196;
        auto sq = [&](int dx, int dy, bool on) {
            if (on) rect_filled(fcx + dx - 4, fcy + dy - 4, 9, 9, accent);
            else    rect_outline(fcx + dx - 4, fcy + dy - 4, 9, 9, kGrey);
        };
        sq(0, -13, b7 & 0x80); // Triangle
        sq(13,  0, b7 & 0x40); // Circle
        sq(0,  13, b7 & 0x20); // Cross
        sq(-13, 0, b7 & 0x10); // Square
    } else {
        // Max 18 chars per line at scale 2: content starts at x=8, one glyph
        // cell is 12 px, and [228..240) is reserved for the A/B chrome.
        draw_text(kContentX, row_y(2),     "Pair DualSense:", kWhite, 2);
        draw_text(kContentX, row_y(3),     "1. Hold Create+PS", kGrey, 2);
        draw_text(kContentX, row_y(4),     "2. Wait for the", kGrey, 2);
        draw_text(kContentX, row_y(5),     "   light bar to", kGrey, 2);
        draw_text(kContentX, row_y(6),     "   flash blue", kGrey, 2);
    }

    flush_fb();
}

__attribute__((noinline)) void render_screen_rssi() {
    fb_clear();
    draw_header("BT Signal", nullptr);
    if (bt_is_connected()) {
        int8_t rssi = 0;
        bt_get_signal_strength(&rssi);
        char buf[24];
        snprintf(buf, sizeof(buf), "RSSI: %d dBm", (int)rssi);
        draw_text(kContentX, row_y(0), buf, kWhite, 2);

        // Map RSSI range -90..-40 dBm to 0..100% bar
        int pct = ((int)rssi + 90) * 100 / 50;
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        snprintf(buf, sizeof(buf), "Quality: %d%%", pct);
        draw_text(kContentX, row_y(1), buf, kWhite, 2);

        const uint16_t qc = (pct >= 60) ? kGreen : (pct >= 30) ? kYellow : kRed;
        rect_outline(kContentX, row_y(2), 216, 18, kGrey);
        const int fill = (pct * 212) / 100;
        if (fill > 0) rect_filled(kContentX + 2, row_y(2) + 2, fill, 14, qc);

        const char *label = "Poor";
        if (rssi > -55) label = "Excellent";
        else if (rssi > -65) label = "Good";
        else if (rssi > -75) label = "Fair";
        snprintf(buf, sizeof(buf), "Link: %s", label);
        draw_text(kContentX, row_y(3) + 8, buf, qc, 2);
    } else {
        draw_text(kContentX, row_y(3), "(no controller)", kGrey, 2);
    }
    flush_fb();
}

__attribute__((noinline)) void render_screen_diag() {
    fb_clear();
    draw_header("Diagnostics", nullptr);

    sample_diag_rates();
    constexpr int kVisible = 8;
    diag_handle_input(kVisible);

    char line[28];
    for (int i = 0; i < kVisible && diag_scroll + i < kNumDiagRows; i++) {
        format_diag_row(diag_scroll + i, line, sizeof(line));
        draw_text(kContentX, row_y(i), line, kWhite, 2);
    }

    // Scroll indicators on the right edge, clear of the A/B chrome rows.
    if (diag_scroll > 0)                       draw_text(kW - 12, 62,  "^", kGrey, 2);
    if (diag_scroll + kVisible < kNumDiagRows) draw_text(kW - 12, 164, "v", kGrey, 2);

    draw_footer("DPad/Joy = scroll");
    flush_fb();
}

// Latency telemetry (src/latency.h) — same numbers as the OLED variant and
// the 0xFD report; see render_screen_latency in oled.cpp for field notes.
// µs → "x.xx" ms string (2 dp), clamped at "99.9+" — see oled.cpp twin.
static void fmt_ms(char *dst, size_t n, uint32_t us) {
    if (us >= 99990) { snprintf(dst, n, "99.9+"); return; }
    snprintf(dst, n, "%lu.%02lu", (unsigned long)(us / 1000),
             (unsigned long)((us % 1000) / 10));
}

__attribute__((noinline)) void render_screen_latency() {
    fb_clear();

    LatencyStats ls{};
    latency_get(&ls);

    const uint8_t cfg_mode = get_config().polling_rate_mode % 3;
    static const char* const kPollNames[3] = {"250Hz", "500Hz", "1kHz"};
    char tag[10];
    snprintf(tag, sizeof(tag), "[%s]", kPollNames[cfg_mode]);
    draw_header("Latency", tag);

    char buf[28];
    snprintf(buf, sizeof(buf), "BT in : %lu/s", (unsigned long)ls.bt_rate);
    draw_text(kContentX, row_y(0), buf, kWhite, 2);
    snprintf(buf, sizeof(buf), "USBout: %lu/s", (unsigned long)ls.usb_rate);
    draw_text(kContentX, row_y(1), buf, kWhite, 2);
    // Dongle transit in ms (BT arrival -> USB accept); 18-char scale-2 budget.
    char a[8], m[8], p[8];
    fmt_ms(a, sizeof(a), ls.transit_avg_us);
    fmt_ms(m, sizeof(m), ls.transit_max_us);
    fmt_ms(p, sizeof(p), ls.transit_peak_us);
    snprintf(buf, sizeof(buf), "Lat avg %sms", a);
    draw_text(kContentX, row_y(2), buf, theme_accent(), 2);
    snprintf(buf, sizeof(buf), "max %s pk %s", m, p);
    draw_text(kContentX, row_y(3), buf, kWhite, 2);
    fmt_ms(a, sizeof(a), ls.bt_gap_min_us);
    fmt_ms(m, sizeof(m), ls.bt_gap_max_us);
    snprintf(buf, sizeof(buf), "gap %s-%sms", a, m);
    draw_text(kContentX, row_y(4), buf, kWhite, 2);
    fmt_ms(a, sizeof(a), latency_display_busy_max_us());
    snprintf(buf, sizeof(buf), "Disp max %sms", a);
    draw_text(kContentX, row_y(5), buf, kWhite, 2);

    if (g_usb_active_poll_mode != 0xFF && g_usb_active_poll_mode != cfg_mode) {
        // A stale bInterval invalidates every number a latency hunt cares about.
        draw_text(kContentX, kFooterY, "Poll chg = replug", kRed, 2);
    } else {
        draw_footer("avg/max = last 1s");
    }
    flush_fb();
}

__attribute__((noinline)) void render_screen_cpu(bool entered) {
    fb_clear();
    draw_header("CPU / Clock", nullptr);

    char buf[24];

    const uint32_t set_khz = (uint32_t)SYS_CLOCK_KHZ;
    snprintf(buf, sizeof(buf), "Set : %lu MHz", (unsigned long)(set_khz / 1000u));
    draw_text(kContentX, row_y(0), buf, kWhite, 2);

    // frequency_count_khz busy-waits a few ms — measure ONCE on screen entry
    // and cache (clk_sys never changes after boot). Same as the OLED build.
    static uint32_t cached_real_khz = 0;
    if (entered || cached_real_khz == 0) {
        cached_real_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    }
    const uint32_t real_khz = cached_real_khz;
    snprintf(buf, sizeof(buf), "Real: %lu.%01lu MHz",
             (unsigned long)(real_khz / 1000u),
             (unsigned long)((real_khz % 1000u) / 100u));
    draw_text(kContentX, row_y(1), buf, kWhite, 2);

    const int vcode = (int)vreg_get_voltage();
    if (vcode >= 0 && vcode <= 0b01111) {
        const unsigned mv = 550u + 50u * (unsigned)vcode;
        snprintf(buf, sizeof(buf), "Vcore: %u.%02u V", mv / 1000u, (mv % 1000u) / 10u);
    } else {
        snprintf(buf, sizeof(buf), "Vcore: code %d", vcode);
    }
    draw_text(kContentX, row_y(2), buf, kWhite, 2);

    const uint16_t raw = cpu_temp_raw_smoothed();
    const float volts = (float)raw * 3.3f / 4096.0f;
    const float temp_c = 27.0f - (volts - 0.706f) / 0.001721f;
    const int t10 = (int)(temp_c * 10.0f + (temp_c >= 0 ? 0.5f : -0.5f));
    snprintf(buf, sizeof(buf), "Temp : %d.%d C", t10 / 10,
             (t10 < 0 ? -t10 : t10) % 10);
    draw_text(kContentX, row_y(3), buf,
              (t10 >= 700) ? kRed : (t10 >= 600) ? kYellow : kWhite, 2);

    flush_fb();
}

// △ rising edge cycles trigger_preset — identical input handling to oled.cpp.
void triggers_handle_input() {
    if (!bt_is_connected()) { triggers_last_face = 0; return; }
    const uint8_t face = interrupt_in_data[7] & 0xF0;
    const bool tri_now  = (face & 0x80) != 0;
    const bool tri_prev = (triggers_last_face & 0x80) != 0;
    if (tri_now && !tri_prev) {
        trigger_preset = (trigger_preset + 1) % kNumTrigPresets;
        send_trigger_effect(trigger_preset);
    }
    triggers_last_face = face;
}

__attribute__((noinline)) void render_screen_triggers() {
    triggers_handle_input();
    fb_clear();
    draw_header("Trigger Test", nullptr);
    const uint16_t accent = theme_accent();

    char buf[24];
    snprintf(buf, sizeof(buf), "Mode: %s", kTrigPresetNames[trigger_preset]);
    draw_text(kContentX, row_y(0), buf, accent, 2);

    if (bt_is_connected()) {
        const uint8_t l2 = interrupt_in_data[4];
        const uint8_t r2 = interrupt_in_data[5];
        snprintf(buf, sizeof(buf), "L2:%3d  R2:%3d", l2, r2);
        draw_text(kContentX, row_y(1), buf, kWhite, 2);

        rect_outline(kContentX, row_y(2), 216, 18, kGrey);
        const int lfill = (l2 * 212) / 255;
        if (lfill > 0) rect_filled(kContentX + 2, row_y(2) + 2, lfill, 14, accent);
        rect_outline(kContentX, row_y(3), 216, 18, kGrey);
        const int rfill = (r2 * 212) / 255;
        if (rfill > 0) rect_filled(kContentX + 2, row_y(3) + 2, rfill, 14, accent);
    } else {
        draw_text(kContentX, row_y(2), "(no controller)", kGrey, 2);
    }

    draw_footer("Tri = cycle preset");
    flush_fb();
}

__attribute__((noinline)) void render_screen_gyro() {
    fb_clear();
    draw_header("Gyro Tilt", nullptr);
    if (bt_is_connected()) {
        int16_t ax, ay, az;
        memcpy(&ax, &interrupt_in_data[21], 2);
        memcpy(&ay, &interrupt_in_data[23], 2);
        memcpy(&az, &interrupt_in_data[25], 2);
        ax = imu_apply(3, ax);  // bias/sensitivity-corrected accel (identity if no cal)
        ay = imu_apply(4, ay);
        az = imu_apply(5, az);
        char buf[16];
        snprintf(buf, sizeof(buf), "X%+5d", ax); draw_text(kContentX, row_y(0), buf, kWhite, 2);
        snprintf(buf, sizeof(buf), "Y%+5d", ay); draw_text(88,  row_y(0), buf, kWhite, 2);
        snprintf(buf, sizeof(buf), "Z%+5d", az); draw_text(168, row_y(0), buf, kWhite, 2);

        // 100×100 crosshair box; dot driven by X (roll) / Z (pitch), negated —
        // same axis choice + direction rationale as oled.cpp (centred flat).
        const int bx = 70, by = 78, bw = 100, bh = 100;
        rect_outline(bx, by, bw, bh, kGrey);
        for (int x = bx + 1; x < bx + bw - 1; x++) px(x, by + bh / 2, kDGrey);
        for (int y = by + 1; y < by + bh - 1; y++) px(bx + bw / 2, y, kDGrey);
        int dx = -((int)ax * (bw / 2 - 5)) / 8192;
        int dy = -((int)az * (bh / 2 - 5)) / 8192;
        int cx = bx + bw / 2 + dx;
        int cy = by + bh / 2 + dy;
        if (cx < bx + 3) cx = bx + 3;
        if (cx > bx + bw - 4) cx = bx + bw - 4;
        if (cy < by + 3) cy = by + 3;
        if (cy > by + bh - 4) cy = by + bh - 4;
        rect_filled(cx - 2, cy - 2, 5, 5, theme_accent());

        draw_text(kContentX, 196, g_imu_cal_valid ? "cal: factory" : "cal: raw",
                  kGrey, 2);
    } else {
        draw_text(kContentX, row_y(3), "(no controller)", kGrey, 2);
    }
    flush_fb();
}

__attribute__((noinline)) void render_screen_touchpad() {
    fb_clear();
    draw_header("Touchpad", nullptr);
    if (bt_is_connected()) {
        // 216×121 box ≈ the pad's 1920×1080 aspect.
        const int tx = kContentX, ty = 52, tw = 216, th = 121;
        rect_outline(tx, ty, tw, th, kGrey);
        int active = 0;
        for (int finger = 0; finger < 2; finger++) {
            const int off = 32 + finger * 4;
            const uint32_t f = (uint32_t)interrupt_in_data[off] |
                               ((uint32_t)interrupt_in_data[off + 1] << 8) |
                               ((uint32_t)interrupt_in_data[off + 2] << 16) |
                               ((uint32_t)interrupt_in_data[off + 3] << 24);
            const bool not_touching = (f >> 7) & 1u;
            if (not_touching) continue;
            const uint16_t fx = (f >> 8) & 0xFFFu;
            const uint16_t fy = (f >> 20) & 0xFFFu;
            int sx = (tx + 3) + ((int)fx * (tw - 6)) / 1919;
            int sy = (ty + 3) + ((int)fy * (th - 6)) / 1079;
            if (sx < tx + 3)      sx = tx + 3;
            if (sx > tx + tw - 4) sx = tx + tw - 4;
            if (sy < ty + 3)      sy = ty + 3;
            if (sy > ty + th - 4) sy = ty + th - 4;
            rect_filled(sx - 2, sy - 2, 6, 6, finger == 0 ? theme_accent() : kWhite);
            active++;
        }
        char buf[20];
        snprintf(buf, sizeof(buf), "Fingers: %d", active);
        draw_text(kContentX, 186, buf, kWhite, 2);
    } else {
        draw_text(kContentX, row_y(3), "(no controller)", kGrey, 2);
    }
    flush_fb();
}

__attribute__((noinline)) void render_screen_lightbar() {
    lightbar_handle_input();
    fb_clear();
    draw_header("Lightbar", lb_mode_tag(lb_mode));

    if (bt_is_connected()) {
        // lb_r/g/b are computed every frame by lightbar_service(); display only.
        char buf[16];
        snprintf(buf, sizeof(buf), "R:%3u", lb_r); draw_text(kContentX, row_y(0), buf, kPureR, 2);
        snprintf(buf, sizeof(buf), "G:%3u", lb_g); draw_text(88,  row_y(0), buf, kPureG, 2);
        snprintf(buf, sizeof(buf), "B:%3u", lb_b); draw_text(168, row_y(0), buf, kPureB, 2);

        // Live swatch — the colour panel a mono OLED never could show.
        rect_outline(kContentX, row_y(1), 216, 26, kGrey);
        rect_filled(kContentX + 2, row_y(1) + 2, 212, 22, rgb565(lb_r, lb_g, lb_b));

        // Per-channel bars in their own colours.
        const int by0 = row_y(2) + 8;
        rect_outline(kContentX, by0,      216, 10, kDGrey);
        rect_outline(kContentX, by0 + 14, 216, 10, kDGrey);
        rect_outline(kContentX, by0 + 28, 216, 10, kDGrey);
        const int rf = (lb_r * 212) / 255;
        const int gf = (lb_g * 212) / 255;
        const int bf = (lb_b * 212) / 255;
        if (rf > 0) rect_filled(kContentX + 2, by0 + 2,      rf, 6, kPureR);
        if (gf > 0) rect_filled(kContentX + 2, by0 + 16,     gf, 6, kPureG);
        if (bf > 0) rect_filled(kContentX + 2, by0 + 30,     bf, 6, kPureB);

        // Face button rising-edge -> save current colour to favourite slot.
        const uint8_t face = interrupt_in_data[7] & 0xF0;
        const uint8_t pressed = face & ~lb_last_face;
        lb_last_face = face;
        int save_slot = -1;
        if      (pressed & 0x80) save_slot = 0; // Triangle
        else if (pressed & 0x40) save_slot = 1; // Circle
        else if (pressed & 0x20) save_slot = 2; // Cross
        else if (pressed & 0x10) save_slot = 3; // Square
        if (save_slot >= 0) {
            lb_fav_r[save_slot] = lb_r;
            lb_fav_g[save_slot] = lb_g;
            lb_fav_b[save_slot] = lb_b;
            lb_dirty = true; // persisted on leaving the Lightbar screen
        }

        // Favourite swatches with their real colours.
        draw_text(kContentX, 168, "Fav:", kGrey, 2);
        for (int i = 0; i < 4; i++) {
            const int fx = 62 + i * 42;
            rect_outline(fx, 166, 34, 18, kGrey);
            rect_filled(fx + 2, 168, 30, 14, rgb565(lb_fav_r[i], lb_fav_g[i], lb_fav_b[i]));
        }
        draw_text(kContentX, 192, "Sv:T0 C1 X2 S3", kGrey, 2);

        const char* hint =
            (lb_mode == 0)           ? "Tilt = R/G/B"   :
            (lb_mode == 5)           ? "Breathing FAV0" :
            (lb_mode == 6)           ? "Rainbow sweep"  :
            (lb_mode == 7)           ? "Fade thru FAVs" :
            (lb_mode == kLbModeHost) ? "Host controls"  :
                                       "Locked to fav";
        char foot[28];
        snprintf(foot, sizeof(foot), "R1=mode  %s", hint);
        draw_footer(foot);
    } else {
        draw_text(kContentX, row_y(3), "(no controller)", kGrey, 2);
        draw_footer("R1=mode");
    }
    flush_fb();
}

__attribute__((noinline)) void render_screen_vu() {
    fb_clear();
    draw_header("Audio Meters", nullptr);
    const uint16_t accent = theme_accent();
    if (bt_is_connected()) {
        const uint8_t spk = audio_peak_speaker();
        const uint8_t hap = audio_peak_haptic();
        char buf[16];
        snprintf(buf, sizeof(buf), "SPK %3u", spk);
        draw_text(kContentX, row_y(0), buf, kWhite, 2);
        rect_outline(kContentX, row_y(1), 216, 18, kGrey);
        const int sfill = (spk * 212) / 255;
        if (sfill > 0) rect_filled(kContentX + 2, row_y(1) + 2, sfill, 14, accent);

        snprintf(buf, sizeof(buf), "HAP %3u", hap);
        draw_text(kContentX, row_y(2) + 6, buf, kWhite, 2);
        rect_outline(kContentX, row_y(3) + 6, 216, 18, kGrey);
        const int hfill = (hap * 212) / 255;
        if (hfill > 0) rect_filled(kContentX + 2, row_y(3) + 8, hfill, 14, accent);

        draw_text(kContentX, row_y(5), "USB audio peaks", kGrey, 2);
    } else {
        draw_text(kContentX, row_y(3), "(no controller)", kGrey, 2);
    }
    flush_fb();
}

// ── Settings (logic identical to oled.cpp, incl. 4-Player rows) ─────────────
void settings_adjust(int delta) {
    Config_body &c = settings_local;
    settings_dirty = true;
    switch (settings_sel) {
        case 0: { // haptics_gain  [1.0, 2.0] step 0.1
            int v = (int)(c.haptics_gain * 10.0f + 0.5f) + delta;
            if (v < 10) v = 10; if (v > 20) v = 20;
            c.haptics_gain = v / 10.0f;
            break;
        }
        case 1: { // speaker_volume  [-100, 0] step 5
            int v = (int)c.speaker_volume + delta * 5;
            if (v < -100) v = -100; if (v > 0) v = 0;
            c.speaker_volume = (float)v;
            break;
        }
        case 2: { // inactive_time  [10, 60] step 5
            int v = (int)c.inactive_time + delta * 5;
            if (v < 10) v = 10; if (v > 60) v = 60;
            c.inactive_time = (uint8_t)v;
            break;
        }
        case 3: c.disable_inactive_disconnect ^= 1; break;
        case 4: c.disable_pico_led ^= 1; break;
        case 5: { // polling_rate_mode  0..2
            int v = (int)c.polling_rate_mode + delta;
            if (v < 0) v = 2; if (v > 2) v = 0;
            c.polling_rate_mode = (uint8_t)v;
            break;
        }
        case 6: { // audio_buffer_length  [16, 128] step 4
            int v = (int)c.audio_buffer_length + delta * 4;
            if (v < 16) v = 16; if (v > 128) v = 128;
            c.audio_buffer_length = (uint8_t)v;
            break;
        }
        case 7: { // controller_mode  0..2
            int v = (int)c.controller_mode + delta;
            if (v < 0) v = 2; if (v > 2) v = 0;
            c.controller_mode = (uint8_t)v;
            break;
        }
        case 8: { // auto_haptics_enable  0..3
            int v = (int)c.auto_haptics_enable + delta;
            if (v < 0) v = 3; if (v > 3) v = 0;
            c.auto_haptics_enable = (uint8_t)v;
            break;
        }
        case 9: { // auto_haptics_gain  [0, 200] step 10
            int v = (int)c.auto_haptics_gain + delta * 10;
            if (v < 0) v = 0; if (v > 200) v = 200;
            c.auto_haptics_gain = (uint8_t)v;
            break;
        }
        case 10: { // auto_haptics_lowpass  0..3
            int v = (int)c.auto_haptics_lowpass + delta;
            if (v < 0) v = 3; if (v > 3) v = 0;
            c.auto_haptics_lowpass = (uint8_t)v;
            break;
        }
        case 11: { // screen_dim_timeout  [0,250] min, 0 = disabled
            int v = (int)c.screen_dim_timeout + delta;
            if (v < 0) v = 0; if (v > 250) v = 250;
            c.screen_dim_timeout = (uint8_t)v;
            break;
        }
        case 12: { // screen_off_timeout  [0,250] min, 0 = disabled
            int v = (int)c.screen_off_timeout + delta;
            if (v < 0) v = 0; if (v > 250) v = 250;
            c.screen_off_timeout = (uint8_t)v;
            break;
        }
        case 13: c.bt_mic_enable ^= 1; break;
        case 14: c.controller_wakes_display ^= 1; break;
        case 15: { // player_id 0 (off) .. 8, wraps (8-Player extension)
            int v = (int)c.player_id + delta;
            if (v < 0) v = 8; if (v > 8) v = 0;
            // Crossing into P5..P8 defaults BT mic off (2.4 GHz airtime is the
            // scaling constraint; audio streams are the hog). Overridable —
            // just re-enable the mic row after picking the player number.
            if (v >= 5 && c.player_id < 5) c.bt_mic_enable = 0;
            c.player_id = (uint8_t)v;
            break;
        }
        case 16: c.pair_lock ^= 1; break; // pairing lock on/off (4-Player Edition)
    }
}

// Same flow as oled.cpp, driven by the merged controller+joystick sources so
// Settings can be navigated panel-only (joystick + CTRL) with no controller.
void settings_handle_input() {
    const uint8_t dpad = merged_dpad();
    const uint8_t face = merged_face();

    if (dpad != settings_last_dpad && dpad != 8) {
        if      (dpad == 0) settings_sel = (settings_sel - 1 + kNumSettingsItems) % kNumSettingsItems;
        else if (dpad == 4) settings_sel = (settings_sel + 1) % kNumSettingsItems;
        else if (dpad == 6) settings_adjust(-1);
        else if (dpad == 2) settings_adjust(+1);
    }
    settings_last_dpad = dpad;

    // Triangle (or joystick CTRL): Reset/Wipe rows need a 2 s hold; every
    // other row saves on a short press.
    const bool tri_now = (face & 0x80) != 0;
    const bool tri_prev = (settings_last_face & 0x80) != 0;
    const bool is_hold_item = (settings_sel == kSettingsResetIdx
                               || settings_sel == kSettingsWipeSlotsIdx);
    if (tri_now && !tri_prev) {
        settings_tri_press_us = (uint32_t)time_us_32();
        settings_reset_triggered = false;
    }
    if (is_hold_item && tri_now && !settings_reset_triggered
        && ((uint32_t)time_us_32() - settings_tri_press_us) >= kResetHoldUs) {
        settings_reset_triggered = true;
        if (settings_sel == kSettingsResetIdx) {
            config_default();
            if (config_save()) {
                settings_local = get_config();
                lightbar_load_config(); // refresh RAM lightbar state (no reboot here)
                bright_idx = get_config().screen_brightness;
                settings_dirty = false;
                settings_save_status = "Reset!";
            } else {
                settings_save_status = "Reset FAIL";
            }
            bt_pairing_posture_refresh(); // 4-Player Edition: defaults unlock pairing
        } else {
            bt_wipe_all_slots();
            settings_save_status = "Slots wiped!";
        }
    }
    if (!tri_now && tri_prev) {
        if (!is_hold_item && !settings_reset_triggered) {
            set_config(settings_local);
            settings_save_status = config_save() ? "Saved!" : "Save FAIL";
            if (settings_save_status[0] == 'S' && settings_save_status[1] == 'a') {
                settings_dirty = false;
            }
            bt_pairing_posture_refresh(); // 4-Player Edition: apply pair_lock immediately
        }
        settings_reset_triggered = false;
    }
    settings_last_face = face;
}

__attribute__((noinline)) void format_settings_item(int idx, char* line, size_t n) {
    const Config_body &c = settings_local;
    const char *cur = (idx == settings_sel) ? ">" : " ";
    switch (idx) {
        case 0: {
            int g = (int)(c.haptics_gain * 10.0f + 0.5f);
            snprintf(line, n, "%s Hap Gain %d.%dx", cur, g / 10, g % 10);
            break;
        }
        case 1: snprintf(line, n, "%s Spk Vol %ddB", cur, (int)c.speaker_volume); break;
        case 2: snprintf(line, n, "%s Inact %umin", cur, c.inactive_time); break;
        case 3: snprintf(line, n, "%s InactDC %s", cur, c.disable_inactive_disconnect ? "off" : "on"); break;
        case 4: snprintf(line, n, "%s Pico LED %s", cur, c.disable_pico_led ? "off" : "on"); break;
        case 5: {
            const char* names[3] = {"250Hz", "500Hz", "RT"};
            snprintf(line, n, "%s Poll %s", cur, names[c.polling_rate_mode % 3]);
            break;
        }
        case 6: snprintf(line, n, "%s AudBuf %u", cur, c.audio_buffer_length); break;
        case 7: {
            const char* names[3] = {"DS5", "DSE", "Auto"};
            snprintf(line, n, "%s Ctrl %s", cur, names[c.controller_mode % 3]);
            break;
        }
        case 8: {
            const char* names[4] = {"Off", "Fallback", "Mix", "Replace"};
            snprintf(line, n, "%s AutoHap %s", cur, names[c.auto_haptics_enable & 3]);
            break;
        }
        case 9: snprintf(line, n, "%s AH Gain %u%%", cur, c.auto_haptics_gain); break;
        case 10: {
            const char* names[4] = {"80Hz", "160Hz", "250Hz", "400Hz"};
            snprintf(line, n, "%s AH LP %s", cur, names[c.auto_haptics_lowpass & 3]);
            break;
        }
        case 11:
            if (c.screen_dim_timeout == 0) snprintf(line, n, "%s ScrDim off", cur);
            else snprintf(line, n, "%s ScrDim %umin", cur, c.screen_dim_timeout);
            break;
        case 12:
            if (c.screen_off_timeout == 0) snprintf(line, n, "%s ScrOff off", cur);
            else snprintf(line, n, "%s ScrOff %umin", cur, c.screen_off_timeout);
            break;
        case 13: snprintf(line, n, "%s BT Mic %s", cur, c.bt_mic_enable ? "on" : "off"); break;
        case 14: snprintf(line, n, "%s CtrlWake %s", cur, c.controller_wakes_display ? "on" : "off"); break;
        case 15:
            if (c.player_id == 0) snprintf(line, n, "%s Player off", cur);
            else snprintf(line, n, "%s Player P%u", cur, c.player_id);
            break;
        case 16: snprintf(line, n, "%s PairLock %s", cur, c.pair_lock ? "on" : "off"); break;
        case 17: snprintf(line, n, "%s Reset to defaults", cur); break;
        case 18: snprintf(line, n, "%s Wipe all slots", cur); break;
    }
}

__attribute__((noinline)) void render_screen_settings() {
    if (!settings_init_done) {
        settings_local = get_config();
        settings_init_done = true;
    }
    settings_handle_input();

    fb_clear();
    draw_header("Settings", settings_dirty ? "(*)" : nullptr);
    if (settings_save_status[0]) {
        const int w = text_width(settings_save_status, 2);
        draw_text(kW - 14 - w, 196, settings_save_status, kGreen, 2);
    }

    constexpr int kVisible = 8;
    int top = 0;
    if (settings_sel >= kVisible) top = settings_sel - kVisible + 1;
    char line[28];
    for (int i = 0; i < kVisible && top + i < kNumSettingsItems; i++) {
        const int idx = top + i;
        format_settings_item(idx, line, sizeof(line));
        // Player row renders in the candidate player's colour — instant
        // preview of the accent theme you're about to pick.
        uint16_t colr = (idx == settings_sel) ? kWhite : kGrey;
        if (idx == 15 && settings_local.player_id >= 1 && settings_local.player_id <= 8) {
            colr = kPlayerAccent[settings_local.player_id];
        }
        if (idx == settings_sel) {
            rect_filled(2, row_y(i) - 3, 4, 20, theme_accent());
        }
        draw_text(kContentX, row_y(i), line, colr, 2);
    }

    if (settings_sel == kSettingsResetIdx) {
        draw_footer("Hold Tri/Ctrl 2s=RESET");
    } else if (settings_sel == kSettingsWipeSlotsIdx) {
        draw_footer("Hold Tri/Ctrl 2s=WIPE");
    } else {
        draw_footer("DP/Joy nav  Tri=save");
    }
    flush_fb();
}

// ── Slots (Phase G UI; logic identical, joystick-navigable) ─────────────────
int slots_cursor = -1;             // initialized to active slot on first entry
uint8_t slots_last_dpad = 8;
uint8_t slots_last_face = 0;
uint32_t slots_sq_press_us = 0;
bool slots_wipe_triggered = false;
const char* slots_status = "";
uint32_t slots_status_until_us = 0;
constexpr uint32_t kSlotsWipeHoldUs = 1500000;  // 1.5 s

void slots_handle_input() {
    if (slots_cursor < 0) slots_cursor = bt_get_slot();
    const uint8_t dpad = merged_dpad();
    const uint8_t face = merged_face();

    if (dpad != slots_last_dpad && dpad != 8) {
        if      (dpad == 0) slots_cursor = (slots_cursor - 1 + kNumSlots) % kNumSlots;
        else if (dpad == 4) slots_cursor = (slots_cursor + 1) % kNumSlots;
    }
    slots_last_dpad = dpad;

    // Triangle (or joystick CTRL) rising edge: switch to cursor slot.
    const bool tri_now = (face & 0x80) != 0;
    const bool tri_prev = (slots_last_face & 0x80) != 0;
    if (tri_now && !tri_prev) {
        if (slots_cursor != bt_get_slot()) {
            bt_set_slot(slots_cursor);
            slots_status = "Switched!";
            slots_status_until_us = (uint32_t)time_us_32() + 1500000;
        }
    }

    // Square hold 1.5 s: wipe cursor slot (controller only — the joystick
    // has no second button, and wipe deserves a deliberate gesture).
    const bool sq_now = (face & 0x10) != 0;
    const bool sq_prev = (slots_last_face & 0x10) != 0;
    if (sq_now && !sq_prev) {
        slots_sq_press_us = (uint32_t)time_us_32();
        slots_wipe_triggered = false;
    }
    if (sq_now && !slots_wipe_triggered
        && ((uint32_t)time_us_32() - slots_sq_press_us) >= kSlotsWipeHoldUs) {
        slots_wipe_triggered = true;
        bt_forget_slot(slots_cursor);
        slots_status = "Wiped!";
        slots_status_until_us = (uint32_t)time_us_32() + 1500000;
    }
    if (!sq_now && sq_prev) slots_wipe_triggered = false;

    slots_last_face = face;
}

__attribute__((noinline)) void render_screen_slots() {
    slots_handle_input();
    if (slots_cursor < 0) slots_cursor = bt_get_slot();

    fb_clear();
    const int active = bt_get_slot();
    const bool conn = bt_is_connected();
    char tag[12];
    snprintf(tag, sizeof(tag), "[s%d %s]", active, conn ? "ON" : "--");
    draw_header("Slots", tag);

    const uint16_t accent = theme_accent();
    for (int i = 0; i < kNumSlots; i++) {
        const int y = row_y(i) + i * 8; // 30 px pitch — roomier than text rows
        // Cursor = subtle row background; active slot = accent number badge.
        if (i == slots_cursor) rect_filled(2, y - 4, 224, 26, kDGrey);
        if (i == active) {
            rect_filled(kContentX - 2, y - 2, 18, 22, accent);
            char d[2] = {(char)('0' + i), 0};
            draw_text(kContentX + 1, y, d, kBlack, 2);
        } else {
            char d[2] = {(char)('0' + i), 0};
            draw_text(kContentX + 1, y, d, kGrey, 2);
        }
        char line[24];
        if (slot_occupied(i)) {
            uint8_t a[6];
            slot_get_addr(i, a);
            snprintf(line, sizeof(line), "%02X:%02X:%02X:%02X:%02X:%02X",
                     a[0], a[1], a[2], a[3], a[4], a[5]);
            draw_text(30, y, line, (i == slots_cursor) ? kWhite : kGrey, 2);
        } else {
            draw_text(30, y, "(empty)", kDGrey, 2);
        }
    }

    if (slots_status[0] && (uint32_t)time_us_32() < slots_status_until_us) {
        draw_text(kContentX, 196, slots_status, kGreen, 2);
    }

    draw_footer("Tri=switch SqHold=wipe");
    flush_fb();
}

// Dim-tier renderer — same walking "I'm alive" dot as the OLED build (2 s
// blink, 8 positions, 30 s per position), scaled to 240×240. On the LCD the
// real power saving is the backlight at kDimPct; the dot just proves life.
// noinline: literal-pool reach (see CLAUDE.md).
__attribute__((noinline))
void render_dim_pulse(uint32_t dim_elapsed_us) {
    fb_clear();
    constexpr uint32_t kPulsePeriodUs = 2UL * 1000000UL; // 2 s blink cycle
    constexpr uint32_t kPulseOnUs     = 1UL * 1000000UL; // 1 s on, 1 s off
    constexpr uint32_t kPosStepUs     = 30UL * 1000000UL; // 30 s per position
    constexpr int kPositions[][2] = {
        { 30,  30}, {120,  30}, {210,  30},
        {210, 120},
        {210, 210}, {120, 210}, { 30, 210},
        { 30, 120},
    };
    constexpr int kNumPositions = sizeof(kPositions) / sizeof(kPositions[0]);
    const bool dot_on = (dim_elapsed_us % kPulsePeriodUs) < kPulseOnUs;
    if (dot_on) {
        const int idx = (int)((dim_elapsed_us / kPosStepUs) % (uint32_t)kNumPositions);
        rect_filled(kPositions[idx][0], kPositions[idx][1], 4, 4, kGrey);
    }
    flush_fb_raw(); // no chrome — nothing to navigate from sleep
}

void boot_splash() {
    fb_clear();
    auto cx_for = [](const char* s, int scale) {
        int n = 0; while (s[n]) n++;
        return (kW - (n * 6 * scale - scale)) / 2;
    };
    const char* l1 = "DS5 Bridge";
    const char* l2 = FIRMWARE_VERSION;
    const char* l3 = "Pico2W + LCD 1.3";
    draw_text(cx_for(l1, 3), 70,  l1, theme_accent(), 3);
    draw_text(cx_for(l2, 2), 110, l2, kWhite, 2);
    draw_text(cx_for(l3, 2), 140, l3, kGrey, 2);
    // Boot only: flush synchronously, THEN light the backlight — the panel
    // powers up showing the splash instead of a white flash of random GRAM.
    flush_blocking();
    bl_set_pct(kBrightPct[bright_idx]);
    sleep_ms(1500);
}

} // namespace

// Entry points — same names as the OLED backend (src/oled.h); main.cpp calls
// these unconditionally, whichever display file was compiled in.
void oled_init() {
    spi_init(spi1, 40 * 1000 * 1000);         // ST7789VW max write clk is 62.5 MHz
    gpio_set_function(kPinCLK, GPIO_FUNC_SPI);
    gpio_set_function(kPinMOSI, GPIO_FUNC_SPI);

    gpio_init(kPinCS);   gpio_set_dir(kPinCS, GPIO_OUT);  gpio_put(kPinCS, 1);
    gpio_init(kPinDC);   gpio_set_dir(kPinDC, GPIO_OUT);  gpio_put(kPinDC, 0);
    gpio_init(kPinRST);  gpio_set_dir(kPinRST, GPIO_OUT); gpio_put(kPinRST, 1);

    gpio_init(kPinKey0); gpio_set_dir(kPinKey0, GPIO_IN); gpio_pull_up(kPinKey0);
    gpio_init(kPinKey1); gpio_set_dir(kPinKey1, GPIO_IN); gpio_pull_up(kPinKey1);
    gpio_init(kPinKeyX); gpio_set_dir(kPinKeyX, GPIO_IN); gpio_pull_up(kPinKeyX);
    gpio_init(kPinKeyY); gpio_set_dir(kPinKeyY, GPIO_IN); gpio_pull_up(kPinKeyY);
    gpio_init(kPinJoyUp);    gpio_set_dir(kPinJoyUp, GPIO_IN);    gpio_pull_up(kPinJoyUp);
    gpio_init(kPinJoyDown);  gpio_set_dir(kPinJoyDown, GPIO_IN);  gpio_pull_up(kPinJoyDown);
    gpio_init(kPinJoyLeft);  gpio_set_dir(kPinJoyLeft, GPIO_IN);  gpio_pull_up(kPinJoyLeft);
    gpio_init(kPinJoyRight); gpio_set_dir(kPinJoyRight, GPIO_IN); gpio_pull_up(kPinJoyRight);
    gpio_init(kPinJoyCtrl);  gpio_set_dir(kPinJoyCtrl, GPIO_IN);  gpio_pull_up(kPinJoyCtrl);

    bl_init();                                 // backlight dark until the splash
    dma_ch = dma_claim_unused_channel(true);   // after cyw43_arch_init — no clash

    hw_reset();
    st7789_init();
    fb_clear();

    // Restore persisted lightbar mode/favourites and brightness (config_load()
    // already ran in main()). config_valid clamps screen_brightness to a legal
    // kBrightPct index, so this is safe to use directly.
    lightbar_load_config();
    bright_idx = get_config().screen_brightness;

    boot_splash();
}

void oled_loop() {
    handle_buttons();
    const uint32_t now = time_us_32();
    rumble_burst_tick(now);

    // Retire an in-flight DMA flush (non-blocking status check, every loop).
    flush_service();

    if ((now - last_render_us) < kFrameUs) return;
    // A render pass draws into the framebuffer the DMA reads from — never
    // overlap them. Deferring costs one loop iteration (~µs), not a frame.
    if (flush_in_progress) return;
    last_render_us = now;

    // Track charge progress every frame — before the power-ladder early-returns
    // below, so step timing stays correct even while the panel is dimmed/off.
    sample_charge_eta();
    // Parse the DS5's per-unit IMU calibration once it lands (no-op until then).
    imu_cal_service();
    // Drive the controller LED every frame (any screen / power state).
    lightbar_service();

    // Bump activity on controller input changes — identical rolling hash and
    // stick-deadzone collapse as oled.cpp (see the rationale there).
    uint32_t hash = 0;
    for (int i = 0; i < 10; i++) {
        if (i == 6) continue;
        uint8_t b = interrupt_in_data[i];
        if (i < 4 && b >= 120 && b <= 140) b = 128; // stick deadzone
        hash = hash * 31u + b;
    }
    if (hash != last_input_hash) {
        last_input_hash = hash;
        if (get_config().controller_wakes_display) last_activity_us = time_us_64();
    }
    const bool bt_connected_now = bt_is_connected();
    if (bt_connected_now && !prev_bt_connected) last_activity_us = time_us_64();
    prev_bt_connected = bt_connected_now;

    // Idle power ladder — Active → Dim → Off, thresholds in config minutes
    // (0 = tier disabled), capped at Dim while charging: same policy as the
    // OLED build. On this panel the tiers map to the BACKLIGHT (user PWM →
    // kDimPct glow → fully OFF + DISPOFF), because an IPS backlight is what
    // actually glows in a dark room.
    const uint64_t idle   = time_us_64() - last_activity_us;
    const uint64_t dim_us = (uint64_t)get_config().screen_dim_timeout * 60ULL * 1000000ULL;
    const uint64_t off_us = (uint64_t)get_config().screen_off_timeout * 60ULL * 1000000ULL;
    const bool off_enabled = get_config().screen_off_timeout != 0;
    const bool dim_enabled = get_config().screen_dim_timeout != 0;
    if (off_enabled && idle > off_us && !g_charge_eta.charging) {
        if (disp_power_state != DISP_OFF) {
            bl_set_pct(0);              // the actual "dark room" guarantee
            st7789_display(false);      // and stop the panel scanning
            disp_power_state = DISP_OFF;
        }
        return; // panel dark, nothing to draw
    }
    if (disp_power_state == DISP_OFF) st7789_display(true); // wake before drawing
    if (dim_enabled && idle > dim_us) {
        bl_set_pct(kDimPct);
        disp_power_state = DISP_DIM;
        const uint32_t dim_t0 = time_us_32();
        render_dim_pulse((uint32_t)(idle - dim_us));
        latency_note_display_busy(time_us_32() - dim_t0);
        return; // skip the regular per-screen render path
    }
    bl_set_pct(kBrightPct[bright_idx]);
    disp_power_state = DISP_ACTIVE;

    // True on the first render after navigating to a different screen.
    static int last_rendered_screen = -1;
    const bool screen_entered = (current_screen != last_rendered_screen);

    // Leaving Trigger Test → reset the adaptive trigger preset (same
    // surprise-avoidance as oled.cpp).
    if (last_rendered_screen == kScreenTriggers
        && current_screen != kScreenTriggers) {
        trigger_preset = 0;
        send_trigger_effect(0);
    }

    // Leaving the Lightbar screen → persist mode/favourite changes, batched
    // into a single flash write.
    if (last_rendered_screen == kScreenLightbar
        && current_screen != kScreenLightbar
        && lb_dirty) {
        lightbar_save_config();
    }

    last_rendered_screen = current_screen;

    // Time the render pass (draw into RAM + start the background DMA). This
    // is the LCD build's entire display-path blocking cost — compare against
    // the OLED build on the Latency screen / 0xFD [70..73].
    const uint32_t disp_t0 = time_us_32();
    switch (current_screen) {
        case kScreenStatus:   render_screen();           break;
        case kScreenSlots:    render_screen_slots();     break;
        case kScreenLightbar: render_screen_lightbar();  break;
        case kScreenTriggers: render_screen_triggers();  break;
        case kScreenGyro:     render_screen_gyro();      break;
        case kScreenTouchpad: render_screen_touchpad();  break;
        case kScreenDiag:     render_screen_diag();      break;
        case kScreenLatency:  render_screen_latency();   break;
        case kScreenCpu:      render_screen_cpu(screen_entered); break;
        case kScreenRssi:     render_screen_rssi();      break;
        case kScreenVU:       render_screen_vu();        break;
        case kScreenSettings: render_screen_settings();  break;
    }
    latency_note_display_busy(time_us_32() - disp_t0);
}
