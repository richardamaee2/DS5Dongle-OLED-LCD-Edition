//
// Created by awalol on 2026/5/4.
//

#include "cmd.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "audio.h"
#include "bt.h"
#include "config.h"
#include "device/usbd.h"
#include "latency.h"
#include "pico/time.h"
#include "slots.h"
#include "remap.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/vreg.h"

uint16_t cpu_temp_raw_smoothed() {
    // One-time ADC bring-up. This is the only place the ADC is initialised
    // now (oled.cpp's CPU screen calls through here too). Runs on core0
    // under the cooperative main loop; adc_select_input(4) is set before
    // every read, so the shared ADC needs no locking.
    static bool adc_ready = false;
    if (!adc_ready) {
        adc_init();
        adc_set_temp_sensor_enabled(true);
        adc_ready = true;
    }
    adc_select_input(4);

    // The temp sensor has a shallow slope (-1.721 mV/C) and ~1 LSB ≈ 0.47 C,
    // so a lone 12-bit sample swings several tenths of a degree frame to
    // frame. Average a big block to kill that...
    constexpr int kSamples = 256;
    uint32_t acc = 0;
    for (int i = 0; i < kSamples; i++) acc += adc_read();
    const float mean = (float)acc / (float)kSamples;

    // ...then a slow EMA so the displayed value glides to the true die
    // temperature rather than mirroring the latest block. Seeded on the
    // first call so it doesn't ramp up from zero.
    static float ema = -1.0f;
    if (ema < 0.0f) ema = mean;
    else            ema += (mean - ema) * 0.15f;
    return (uint16_t)(ema + 0.5f);
}

// Mic-debug globals (defined in main.cpp). File-scope extern so the
// linker resolves them once and cmd.cpp's 0xFD handler reads the same
// memory main.cpp writes to.
extern volatile uint32_t g_bt_31_packets;
extern volatile uint32_t g_bt_other_packets;
extern volatile uint8_t  g_last_other_id;
extern volatile uint8_t  g_other_id_or;
extern volatile uint8_t  g_31_b2_or;
extern volatile uint8_t  g_last_31_b2;
extern volatile uint16_t g_31_len_min;
extern volatile uint16_t g_31_len_max;
extern volatile uint8_t  g_last_other_prefix[8];
extern volatile uint8_t  g_last_any_prefix[16];
extern volatile uint16_t g_longest_len;
extern volatile uint8_t  g_longest_frame[80];
extern volatile uint32_t g_host_out02_total;
extern volatile uint32_t g_host_out02_trig_allow;
extern volatile uint32_t g_host_out02_to_bt;

bool is_pico_cmd(uint8_t report_id) {
    if (report_id == 0xf6 ||
        report_id == 0xf7 ||
        report_id == 0xf8 ||
        report_id == 0xf9 ||
        report_id == 0xfa ||
        report_id == 0xfb ||
        report_id == 0xfc ||
        report_id == 0xfd ||  // mic-debug counters
        report_id == 0xfe     // mic-debug longest-frame dump
    ) {
        return true;
    }
    return false;
}

uint16_t pico_cmd_get(uint8_t report_id, uint8_t *buffer, uint16_t reqlen) {
    if (report_id == 0xf7) {
        printf("[HID] Receive 0xf7 getting config\n");
        const size_t cfg_len = sizeof(Config_body);
        if (cfg_len > reqlen) {
            printf("[Config] Warning: Config_body overflow\n");
        }
        const auto len = std::min(cfg_len, static_cast<size_t>(reqlen));
        memcpy(buffer, &get_config(), len);

        // OLED Edition: append the button-remap block right after Config_body
        // when the host asked for enough room. Old clients request exactly
        // sizeof(Config_body) and never see it; new web tools read config +
        // remap in one GET (the 0xF6/0xF7 reports are 63 bytes, plenty).
        //   [+0]      'R'
        //   [+1]      'M'
        //   [+2]      protocol version (kRemapProtoVer)
        //   [+3..+4]  revision uint16 LE (bumps on each successful set)
        //   [+5..+20] 16-byte remap table (source idx -> target idx, 0xFF=off)
        constexpr size_t kRemapBlock = 5 + kRemapCount;
        if (reqlen >= cfg_len + kRemapBlock) {
            uint8_t *p = buffer + cfg_len;
            p[0] = 'R';
            p[1] = 'M';
            p[2] = kRemapProtoVer;
            const uint16_t rev = remap_revision();
            p[3] = (uint8_t)(rev & 0xFF);
            p[4] = (uint8_t)((rev >> 8) & 0xFF);
            remap_get(p + 5);
            return cfg_len + kRemapBlock;
        }
        return len;
    }
    if (report_id == 0xf8) {
        printf("[HID] Receive 0xf8 getting firmware version\n");
        const auto len = std::min(strlen(PICO_PROGRAM_VERSION_STRING), static_cast<size_t>(reqlen));
        memcpy(buffer, PICO_PROGRAM_VERSION_STRING, len);
        return len;
    }
    if (report_id == 0xf9) {
        // [-128,0]
        int8_t rssi = 0;
        bt_get_signal_strength(&rssi);
        if (reqlen == 0) {
            return 0;
        }
        buffer[0] = rssi;
#if ENABLE_VERBOSE
        printf("[HID] 0xf9 RSSI=%d raw=0x%02X\n", rssi, buffer[0]);
#endif
        return 1;
    }
    if (report_id == 0xfa) {
        // Slots dump. Legacy layout (28 bytes): 4 x bd_addr + 4 x occupied.
        // v2 appends the user-given slot names: 4 x 12 chars = 76 bytes total.
        // reqlen-gated like the 0xFD sections so old readers (web emulator)
        // keep getting exactly the 28 bytes they always asked for.
        constexpr uint16_t kLegacy = 28;
        constexpr uint16_t kFull   = kLegacy + kNumSlots * kSlotNameLen; // 76
        if (reqlen < kLegacy) {
            printf("[HID] 0xfa reqlen=%u too small for slots payload (%u)\n", reqlen, kLegacy);
            return 0;
        }
        for (int i = 0; i < 4; i++) {
            uint8_t addr[6];
            bt_slot_get_addr(i, addr);
            memcpy(buffer + i * 6, addr, 6);
        }
        for (int i = 0; i < 4; i++) {
            buffer[24 + i] = bt_slot_occupied(i) ? 1 : 0;
        }
        if (reqlen < kFull) return kLegacy;
        for (int i = 0; i < kNumSlots; i++) {
            char name[kSlotNameLen + 1];
            slot_get_name(i, name);
            memcpy(buffer + kLegacy + i * kSlotNameLen, name, kSlotNameLen);
        }
        return kFull;
    }
    if (report_id == 0xfb) {
        // OLED Edition: diagnostics + audio meters for the web emulator.
        constexpr uint16_t want = 18;
        if (reqlen < want) {
            printf("[HID] 0xfb reqlen=%u too small for diag payload (%u)\n", reqlen, want);
            return 0;
        }
        const uint32_t uptime_s   = time_us_32() / 1000000u;
        const uint32_t usb_frames = audio_usb_frames();
        const uint32_t bt_packets = audio_bt_packets();
        const uint32_t hci_errs   = bt_hci_err_count();
        memcpy(buffer + 0,  &uptime_s,   4);
        memcpy(buffer + 4,  &usb_frames, 4);
        memcpy(buffer + 8,  &bt_packets, 4);
        buffer[12] = audio_peak_speaker();
        buffer[13] = audio_peak_haptic();
        memcpy(buffer + 14, &hci_errs,   4);
        return want;
    }
    if (report_id == 0xfc) {
        // OLED Edition: CPU / Clock telemetry for the web emulator. 11 bytes:
        //   [0..3]  set_khz  uint32  configured clk_sys (SYS_CLOCK_KHZ)
        //   [4..7]  real_khz uint32  measured clk_sys (cached, see below)
        //   [8]     vcode    uint8   vreg_get_voltage() raw enum code
        //   [9..10] temp_raw uint16  ADC ch4 12-bit reading
        // The web side does the volts/temperature math (same formulas as
        // render_screen_cpu) so the firmware HID path stays float-free.
        constexpr uint16_t want = 11;
        if (reqlen < want) {
            printf("[HID] 0xfc reqlen=%u too small for cpu payload (%u)\n", reqlen, want);
            return 0;
        }
        const uint32_t set_khz = (uint32_t)SYS_CLOCK_KHZ;

        // clk_sys is fixed at boot and frequency_count_khz() busy-waits a few
        // ms — measure exactly once (lazily) and cache. Doing it here on the
        // first poll keeps it off the boot path; one ~ms stall in a single
        // GET_REPORT is acceptable.
        static uint32_t cached_real_khz = 0;
        if (cached_real_khz == 0) {
            cached_real_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
        }

        const uint16_t temp_raw = cpu_temp_raw_smoothed();

        const uint8_t vcode = (uint8_t)vreg_get_voltage();

        memcpy(buffer + 0, &set_khz,         4);
        memcpy(buffer + 4, &cached_real_khz, 4);
        buffer[8] = vcode;
        memcpy(buffer + 9, &temp_raw,        2);
        return want;
    }
    if (report_id == 0xfd) {
        // Bridge-diagnostics feature report. 44-byte payload.
        // Section 1: mic-investigation counters (original 0..31).
        //   [0..3]   uint32  BT 0x31 input report count
        //   [4..7]   uint32  BT non-0x31 input report count
        //   [8]      uint8   last non-0x31 report ID seen
        //   [9]      uint8   OR mask of all non-0x31 report IDs seen
        //   [10]     uint8   OR mask of byte[2] across all 0x31 frames
        //   [11]     uint8   last value of byte[2] in a 0x31 frame
        //   [12..13] uint16  min frame length seen
        //   [14..15] uint16  max frame length seen
        //   [16..23] uint8[8]  first 8 bytes of last non-0x31 frame
        //   [24..31] uint8[8]  first 8 bytes of most recent ANY frame
        // Section 2: trigger-flow counters (issue #3 triage).
        //   [32..35] uint32  host 0x02 OUT reports received total
        //   [36..39] uint32  ...of those, with Allow*TriggerFFB set
        //   [40..43] uint32  ...forwarded as BT 0x31 sub-0x10
        // Section 3: latency telemetry (host-side twin of the Latency screen;
        // fields mirror src/latency.h — reqlen-guarded, so old 44-byte readers
        // like mic_diag.sh bt-trace keep working unchanged).
        //   [44]     uint8   configured polling_rate_mode (0=250Hz 1=500Hz 2=RT)
        //   [45]     uint8   poll mode patched at last USB enumeration (0xFF = pre-enum)
        //   [46..47] uint16  BT input reports/s
        //   [48..49] uint16  USB input reports/s accepted by TinyUSB
        //   [50..53] uint32  dongle transit avg us (1 s window)
        //   [54..57] uint32  dongle transit max us (1 s window)
        //   [58..61] uint32  dongle transit peak us since boot
        //   [62..65] uint32  BT inter-report gap min us (window)
        //   [66..69] uint32  BT inter-report gap max us (window)
        //   [70..73] uint32  worst display-path blocking us since boot
        //   [74]     uint8   display variant: 0 = OLED (SH1107), 1 = LCD (ST7789)
        //   [75]     uint8   reserved (0)
        constexpr uint16_t want = 76;
        for (uint16_t i = 0; i < want && i < reqlen; i++) buffer[i] = 0;

        const uint32_t bt31    = g_bt_31_packets;
        const uint32_t btother = g_bt_other_packets;
        const uint16_t lmin    = g_31_len_min == 0xFFFF ? 0 : g_31_len_min;
        const uint16_t lmax    = g_31_len_max;

        memcpy(buffer + 0,  &bt31, 4);
        memcpy(buffer + 4,  &btother, 4);
        buffer[8]  = g_last_other_id;
        buffer[9]  = g_other_id_or;
        buffer[10] = g_31_b2_or;
        buffer[11] = g_last_31_b2;
        memcpy(buffer + 12, &lmin, 2);
        memcpy(buffer + 14, &lmax, 2);
        for (int i = 0; i < 8 && (16 + i) < reqlen; i++) buffer[16 + i] = g_last_other_prefix[i];
        for (int i = 0; i < 8 && (24 + i) < reqlen; i++) buffer[24 + i] = g_last_any_prefix[i];

        const uint32_t out02   = g_host_out02_total;
        const uint32_t out02_t = g_host_out02_trig_allow;
        const uint32_t out02_b = g_host_out02_to_bt;
        if ((32 + 4) <= reqlen) memcpy(buffer + 32, &out02,   4);
        if ((36 + 4) <= reqlen) memcpy(buffer + 36, &out02_t, 4);
        if ((40 + 4) <= reqlen) memcpy(buffer + 40, &out02_b, 4);

        // Section 3 — assembled locally, then copied under the same
        // reqlen guard as everything above.
        LatencyStats ls{};
        latency_get(&ls);
        uint8_t lat[32] = {0};
        lat[0] = get_config().polling_rate_mode;
        lat[1] = g_usb_active_poll_mode;
        const uint16_t btr = (uint16_t)(ls.bt_rate  > 0xFFFF ? 0xFFFF : ls.bt_rate);
        const uint16_t usr = (uint16_t)(ls.usb_rate > 0xFFFF ? 0xFFFF : ls.usb_rate);
        memcpy(lat + 2,  &btr, 2);
        memcpy(lat + 4,  &usr, 2);
        memcpy(lat + 6,  &ls.transit_avg_us,  4);
        memcpy(lat + 10, &ls.transit_max_us,  4);
        memcpy(lat + 14, &ls.transit_peak_us, 4);
        memcpy(lat + 18, &ls.bt_gap_min_us,   4);
        memcpy(lat + 22, &ls.bt_gap_max_us,   4);
        const uint32_t disp_busy = latency_display_busy_max_us();
        memcpy(lat + 26, &disp_busy, 4);
#ifdef DISPLAY_LCD13
        lat[30] = 1;
#else
        lat[30] = 0;
#endif
        for (int i = 0; i < 32 && (44 + i) < reqlen; i++) buffer[44 + i] = lat[i];
        return (reqlen < want) ? reqlen : want;
    }
    if (report_id == 0xfe) {
        // 0xFE: full content of the LONGEST 0x31 frame seen. Bytes 0-1
        // = length (uint16 LE), bytes 2+ = the captured frame bytes.
        constexpr uint16_t want = 82;  // 2 length + 80 frame bytes
        const uint16_t lim = (reqlen < want) ? reqlen : want;
        for (uint16_t i = 0; i < lim; i++) buffer[i] = 0;
        const uint16_t llen = g_longest_len;
        if (lim >= 2) {
            buffer[0] = (uint8_t)(llen & 0xFF);
            buffer[1] = (uint8_t)((llen >> 8) & 0xFF);
        }
        for (uint16_t i = 0; i < 80 && (i + 2) < lim; i++) {
            buffer[2 + i] = g_longest_frame[i];
        }
        return lim;
    }
    return 0;
}

void pico_cmd_set(uint8_t report_id, uint8_t const *buffer, uint16_t bufsize) {
    (void) report_id;
    if (bufsize == 0) {
        return;
    }

    // 0x01 update config in variable
    // 0x02 write config to flash
    // 0x03 reconnect tinyusb device;
    if (buffer[0] == 0x01) {
        printf("[CMD] Enter config set func\n");
        set_config(buffer + 1, bufsize - 1);
        // 4-Player Edition: pair_lock may have changed — refresh discoverability.
        bt_pairing_posture_refresh();
    }
    if (buffer[0] == 0x02) {
        printf("[CMD] Enter config save func\n");
        config_save();
    }
    if (buffer[0] == 0x03) {
        printf("[CMD] Enter tud reconnect func\n");
        tud_disconnect();
        sleep_ms(150);
        tud_connect();
    }
    // 0x10 set button-remap table (OLED Edition). Hardened framing so a stray
    // write to 0xF6 can't corrupt the map: magic 'R''M' + protocol version gate
    // before remap_set() (which itself validates each entry <16 or 0xFF=off).
    //   [0]      0x10  func-id
    //   [1]      'R'
    //   [2]      'M'
    //   [3]      protocol version (must == kRemapProtoVer)
    //   [4..19]  16-byte remap table
    if (buffer[0] == 0x10) {
        constexpr uint16_t kNeed = 4 + kRemapCount;
        if (bufsize < kNeed) {
            printf("[CMD] 0x10 remap-set too short (%u<%u)\n", bufsize, kNeed);
            return;
        }
        if (buffer[1] != 'R' || buffer[2] != 'M' || buffer[3] != kRemapProtoVer) {
            printf("[CMD] 0x10 remap-set bad magic/version\n");
            return;
        }
        if (remap_set(buffer + 4)) printf("[CMD] remap set ok (rev=%u)\n", remap_revision());
        else                       printf("[CMD] remap set rejected (invalid table)\n");
    }
    // 0x11 slot write (8-Player fleet bookkeeping). Same hardened framing idea
    // as 0x10: magic 'S''L' + protocol version gate so a stray 0xF6 write can't
    // touch the slot table. Persisted immediately by slots.cpp.
    //   [0]      0x11  func-id
    //   [1]      'S'
    //   [2]      'L'
    //   [3]      protocol version (1)
    //   [4]      op: 1 = set addr+name (occupied=1), 2 = set name only, 3 = clear slot
    //   [5]      slot index 0..3
    //   [6..11]  bd_addr (op 1; byte order as shown on the Slots screen / 0xFA dump)
    //   [12..23] name, 12 chars, NUL-padded (ops 1 and 2)
    if (buffer[0] == 0x11) {
        constexpr uint16_t kNeed = 6 + 6 + kSlotNameLen;
        if (bufsize < kNeed) {
            printf("[CMD] 0x11 slot-write too short (%u<%u)\n", bufsize, kNeed);
            return;
        }
        if (buffer[1] != 'S' || buffer[2] != 'L' || buffer[3] != 1) {
            printf("[CMD] 0x11 slot-write bad magic/version\n");
            return;
        }
        const uint8_t op = buffer[4];
        const int slot = buffer[5];
        if (slot < 0 || slot >= kNumSlots) {
            printf("[CMD] 0x11 slot index %d out of range\n", slot);
            return;
        }
        char name[kSlotNameLen + 1];
        memcpy(name, buffer + 12, kSlotNameLen);
        name[kSlotNameLen] = 0;
        if (op == 1) {
            slot_set_entry(slot, buffer + 6, name);
            printf("[CMD] slot %d set: addr+name '%s'\n", slot, name);
        } else if (op == 2) {
            slot_set_name(slot, name);
            printf("[CMD] slot %d name set: '%s'\n", slot, name);
        } else if (op == 3) {
            slot_forget(slot);
            printf("[CMD] slot %d cleared\n", slot);
        } else {
            printf("[CMD] 0x11 unknown op %u\n", op);
        }
    }
}
