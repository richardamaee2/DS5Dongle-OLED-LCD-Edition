//
// Latency telemetry — dongle transit (BT arrival → USB accept), BT
// inter-packet jitter, and display-path blocking time. Display-independent
// plumbing shared by the Latency screen (src/oled.cpp / src/lcd.cpp) and the
// 0xFD vendor feature report (src/cmd.cpp).
//
// Design constraints (why this looks the way it does):
//   - The hot input path may only pay for a couple of timestamps: samples are
//     ACCUMULATED inline (a time_us_32() read plus a few integer ops) and the
//     divide/publish work happens in latency_get(), which only runs at display
//     / feature-report rate (~10 Hz).
//   - Everything runs cooperatively on core 0, so plain (non-atomic) globals
//     are safe; `volatile` is not needed for correctness here and is omitted
//     to keep the accumulation paths to single load/store instructions.
//
// Implementation lives in src/main.cpp next to the other diagnostic counters.
//

#ifndef DS5_BRIDGE_LATENCY_H
#define DS5_BRIDGE_LATENCY_H

#include <cstdint>

// One-second-window statistics, published on the fly by latency_get().
// Rates are per second; times are microseconds. All zero when no traffic
// (e.g. controller disconnected) or before the first full window.
struct LatencyStats {
    uint32_t transit_avg_us;   // mean BT-arrival → tud_hid_report-accept, last window
    uint32_t transit_max_us;   // worst transit, last window
    uint32_t transit_peak_us;  // worst transit since boot (catches rare stalls)
    uint32_t bt_rate;          // BT 0x31 input reports accepted per second
    uint32_t usb_rate;         // HID input reports accepted by TinyUSB per second
    uint32_t bt_gap_min_us;    // min BT inter-arrival interval, last window
    uint32_t bt_gap_max_us;    // max BT inter-arrival interval, last window
};

// Snapshot the current stats. Finalizes the 1 s accumulation window when it
// has elapsed — call at display rate, NOT from the input path.
void latency_get(LatencyStats *out);

// Record one display-path blocking interval (render + flush bookkeeping time
// spent inside oled_loop()/lcd_loop() while the main loop was stalled).
// Both display backends report here; the max since boot is the number the
// "no display pass may block longer than the OLED render" budget is judged by.
void latency_note_display_busy(uint32_t us);
uint32_t latency_display_busy_max_us();

// USB polling mode actually patched into the endpoint descriptors at the last
// enumeration (0/1/2 = 250/500/1000 Hz), 0xFF before first enumeration.
// Written by tud_descriptor_configuration_cb (src/usb_descriptors.cpp); lets
// the Latency screen flag "config says RT but USB is still enumerated at
// 500 Hz — replug" without touching any descriptor bytes.
extern uint8_t g_usb_active_poll_mode;

#endif // DS5_BRIDGE_LATENCY_H
