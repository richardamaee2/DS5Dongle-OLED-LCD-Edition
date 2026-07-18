//
// Created by awalol on 2026/5/15.
//

#include <cstddef>
#include <cstring>

#include "utils.h"
#include "state_mgr.h"
#include "config.h"
#include "bt.h"
#include "pico/time.h"

// Set by the OLED lightbar service (src/oled.cpp). While true, the firmware
// owns the lightbar (an OLED mode or the charging pulse) and the host's
// AllowLedColor writes are suppressed below so they can't stomp it.
extern bool g_lightbar_override;

namespace {
    constexpr size_t kAudioControlOffset = offsetof(SetStateData, MuteLightMode) - sizeof(uint8_t);
    constexpr size_t kMuteControlOffset = offsetof(SetStateData, RightTriggerFFB) - sizeof(uint8_t);
    constexpr size_t kMotorPowerLevelOffset = offsetof(SetStateData, HostTimestamp) + sizeof(uint32_t);
    constexpr size_t kAudioControl2Offset = kMotorPowerLevelOffset + sizeof(uint8_t);
    constexpr size_t kHapticLowPassFilterOffset = offsetof(SetStateData, LightFadeAnimation) - 2 * sizeof(uint8_t);
    constexpr size_t kPlayerIndicatorsOffset = offsetof(SetStateData, LedRed) - sizeof(uint8_t);
}

static constexpr uint8_t state_init_data[63] = {
    0xfd, 0xf7, 0x0, 0x0,
    0x7f, 0x64, // Headphones, Speaker
    0x40, 0x9, 0x0, 0x00, 0x0, 0x0, 0x0, 0x0, // VolumeMic=64, MuteControl all clear (no PowerSave)
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xa,
    0x7, 0x0, 0x0, 0x2, 0x1,
    0x00,
    0xff, 0xd7, 0x00 // RGB LED: R, G, B (Nijika Color!)✨
};

uint8_t state[63]{};

void state_init() {
    memcpy(state, state_init_data, sizeof(state));
}

void state_set(uint8_t *data, const uint8_t size) {
    if (size > 63) {
        printf("[StateMgr] Warning: State Set over 63 bytes\n");
    }
    memcpy(data, state, size);
}

void state_set_led(uint8_t r, uint8_t g, uint8_t b) {
    state[offsetof(SetStateData, LedRed) + 0] = r;
    state[offsetof(SetStateData, LedRed) + 1] = g;
    state[offsetof(SetStateData, LedRed) + 2] = b;
}

void state_get_led(uint8_t *r, uint8_t *g, uint8_t *b) {
    *r = state[offsetof(SetStateData, LedRed) + 0];
    *g = state[offsetof(SetStateData, LedRed) + 1];
    *b = state[offsetof(SetStateData, LedRed) + 2];
}

// ---- 4-Player Edition: per-dongle player identity --------------------------
// When config.player_id is 1..8 the dongle applies PS5-style defaults at
// connect time: the white player-indicator LEDs show the player number, and —
// while lightbar_mode is HOST/passthrough — the lightbar shows the PS5 player
// colour (P1 blue / P2 red / P3 green / P4 pink). Exactly like a console: the
// moment the host/game claims the LEDs via an output report (AllowLedColor /
// AllowPlayerIndicators), the host wins and the stamps stop.
//
// The DS5 ignores LED writes during its BT pair animation, so a connect-time
// stamp alone can get swallowed. player_tick() therefore re-stamps and pushes
// a 0x31 every 500 ms between 0.5 s and 4.5 s after connect (skipped while the
// speaker stream is active — state[] already rides every 0x36 audio frame on
// the load-bearing audio path, which must not be intruded on).

extern int reportSeqCounter; // main.cpp — shared BT 0x31 sequence counter
extern bool spk_active;      // main.cpp — true while the USB speaker stream is open

static bool g_host_led_claimed = false;
static bool g_host_player_claimed = false;
static absolute_time_t g_player_connect_time = 0;
static absolute_time_t g_player_last_push = 0;
static bool g_player_assert_pending = false;

// Player-indicator bitmasks per Linux hid-playstation / SDL (P1..P4). The PS5
// only defines four; P5..P8 patterns are this fork's own, chosen to be
// mutually distinct on the 5-LED bar (8-Player extension).
static constexpr uint8_t kPlayerLedMask[8] = {0x04, 0x0A, 0x15, 0x1B,
                                              0x1F, 0x11, 0x0E, 0x17};
// Player colours: P1..P4 are the PS5 set (blue/red/green/pink); P5..P8 are
// fork-invented (orange/cyan/purple/yellow), matched by the LCD accent table.
static constexpr uint8_t kPlayerColor[8][3] = {
    {0, 0, 255},
    {255, 0, 0},
    {0, 255, 0},
    {255, 0, 128},
    {255, 120, 0},
    {0, 200, 220},
    {150, 60, 255},
    {255, 200, 0},
};
// Keep in sync with kLbModeHost in src/oled.cpp (documented in config.h).
constexpr uint8_t kLightbarModeHost = 8;

static void player_stamp() {
    const uint8_t pid = get_config().player_id;
    if (pid < 1 || pid > 8) return;
    if (!g_host_player_claimed) {
        // PlayerLightFade (bit 5) left 0 = LEDs fade in, like a console slot assign.
        state[kPlayerIndicatorsOffset] = kPlayerLedMask[pid - 1];
    }
    if (!g_host_led_claimed && !g_lightbar_override
        && get_config().lightbar_mode == kLightbarModeHost) {
        state_set_led(kPlayerColor[pid - 1][0], kPlayerColor[pid - 1][1], kPlayerColor[pid - 1][2]);
    }
}

void player_on_connect() {
    g_host_led_claimed = false;
    g_host_player_claimed = false;
    g_player_connect_time = get_absolute_time();
    g_player_last_push = 0;
    const uint8_t pid = get_config().player_id;
    g_player_assert_pending = (pid >= 1 && pid <= 8);
    if (g_player_assert_pending) player_stamp();
}

void state_on_disconnect() {
    g_player_assert_pending = false;
    g_host_led_claimed = false;
    g_host_player_claimed = false;
}

void player_tick() {
    if (!g_player_assert_pending) return;
    if (!bt_is_connected()) {
        g_player_assert_pending = false;
        return;
    }
    const absolute_time_t now = get_absolute_time();
    const int64_t since_connect = absolute_time_diff_us(g_player_connect_time, now);
    // Past 4.5 s the pair animation is over and the stamped state[] rides
    // every subsequent host/audio frame on its own — stop.
    if (since_connect > 4500 * 1000) {
        g_player_assert_pending = false;
        return;
    }
    if (since_connect < 500 * 1000) return; // let the connect-time init packet land first
    if (g_player_last_push != 0
        && absolute_time_diff_us(g_player_last_push, now) < 500 * 1000) {
        return;
    }
    g_player_last_push = now;

    player_stamp();
    if (spk_active) return; // state[] rides the 0x36 audio frames while streaming
    uint8_t outputData[78]{};
    outputData[0] = 0x31;
    outputData[1] = reportSeqCounter << 4;
    if (++reportSeqCounter == 256) {
        reportSeqCounter = 0;
    }
    outputData[2] = 0x10;
    state_set(outputData + 3, sizeof(SetStateData));
    bt_write(outputData, sizeof(outputData));
}

void state_update(const uint8_t *data, const uint8_t size) {
    if (size < sizeof(SetStateData)) {
        printf(
            "[StateMgr] Error: SetStateData at least %u bytes\n",
            static_cast<unsigned>(sizeof(SetStateData))
        );
        return;
    }

    SetStateData update{};
    memcpy(&update, data, sizeof(update));

    const auto copy_if_allowed = [&](const bool allowed, const size_t offset, const size_t length) {
        if (allowed) {
            memcpy(state + offset, data + offset, length);
        }
    };
    auto set_bit = [](uint8_t &byte, const int bit, const bool value) {
        byte = (byte & ~(1 << bit)) | (value << bit);
    };

    set_bit(state[0], 0, update.EnableRumbleEmulation);
    set_bit(state[0], 1, update.UseRumbleNotHaptics);
    set_bit(state[38], 2, update.EnableImprovedRumbleEmulation);
    copy_if_allowed(
        update.UseRumbleNotHaptics || update.EnableRumbleEmulation,
        offsetof(SetStateData, RumbleEmulationRight),
        2
    );

    /*copy_if_allowed(
        update.AllowHeadphoneVolume,
        offsetof(SetStateData, VolumeHeadphones),
        sizeof(update.VolumeHeadphones)
    );*/
    /*copy_if_allowed(
        update.AllowSpeakerVolume,
        offsetof(SetStateData, VolumeSpeaker),
        sizeof(update.VolumeSpeaker)
    );*/
    /*copy_if_allowed(
        update.AllowMicVolume,
        offsetof(SetStateData, VolumeMic),
        sizeof(update.VolumeMic)
    );*/
    /*copy_if_allowed(
        update.AllowAudioControl,
        kAudioControlOffset,
        sizeof(uint8_t)
    );*/

    copy_if_allowed(
        update.AllowMuteLight,
        offsetof(SetStateData, MuteLightMode),
        sizeof(update.MuteLightMode)
    );

    /*copy_if_allowed(
        update.AllowAudioMute,
        kMuteControlOffset,
        sizeof(uint8_t)
    );*/

    copy_if_allowed(
        update.AllowRightTriggerFFB,
        offsetof(SetStateData, RightTriggerFFB),
        sizeof(update.RightTriggerFFB)
    );
    copy_if_allowed(
        update.AllowLeftTriggerFFB,
        offsetof(SetStateData, LeftTriggerFFB),
        sizeof(update.LeftTriggerFFB)
    );

    /*copy_if_allowed(
        update.AllowMotorPowerLevel,
        kMotorPowerLevelOffset,
        sizeof(uint8_t)
    );*/
    /*copy_if_allowed(
        update.AllowAudioControl2,
        kAudioControl2Offset,
        sizeof(uint8_t)
    );*/
    /*copy_if_allowed(
        update.AllowHapticLowPassFilter,
        kHapticLowPassFilterOffset,
        sizeof(uint8_t)
    );*/

    copy_if_allowed(
        update.AllowColorLightFadeAnimation,
        offsetof(SetStateData, LightFadeAnimation),
        sizeof(update.LightFadeAnimation)
    );
    copy_if_allowed(
        update.AllowLightBrightnessChange,
        offsetof(SetStateData, LightBrightness),
        sizeof(update.LightBrightness)
    );
    copy_if_allowed(
        update.AllowPlayerIndicators,
        kPlayerIndicatorsOffset,
        sizeof(uint8_t)
    );
    copy_if_allowed(
        update.AllowLedColor && !g_lightbar_override,
        offsetof(SetStateData, LedRed),
        sizeof(update.LedRed) * 3
    );

    // 4-Player Edition: once the host claims the LEDs, the player-identity
    // defaults yield permanently (until the next connect) — console semantics.
    if (update.AllowPlayerIndicators) {
        g_host_player_claimed = true;
    }
    if (update.AllowLedColor && !g_lightbar_override) {
        g_host_led_claimed = true;
    }
}
