# Pico-LCD-1.3 colour port — hardware test plan

Hardware: 1× Pico 2 W + Pico-OLED-1.3 (known-good dongle), 1× Pico 2 W + Pico-LCD-1.3,
1× bare Pico 2 W, DualSense controller(s), Windows 10 PC.
UF2s: `ds5-bridge-oled.uf2` and `ds5-bridge-lcd13.uf2` from the latest green CI run.

Order matters: 1–3 are regression gates before touching the new panel.

## 1. OLED regression (ds5-bridge-oled.uf2 on the OLED Pico)
- [ ] Boots, pairs, plays — behaviour identical to v0.6.12+4-player apart from the two intended changes below.
- [ ] Screen list now: Status → Slots → Lightbar → Triggers → Gyro → Touchpad → Diag → **Latency** → CPU → RSSI → VU → Settings (12).
- [ ] Latency screen shows BT in ≈ controller report rate, USB out ≈ poll rate, transit avg/max, peak, BT gap; "Disp busy max" settles to a steady value (note it — this is the OLED baseline).
- [ ] Settings unchanged (19 rows incl. Player/PairLock). NOTE: fresh flash / factory reset now defaults Poll to RT — a previously saved 250/500 choice survives.

## 2. Headless (bare Pico 2 W, either UF2 — run both if time allows)
- [ ] Clean boot with no display attached, pairs, plays. No hangs, no watchdog loops (LED triple-blink after replug would indicate one).

## 3. Latency parity gate (the blocker check)
- [ ] Both dongles set to RT/1 kHz (`set_ds5.py --polling rt`, then **replug USB**).
- [ ] Same controller, same distance: compare Latency screens. LCD transit avg/max/peak must be within noise of OLED (≤ ~100 µs difference in avg; peaks similar).
- [ ] "Disp busy max" on LCD must be ≤ the OLED baseline from step 1 (target: same order, ~1 ms).
- [ ] Optional host-side: `python scripts/set_ds5.py --latency` (works on Linux hidraw; Windows may refuse the undeclared 0xFD ID — the on-screen page shows the same numbers).
- If the LCD build is measurably worse at 1 kHz: STOP, report — that's a blocker, not a footnote.

## 4. LCD panel bring-up (ds5-bridge-lcd13.uf2 on the LCD Pico)
- [ ] Splash appears (backlight comes up already showing it), correct orientation with joystick left / A-B right. If upside-down: flip `kRotate180` in src/lcd.cpp — board revisions differ.
- [ ] All 12 screens render; A = next, B = back (identical to OLED KEY0/KEY1).
- [ ] Colours sane: red battery bar when low, pure R/G/B channel bars on Lightbar, no red/blue swap (if swapped, MADCTL RGB bit — report it).

## 5. Player theming
- [ ] `set_ds5.py --player 1..4` (or Settings → Player): UI accent follows P1 blue / P2 red / P3 green / P4 pink; matches the lightbar colour stamped on connect (lightbar_mode HOST). Player off → white chrome.
- [ ] Settings "Player" row previews the candidate colour while adjusting.

## 6. Backlight power ladder (dark room)
- [ ] Set ScrDim 1 min, ScrOff 2 min, CtrlWake off. Active → dims to a faint glow + walking dot after 1 min → backlight fully OFF (zero glow) after 2 min.
- [ ] Wakes on A/B, joystick, controller connect; while charging it dims but never fully sleeps (same policy as OLED).
- [ ] B long-press cycles 100/55/28/10 % and survives a power cycle.

## 7. Joystick (LCD extra)
- [ ] With NO controller: joystick navigates Settings/Slots/Diag rows; CTRL short-press saves a setting; CTRL 2 s hold on Reset row factory-resets.
- [ ] With controller connected: D-pad wins; joystick still works when D-pad released.

## 8. System integrity on LCD build
- [ ] Audio: game/speaker test (`test_speaker.sh` or in-game) — speaker + haptics alive, no stutter while screens redraw.
- [ ] Native triggers still fire (descriptor untouched) — quick Cyberpunk/DS2 check if convenient.
- [ ] 4-player: pair_lock + slots behave as on OLED; PS+Mute 2 s soft-reboot works; A+B 1 s chord reboots.
- [ ] Leave running 30+ min at 1 kHz: Latency peak stays sane (flash saves briefly spike it — expected), no watchdog reboot.

## Known-acceptable differences vs OLED
- Boot splash shows firmware version via FIRMWARE_VERSION (OLED splash still says v0.6.0 — pre-existing).
- Dim tier is a backlight level, not a contrast trick; Off uses DISPOFF + backlight cut, not SH1107 sleep.
- Slots rows use a colour badge/highlight instead of the `>`/`*` text markers (same information).
