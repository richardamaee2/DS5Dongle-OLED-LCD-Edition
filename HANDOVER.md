# HANDOVER — Rich ⇄ Claude session log & continuation guide

Written 2026-07-19 by the outgoing Claude session, for whichever LLM continues this work.
Read `CLAUDE.md` in this repo FIRST — it is the engineering bible (build system, architecture,
load-bearing gotchas, release ritual). This file covers what CLAUDE.md doesn't: the human,
the session history across all projects, current state, and how to work well here.

---

## 1. Who you're working with

Rich (richardamaee@gmail.com, GitHub `richardamaee2`). PhD scientist, 40 years of software
fiddling, but NEW to game-modding/firmware jargon — explain each new term once, plainly, and
he has it forever (capo-relative fret numbering, transaction translators, and BCFZ containers
all landed first try). Treat him as a sharp reviewer, not a novice: he caught a CamelCase
`DLCKey` rename bug, a 0–8 vs 1–8 off-by-nine phrasing, and called out a physically impossible
suggestion (plugging a display hat onto a headerless Pico). Be explicit about WHERE to type
commands and WHICH folder things go in. He is very lag-sensitive (drives many choices below),
buys hardware readily when justified, likes honest uncertainty + verification checklists, and
prefers concise, direct answers (stated preference).

Machines: gaming PC with RTX 5090 (Selaco/Cyberpunk/RE mods live there); a laptop where the
Claude sessions run (this repo folder is on it: `C:\Data\Git\DS5Dongle-OLED-LCD-Edition`);
a FEVM FA-EX1 mini-PC (Ryzen AI Max+ 395, 2×USB-A + 2×USB-C) that will host the dongle fleet.
UK-based (buys from Pi Hut). Steam ID 76561198211145107 (~869 games, library survey done).

## 2. Session history — all projects, status at handover

### A. DS5Dongle firmware (THIS repo — the active project)
Fork lineage: awalol/DS5Dongle → MarcelineVPQ/DS5Dongle-OLED-Edition → richardamaee2/
DS5Dongle-OLED-LCD-Edition. Everything below is COMMITTED to master (via browser commits)
and COMPILES CLEAN (both variants linked 2026-07-19) but is NOT yet hardware-verified:

1. Clip-safe display strings (6 strings re-wrapped; width budgets: LCD ≤18 chars @ scale 2
   because column [228..240) is A/B chrome; OLED ≤20 chars @ 128 px, kContentX=6).
2. `pico_led_apply()` — "Pico LED off" now applies the instant settings are SAVED
   (both `set_config` overloads call one policy point: LED on ⇔ connected ∧ !disabled).
3. Latency screen in milliseconds (`fmt_ms`, clamped "99.9+"), both variants. The measurement
   itself (BT-arrival→USB-accept transit) already existed in µs; this is presentation.
4. 8-Player extension: `player_id` 0–8 (0=off). P5–P8 fork colours orange/cyan/purple/yellow
   (state_mgr kPlayerColor ↔ lcd kPlayerAccent, keep in sync), invented 5-LED masks
   (Sony defines only P1–P4), and selecting P5–P8 defaults `bt_mic_enable=0` (overridable).
5. Per-slot controller NAMES (12 chars): slots flash layout v2 (auto-migration from v1,
   name cleared when a different pad takes a slot), shown on both Slots screens, dumped via
   0xFA (76 bytes, reqlen-gated; legacy 28-byte readers unaffected), written via 0xF6
   func 0x11 ('S','L',ver=1; ops: 1=addr+name, 2=name, 3=clear).
6. `set_ds5.py`: `--player off,1..8`, `--bt-mic on|off`, `--slots`, `--slot-set N
   --slot-addr AA:BB:.. --slot-name "..."`, `--slot-clear N`. Windows-compatible (hidapi).

PENDING VERIFICATION (his very next step — walk him through it):
LED-off applies on save; `Lat avg x.xx ms` renders; P5–P8 identity end-to-end (badge, accent,
lightbar stamp, LED mask, mic auto-off on crossing into P5+); slot names (set via CLI → shows
on screen → survives reboot → v1 bonds survived migration); pairing-screen text unclipped.
He tests on: 1× headerless Pico (build: oled uf2), 1× OLED, 1× LCD.

Fleet plan: 8 dongles = 6 LCD + 1 OLED (P7) + 1 headerless (P8, configured via CLI only —
no header pins, hats can't attach). Six more Pico+LCD arriving AFTER 2026-07-20. Pi Hut order
reviewed & approved: 5× Pico 2 W w/headers, 5× Waveshare Pico-LCD-1.3, cables 4×1m/2×50cm/
2×30cm, 5 heatsinks (+3 he has = 8), and the 10-port USB 2.0 5V/4A hub (SKU MMP-0272) —
chosen over bus-powered minis (8 radios ≈ 1.2–1.6 A ≫ one port's 500 mA). Doctrine: USB 2.0
hub only (USB 3 radiates 2.4 GHz hash), 20–30 cm spacing between dongles, away from PC rear,
spread across the hub's internal port banks (each internal hub chip = its own TT; verify
mapping with "USB Device Tree Viewer" on Windows). Heatsink goes on the BLACK RP2350 chip,
never the silver CYW43 radio can (antenna!). Per-dongle setup line:
`python scripts\set_ds5.py --player N --polling rt` → pair → `--slot-set 0 --slot-name "..."`
→ `--pair-lock on` when the family is settled. He owns ~20 colour-distinct DualSenses and is
building an addr↔name database from `--slots` dumps. DualSense = BT 5.1; one controller per
dongle (slots are memory, not concurrency).

### B. Rocksmith 2014 (guitar) — second most active
- `outputs/hallelujah/jeffhallcv_p.psarc` = "Hallelujah (Capo View)": his OFFICIAL Jeff
  Buckley DLC modded so the capo-5 lead chart displays capo-RELATIVE frets (his mental model:
  8th absolute = 3rd relative). Technique: every fret −5, `capo`→0, tuning +5/string (both
  SNG metadata and manifest), all IDs/keys renamed jeffhall→jeffhallcv (audio/art shared).
  NOT yet play-tested. Needs CDLC Enabler installed; tuner will show +5 → clip capo, pluck, pass.
- Medley build kit `outputs/hallelujah/medley-cdlc/`: Rocksmith XML chart (1,347 events,
  45 sections) for "Hallelujah / I Know It's Over" (Mystery White Boy, his FLAC), assembled
  from (a) his purchased score, (b) his commissioned capo-relative transcription (PDFs, read
  visually), (c) onset analysis of the audio (anchors verified: first note 9.4 s, build 273 s,
  silence at 414 s). 0:25–4:33 reuses time-warped official-chart patterns (3.2% stretch).
  He must run the Rocksmith Custom Song Toolkit (2-min GUI step, README-BUILD.md walks it)
  to produce the psarc (audio→wem needs Wwise, PC-only). Expect drift; the plan is: he plays
  once, reports early/late sections, a session regenerates with piecewise anchors.
  ⚠ The psarc/SNG tooling was written IN-SESSION in the sandbox (/tmp — EPHEMERAL, gone now).
  To rebuild it: pure-python PSARC codec (AES keys are public constants in 0x0L/rocksmith on
  GitHub; fetch raw files) + SNG struct from its sng.py schema; round-trip verified byte-identical.
- Latency doctrine for him: RS_ASIO mod + `Rocksmith.ini` (ExclusiveMode=1,
  Win32UltraLowLatencyMode=1, LatencyBuffer=1); Real Tone Cable + ASIO4ALL now; he intends to
  buy a Focusrite Scarlett Solo 4th Gen → then RS_ASIO w/ Focusrite driver, 48 kHz, 48–96
  buffer. Write his exact configs when he says the interface arrived. Powered-hub trick is
  about RTC power, not latency.

### C. Delivered & dormant (revive on request)
- **Selaco-RT** (`outputs/Selaco-RT-mod.zip`): ReShade screen-space RT for Selaco. Auto-installer
  finds Steam, downloads ReShade addon-build (extract-DLL-from-setup trick) + free shader packs,
  bundles his iMMERSE Pro/Ultimate 2506 Patreon files (HE LICENSED THEM — never redistribute
  beyond his own deliverables), presets: rt/pt free + marty variants (RTGI etc., technique names
  verified from his actual files). Launch bats force OpenGL so vanilla Steam/Vulkan stays clean;
  `-config selaco-rt-engine.ini` mirror keeps his config untouched. Untested on the 5090 yet;
  depth flags may need flipping (README troubleshooting covers DisplayDepth).
- **RE-Enhance** (`outputs/RE-Enhance.zip`): RE9 Requiem + RE4R. Dropzone installer (his Nexus
  downloads ride inside the zip): REFramework (GitHub auto-fetch w/ per-game asset matching
  REFramework-12=RE4 / -13=RE9), Graphics Enhancement lua scripts (engine RT unlocks), ReShade
  as dxgi.dll + grade-only presets (ConvolutionBloom+ReGrade; NO screen-space GI — engine RT is
  better), structure-preserving `reframework`/`natives` merges, game-affinity routing.
  In game: INSERT = REFramework menu ("Default" once), HOME = ReShade.
- **Cyberpunk guide** (`outputs/Cyberpunk2077-Photorealism-Guide.md`): settings ladder (PT+RR+
  DLSS4+MFG), DreamPunk 3.4.3 / DreamVision (NGD Patreon), HD Reworked, Ultra Plus OR NVIDIA
  PT opt (never both).
- **Steam library survey** (`outputs/Steam-Library-Mod-Tiers.xlsx`): 869 games tiered S–D+Util.
- Marty's Mods: he bought the $9 "Path Tracer" tier (iMMERSE Pro+Ultimate). Files uploaded once;
  copies live inside Selaco-RT and RE-Enhance packages.

## 3. How to work with this codebase (the recipe that worked)

- Read `CLAUDE.md` before touching anything; obey its gotchas (TinyUSB 0.20.0 pin; the
  320 MHz/1.20 V overclock is load-bearing; renders are noinline for literal-pool reasons;
  never hardcode screen indices; mirror EVERY UI change oled.cpp ↔ lcd.cpp).
- Cadence: one feature per UF2 + user checkpoint (he has hardware in hand). He explicitly
  waived it ONCE under deadline pressure (2026-07-19 batch); default back to it.
- New config field: config.h struct + config_valid clamp + format_settings_item +
  settings_adjust (BOTH display files) + CHANGELOG. New vendor op: follow the 0xF6 func-id +
  magic-gate pattern (0x10 'R''M' remap, 0x11 'S''L' slots). New GET data: extend 0xFA/0xFD
  reqlen-gated so legacy readers survive. Keep CHANGELOG `[Unreleased]` current — it is the
  running record he reads.
- Verification without hardware: quote/comment-aware brace/paren tokenizer over edited files
  (regex-only checks false-positive), grep for over-wide strings vs the width budgets,
  round-trip parse anything binary you rewrite.
- End every firmware change with an explicit "rebuild and flash now" + a per-feature verify
  checklist. He runs: `cmake --build build-oled --target ds5-bridge ; cmake --build build-lcd
  --target ds5-bridge` (both dirs configured 2026-07-19; a bare `-DPICO_SDK_PATH=$env:...`
  needs quoting in PowerShell, but his VSCode env supplies the SDK anyway). BOOTSEL + drag UF2.

## 4. Environment & tooling notes (hard-won)

- The session sandbox `/tmp` DOES NOT PERSIST. Anything precious goes in `outputs/` or the repo.
- Write/Edit tools reach the CONNECTED repo folder but NOT `outputs/` (use bash heredocs to
  `/sessions/<name>/mnt/outputs/`). Sandbox pip index is restricted (no `construct`, no
  `pycryptodome`; `cryptography` IS preinstalled). No usable git in sandbox; no custom HTTP
  headers on web_fetch (this killed the Nexus API — their key is header-only; his key from
  that misadventure was revoked).
- GitHub pushes: Claude-in-Chrome extension drives github.com upload pages (he stays logged
  in). Quirks: after staging files the commit form shifts down and the FIRST click+type often
  lands in the search bar or void — screenshot first, then click/type/commit; keep commit
  titles <50 chars (the ProTip banner shifts the button ~13 px); avoid ':' in typed text (a
  hotkey eats it). ALWAYS give him the post-commit local sync:
  `git checkout -- <files> && git pull`. (A cleaner future: have him commit locally in VSCode.)
  Note: web uploads normalize line endings → whole-file diffs; consider .gitattributes someday.
- His subscription window was near limits at handover ("shut off after the 20th unless I pay
  more") — be economical with tool calls; batch browser actions.

## 5. Open threads, in priority order

1. Flash + run the full verification checklist (§2A) on his 3 boards; fix whatever it finds.
2. Rocksmith: Capo View first flight (CDLC Enabler + tuner note), medley Toolkit build, then
   sync-calibration round; RS_ASIO/ini configs when the Scarlett arrives.
3. Fleet assembly when parts arrive: hub TT mapping, per-dongle setup lines, pair_lock,
   controller database from `--slots`, spacing per the RF doctrine.
4. Possible upstream PRs to MarcelineVPQ: clip-safe strings, pico_led_apply, ms latency
   (offer drafted commit messages; he pushes).
5. Selaco-RT + RE-Enhance first runs on the 5090 (depth-flag tweaks likely on Selaco).
6. Nice-to-haves parked: .gitattributes; status-screen slot-name display; player LED patterns
   P5–P8 are invented — revisit if community standardizes; Selaco upstream watch (PR #93 /
   0x36 audio layout note in CLAUDE.md).

One more thing the next session should know: he says thank you by handing you harder problems.
It's a compliment. Enjoy it.
