#!/usr/bin/env python3
"""
DS5Dongle (OLED Edition / 4-Player Edition) configuration tool.

Reads and writes the firmware's persistent config over USB HID feature
reports — no browser, no WebHID, works in any terminal. Ported from
loteran/DS5Dongle's scripts/set_ds5.py and extended for this fork's
Config_body layout.

4-Player Edition additions:
  --player {off,1..8}      per-dongle player identity (PS5-style player LEDs
                           + player lightbar colour until the game overrides;
                           P5-P8 are fork colours and default the BT mic off)
  --bt-mic {on,off}        DualSense mic over Bluetooth
  --pair-lock {on,off}     freeze the bonded-controller set (a locked dongle
                           never pairs new controllers — multi-dongle etiquette)

The script auto-detects the firmware's Config_body length (the firmware
appends an 'RM'-tagged remap block right after the body in the 0xF7
response), so it stays compatible with older firmwares — it just refuses
to set fields the connected firmware doesn't have.

Requires EITHER `cython-hidapi` (preferred — `pip install hidapi`) OR
the `hid` package (`pip install hid`).

Quick usage:
  scripts/set_ds5.py                       # print current config
  scripts/set_ds5.py --player 2            # this dongle is Player 2
  scripts/set_ds5.py --pair-lock on        # lock pairing after setup
  scripts/set_ds5.py --auto-haptics fallback --auto-haptics-gain 120
  scripts/set_ds5.py --haptics-gain 1.5 --speaker-volume -10
  scripts/set_ds5.py --slot 2              # switch active pairing slot
  scripts/set_ds5.py --slots               # dump slot table (BT addr + name)
  scripts/set_ds5.py --slot-set 0 --slot-addr 12:34:56:78:9A:BC --slot-name "Volcanic Red"
  scripts/set_ds5.py --slot-set 1 --slot-name "Cosmic Red"   # rename, keep addr
  scripts/set_ds5.py --version             # print firmware version
  scripts/set_ds5.py --rssi                # print live BT RSSI in dBm
  scripts/set_ds5.py --latency             # print dongle latency telemetry

Credit: protocol + script structure from loteran/DS5Dongle commit 5d6bc2f.
"""

import argparse
import struct
import sys

try:
    import hid
except ImportError:
    print("[ERROR] Missing dependency: install with  pip install hidapi", file=sys.stderr)
    sys.exit(1)


def _open_hid(vid, pid):
    """Open a HID device, abstracting over cython-hidapi vs apmorton/pyhidapi."""
    if hasattr(hid, 'Device'):                   # apmorton/pyhidapi: hid.Device(vid, pid)
        return hid.Device(vid, pid)
    if hasattr(hid, 'device'):                   # cython-hidapi: hid.device() + .open(vid, pid)
        d = hid.device()
        d.open(vid, pid)
        return d
    raise RuntimeError("installed `hid` module has neither Device nor device — unknown variant")

SONY_VID = 0x054C
DS5_PID  = 0x0CE6
DSE_PID  = 0x0DF2

# Config_body wire layouts (little-endian, packed struct), by firmware era:
#   19 bytes — pre-lightbar (≤ v0.6.4)
#   37 bytes — lightbar/screen/mic/brightness/ctrl-wake era (v0.6.5 … v0.6.12)
#   39 bytes — 4-Player Edition (adds player_id + pair_lock at the end)
#
# Offsets (39-byte layout):
#   uint8  config_version          [0]      (firmware-set, ignored on write)
#   float  haptics_gain            [1:5]
#   float  speaker_volume          [5:9]
#   uint8  inactive_time           [9]
#   uint8  disable_inactive_disc   [10]
#   uint8  disable_pico_led        [11]
#   uint8  polling_rate_mode       [12]
#   uint8  audio_buffer_length     [13]
#   uint8  controller_mode         [14]
#   uint8  current_slot            [15]
#   uint8  auto_haptics_enable     [16]     0=off 1=fallback 2=mix 3=replace
#   uint8  auto_haptics_gain       [17]     [0..200] percent
#   uint8  auto_haptics_lowpass    [18]     0=80Hz 1=160Hz 2=250Hz 3=400Hz
#   uint8  lightbar_mode           [19]     0=LIVE 1..4=FAV 5..7=effects 8=HOST
#   uint8  lb_fav_r[4]             [20:24]
#   uint8  lb_fav_g[4]             [24:28]
#   uint8  lb_fav_b[4]             [28:32]
#   uint8  screen_dim_timeout      [32]     minutes, 0=off
#   uint8  screen_off_timeout      [33]     minutes, 0=off
#   uint8  bt_mic_enable           [34]
#   uint8  screen_brightness       [35]
#   uint8  controller_wakes_disp   [36]
#   uint8  player_id               [37]     0=off, 1..4  (4-Player Edition)
#   uint8  pair_lock               [38]     0=open, 1=locked (4-Player Edition)
FMT_19 = '<BffBBBBBBBBBB'
FMT_37 = FMT_19 + 'B12BBBBBB'
FMT_39 = FMT_37 + 'BB'
assert struct.calcsize(FMT_19) == 19
assert struct.calcsize(FMT_37) == 37
assert struct.calcsize(FMT_39) == 39

FIELDS_19 = [
    'config_version',
    'haptics_gain', 'speaker_volume',
    'inactive_time', 'disable_inactive_disconnect', 'disable_pico_led',
    'polling_rate_mode', 'audio_buffer_length', 'controller_mode',
    'current_slot',
    'auto_haptics_enable', 'auto_haptics_gain', 'auto_haptics_lowpass',
]
FIELDS_37 = FIELDS_19 + [
    'lightbar_mode',
    'lb_fav_0', 'lb_fav_1', 'lb_fav_2', 'lb_fav_3',
    'lb_fav_4', 'lb_fav_5', 'lb_fav_6', 'lb_fav_7',
    'lb_fav_8', 'lb_fav_9', 'lb_fav_10', 'lb_fav_11',
    'screen_dim_timeout', 'screen_off_timeout',
    'bt_mic_enable', 'screen_brightness', 'controller_wakes_display',
]
FIELDS_39 = FIELDS_37 + ['player_id', 'pair_lock']

POLLING_MODES    = {0: "250 Hz", 1: "500 Hz", 2: "Real-time (1000 Hz)"}
CONTROLLER_MODES = {0: "DS5", 1: "DSE (Edge)", 2: "Auto"}
AUTO_HAP_MODES   = {
    0: "Off",
    1: "Fallback (only when game sends no native haptic)",
    2: "Mix (native + audio-derived)",
    3: "Replace (audio-derived only, override native)",
}
LOWPASS_MODES = {0: "80 Hz", 1: "160 Hz", 2: "250 Hz", 3: "400 Hz"}
LIGHTBAR_MODES = {0: "LIVE", 1: "FAV0", 2: "FAV1", 3: "FAV2", 4: "FAV3",
                  5: "Breathing", 6: "Rainbow", 7: "Fade", 8: "HOST (passthrough)"}
PLAYER_MODES = {0: "off", 1: "P1 (blue)", 2: "P2 (red)", 3: "P3 (green)", 4: "P4 (pink)",
                5: "P5 (orange)", 6: "P6 (cyan)", 7: "P7 (purple)", 8: "P8 (yellow)"}


def open_device():
    last_err = None
    for pid, label in [(DS5_PID, "DualSense"), (DSE_PID, "DualSense Edge")]:
        try:
            d = _open_hid(SONY_VID, pid)
            print(f"[INFO] Connected to {label} (VID:PID 0x{SONY_VID:04X}:0x{pid:04X})")
            return d
        except Exception as e:
            last_err = e
            continue
    print("[ERROR] No DS5/DSE device found. Pair the controller with the Pico first.", file=sys.stderr)
    if last_err is not None:
        print(f"        (last open() error: {type(last_err).__name__}: {last_err})", file=sys.stderr)
    sys.exit(1)


def detect_body_len(raw):
    """raw = full 0xF7 response incl. report-id echo at [0].

    The firmware appends a remap block ('R','M', proto, rev_lo, rev_hi,
    16 table entries) directly after Config_body when the request leaves
    room, so the marker position IS the body length. Validate the table
    entries (< 16 or 0xFF) to avoid a false 'RM' inside config bytes."""
    data = raw[1:]
    for bl in range(19, min(len(data) - 20, 60) + 1):
        if data[bl] == 0x52 and data[bl + 1] == 0x4D:  # 'R','M'
            table = data[bl + 5: bl + 21]
            if len(table) == 16 and all(v < 16 or v == 0xFF for v in table):
                return bl
    # No remap block (older firmware or short read): fall back to known sizes.
    for bl in (39, 37, 19):
        if len(data) >= bl:
            return bl
    return len(data)


def get_config(device):
    raw = bytes(device.get_feature_report(0xF7, 64))
    body_len = detect_body_len(raw)
    body = raw[1:1 + body_len]

    if body_len >= 39:
        fmt, fields = FMT_39, FIELDS_39
    elif body_len >= 37:
        fmt, fields = FMT_37, FIELDS_37
    elif body_len >= 19:
        fmt, fields = FMT_19, FIELDS_19
    else:
        print(f"[ERROR] Config too short ({body_len} bytes). Flash a newer firmware.", file=sys.stderr)
        sys.exit(1)

    values = struct.unpack(fmt, body[:struct.calcsize(fmt)])
    cfg = dict(zip(fields, values))
    cfg['_body_len'] = body_len
    return cfg


def write_config(device, cfg):
    body_len = cfg['_body_len']
    if body_len >= 39:
        fmt, fields = FMT_39, FIELDS_39
    elif body_len >= 37:
        fmt, fields = FMT_37, FIELDS_37
    else:
        fmt, fields = FMT_19, FIELDS_19
    body = struct.pack(fmt, *[
        (cfg[f] if isinstance(cfg[f], float) else int(cfg[f]) & 0xFF) for f in fields
    ])
    # 0xF6 set protocol:  [0x01, ...body...] = update in-memory  →  [0x02] = persist to flash.
    device.send_feature_report(b'\xf6\x01' + body)
    device.send_feature_report(b'\xf6\x02')


SLOT_NAME_LEN = 12

def get_slots(device):
    """Read the 0xFA slot dump: [4x addr(6)][4x occupied][4x name(12)] = 76.
    Falls back gracefully on pre-name firmware (28-byte legacy layout)."""
    raw = bytes(device.get_feature_report(0xFA, 77))   # id echo + up to 76
    data = raw[1:]
    if len(data) < 28:
        return None
    slots = []
    for i in range(4):
        addr = data[i * 6:i * 6 + 6]
        occ = data[24 + i] != 0
        name = ''
        if len(data) >= 76:
            name = data[28 + i * SLOT_NAME_LEN:28 + (i + 1) * SLOT_NAME_LEN] \
                .rstrip(b'\x00').decode('ascii', errors='replace')
        slots.append({'addr': ':'.join(f'{b:02X}' for b in addr), 'occupied': occ, 'name': name})
    return slots


def print_slots(device):
    slots = get_slots(device)
    if slots is None:
        print("[ERROR] slot dump unavailable", file=sys.stderr)
        return
    print("Slot table (this dongle):")
    for i, s in enumerate(slots):
        if s['occupied']:
            label = f"  {s['name']}" if s['name'] else "  (unnamed)"
            print(f"  slot {i}: {s['addr']}{label}")
        else:
            print(f"  slot {i}: (empty)")


def parse_bd_addr(text):
    parts = text.replace('-', ':').split(':')
    if len(parts) != 6:
        raise ValueError(f"bad BT address '{text}' (want AA:BB:CC:DD:EE:FF)")
    return bytes(int(p, 16) for p in parts)


def slot_write(device, op, slot, addr=b'\x00' * 6, name=''):
    """0xF6 func 0x11: op 1 = set addr+name, 2 = set name only, 3 = clear.
    Address byte order matches the Slots screen / 0xFA dump exactly."""
    nm = name.encode('ascii', errors='replace')[:SLOT_NAME_LEN]
    nm = nm + b'\x00' * (SLOT_NAME_LEN - len(nm))
    payload = bytes([0xF6, 0x11, ord('S'), ord('L'), 1, op, slot]) + addr + nm
    device.send_feature_report(payload)


def get_version(device):
    raw = bytes(device.get_feature_report(0xF8, 64))
    return raw[1:].rstrip(b'\x00').decode('ascii', errors='replace')


def get_rssi(device):
    raw = bytes(device.get_feature_report(0xF9, 64))
    if len(raw) < 2:
        return None
    val = raw[1]
    return val - 256 if val >= 128 else val   # int8


def get_latency(device):
    """Read the 0xFD bridge-diagnostics report and decode section 3
    (latency telemetry, firmware ≥ the LCD/Latency build — bytes 44..75).

    0xFD is intentionally NOT declared in the HID descriptor (declaring
    vendor IDs is what used to break native game triggers), which Linux
    hidraw doesn't mind but some Windows HID stacks refuse. On failure the
    dongle's on-screen Latency page shows the same numbers.
    """
    raw = bytes(device.get_feature_report(0xFD, 77))   # id echo + 76 payload
    data = raw[1:]
    if len(data) < 76:
        return None
    (cfg_mode, act_mode, bt_rate, usb_rate,
     avg_us, max_us, peak_us, gap_min, gap_max,
     disp_busy, variant, _rsv) = struct.unpack('<BBHHIIIIIIBB', data[44:76])
    return {
        'cfg_mode': cfg_mode, 'act_mode': act_mode,
        'bt_rate': bt_rate, 'usb_rate': usb_rate,
        'avg_us': avg_us, 'max_us': max_us, 'peak_us': peak_us,
        'gap_min_us': gap_min, 'gap_max_us': gap_max,
        'disp_busy_us': disp_busy, 'variant': variant,
    }


def fmt_latency(t):
    act = ("(not enumerated yet)" if t['act_mode'] == 0xFF
           else POLLING_MODES.get(t['act_mode'], '?'))
    lines = [
        f"  display variant    {'LCD (ST7789)' if t['variant'] else 'OLED (SH1107)'}",
        f"  polling configured {POLLING_MODES.get(t['cfg_mode'], '?')}",
        f"  polling enumerated {act}",
        f"  BT in rate         {t['bt_rate']} reports/s",
        f"  USB out rate       {t['usb_rate']} reports/s",
        f"  transit avg/max    {t['avg_us']} / {t['max_us']} µs  (last 1 s window)",
        f"  transit peak       {t['peak_us']} µs  (since boot)",
        f"  BT gap min-max     {t['gap_min_us']}-{t['gap_max_us']} µs",
        f"  display busy max   {t['disp_busy_us']} µs  (worst render+flush block)",
    ]
    if t['act_mode'] != 0xFF and t['act_mode'] != t['cfg_mode']:
        lines.append("  [!] polling mode changed since enumeration — replug USB to apply")
    return "\n".join(lines)


def fmt_cfg(c):
    lines = [
        f"  config_version     {c['config_version']}",
        f"  haptics_gain       {c['haptics_gain']:.3f}",
        f"  speaker_volume     {c['speaker_volume']:.1f} dB",
        f"  inactive_time      {c['inactive_time']} min",
        f"  inactive_disc      {'disabled' if c['disable_inactive_disconnect'] else 'enabled'}",
        f"  pico_led           {'off' if c['disable_pico_led'] else 'on'}",
        f"  polling_rate       {c['polling_rate_mode']} ({POLLING_MODES.get(c['polling_rate_mode'], '?')})",
        f"  audio_buffer       {c['audio_buffer_length']}",
        f"  controller_mode    {c['controller_mode']} ({CONTROLLER_MODES.get(c['controller_mode'], '?')})",
        f"  current_slot       {c['current_slot']}",
        f"  auto_haptics       {c['auto_haptics_enable']} ({AUTO_HAP_MODES.get(c['auto_haptics_enable'], '?')})",
        f"  auto_haptics_gain  {c['auto_haptics_gain']}%",
        f"  auto_haptics_lp    {c['auto_haptics_lowpass']} ({LOWPASS_MODES.get(c['auto_haptics_lowpass'], '?')})",
    ]
    if 'lightbar_mode' in c:
        lines += [
            f"  lightbar_mode      {c['lightbar_mode']} ({LIGHTBAR_MODES.get(c['lightbar_mode'], '?')})",
            f"  screen_dim/off     {c['screen_dim_timeout']} / {c['screen_off_timeout']} min",
            f"  bt_mic             {'on' if c['bt_mic_enable'] else 'off'}",
            f"  screen_brightness  {c['screen_brightness']}",
            f"  ctrl_wakes_display {'on' if c['controller_wakes_display'] else 'off'}",
        ]
    if 'player_id' in c:
        lines += [
            f"  player_id          {c['player_id']} ({PLAYER_MODES.get(c['player_id'], '?')})",
            f"  pair_lock          {'locked' if c['pair_lock'] else 'open'}",
        ]
    else:
        lines += ["  (player_id / pair_lock: firmware too old — flash the 4-Player Edition build)"]
    return "\n".join(lines)


AUTO_HAP_ARG = {'off': 0, 'fallback': 1, 'mix': 2, 'replace': 3}
LOWPASS_ARG  = {'80': 0, '160': 1, '250': 2, '400': 3}
CTRL_MODE_ARG = {'ds5': 0, 'dse': 1, 'auto': 2}
POLL_ARG     = {'250': 0, '500': 1, 'realtime': 2, 'rt': 2}
PLAYER_ARG   = {'off': 0, '1': 1, '2': 2, '3': 3, '4': 4,
                '5': 5, '6': 6, '7': 7, '8': 8}


def build_parser():
    p = argparse.ArgumentParser(description="DS5Dongle (OLED / 4-Player Edition) config tool.")
    p.add_argument('--version', action='store_true', help="print firmware version and exit")
    p.add_argument('--rssi', action='store_true', help="print live BT RSSI (dBm) and exit")
    p.add_argument('--latency', action='store_true',
                   help="print dongle latency telemetry (transit µs, rates, jitter) and exit")
    p.add_argument('--haptics-gain', type=float, help="float [1.0, 2.0]")
    p.add_argument('--speaker-volume', type=float, help="dB [-100, 0]")
    p.add_argument('--inactive-time', type=int, help="minutes [10, 60]")
    p.add_argument('--inactive-disc', choices=['on', 'off'], help="silent disconnect on idle")
    p.add_argument('--pico-led', choices=['on', 'off'])
    p.add_argument('--polling', choices=POLL_ARG.keys(), help="USB HID polling rate")
    p.add_argument('--audio-buffer', type=int, help="haptic buffer length [16, 128]")
    p.add_argument('--controller-mode', choices=CTRL_MODE_ARG.keys())
    p.add_argument('--slot', type=int, choices=[0, 1, 2, 3], help="active multi-pairing slot")
    p.add_argument('--auto-haptics', choices=AUTO_HAP_ARG.keys(), help="Auto Haptics mode")
    p.add_argument('--auto-haptics-gain', type=int, help="percent [0, 200]")
    p.add_argument('--auto-haptics-lp', choices=LOWPASS_ARG.keys(), help="LP cutoff Hz")
    p.add_argument('--player', choices=PLAYER_ARG.keys(),
                   help="player identity for THIS dongle (off, 1-8; 5+ defaults BT mic off)")
    p.add_argument('--bt-mic', choices=['on', 'off'],
                   help="DualSense mic over BT (costs controller battery + 2.4 GHz airtime)")
    p.add_argument('--slots', action='store_true',
                   help="print this dongle's slot table (BT address + user name per slot)")
    p.add_argument('--slot-set', type=int, choices=[0, 1, 2, 3], metavar='N',
                   help="write slot N: combine with --slot-addr and/or --slot-name")
    p.add_argument('--slot-addr', metavar='AA:BB:CC:DD:EE:FF',
                   help="BT address for --slot-set (as shown by --slots)")
    p.add_argument('--slot-name', metavar='NAME',
                   help="user label for --slot-set, max 12 chars (e.g. \"Volcanic Red\")")
    p.add_argument('--slot-clear', type=int, choices=[0, 1, 2, 3], metavar='N',
                   help="clear slot N (address + name)")
    p.add_argument('--pair-lock', choices=['on', 'off'],
                   help="4-Player Edition: lock pairing to the already-bonded controllers")
    return p


def main():
    args = build_parser().parse_args()
    device = open_device()

    # Slot bookkeeping ops are terminal commands, like the reads below.
    if args.slot_set is not None or args.slot_clear is not None:
        if args.slot_name and len(args.slot_name) > SLOT_NAME_LEN:
            print(f"[note] name truncated to {SLOT_NAME_LEN} chars: "
                  f"'{args.slot_name[:SLOT_NAME_LEN]}'")
        if args.slot_clear is not None:
            slot_write(device, 3, args.slot_clear)
            print(f"slot {args.slot_clear} cleared")
        if args.slot_set is not None:
            if args.slot_addr:
                addr = parse_bd_addr(args.slot_addr)
                slot_write(device, 1, args.slot_set, addr, args.slot_name or '')
            elif args.slot_name is not None:
                slot_write(device, 2, args.slot_set, name=args.slot_name)
            else:
                print("[ERROR] --slot-set needs --slot-addr and/or --slot-name",
                      file=sys.stderr)
                sys.exit(1)
        print_slots(device)
        return
    if args.slots:
        print_slots(device)
        return

    if args.version:
        print(f"Firmware: {get_version(device)}")
        return
    if args.rssi:
        rssi = get_rssi(device)
        print(f"BT RSSI: {rssi} dBm" if rssi is not None else "RSSI unavailable")
        return
    if args.latency:
        try:
            t = get_latency(device)
        except Exception as e:
            print(f"[ERROR] 0xFD read failed ({type(e).__name__}: {e}).", file=sys.stderr)
            print("        Windows refuses undeclared feature-report IDs; use the", file=sys.stderr)
            print("        dongle's on-screen Latency page (same numbers) or Linux hidraw.", file=sys.stderr)
            sys.exit(1)
        if t is None:
            print("[ERROR] Firmware too old — no latency section in 0xFD. Flash the "
                  "Latency/LCD build first.", file=sys.stderr)
            sys.exit(1)
        print("Latency telemetry:")
        print(fmt_latency(t))
        return

    cfg = get_config(device)
    changes = []

    def set_kv(key, val, label=None):
        if key not in cfg:
            print(f"[ERROR] Connected firmware has no '{label or key}' field — "
                  "flash the 4-Player Edition build first.", file=sys.stderr)
            sys.exit(1)
        if cfg[key] != val:
            changes.append(f"  {label or key}: {cfg[key]} → {val}")
            cfg[key] = val

    if args.haptics_gain is not None:    set_kv('haptics_gain', args.haptics_gain)
    if args.speaker_volume is not None:  set_kv('speaker_volume', args.speaker_volume)
    if args.inactive_time is not None:   set_kv('inactive_time', args.inactive_time)
    if args.inactive_disc is not None:
        set_kv('disable_inactive_disconnect', 1 if args.inactive_disc == 'off' else 0,
               label='inactive_disc')
    if args.pico_led is not None:
        set_kv('disable_pico_led', 1 if args.pico_led == 'off' else 0, label='pico_led')
    if args.polling is not None:         set_kv('polling_rate_mode', POLL_ARG[args.polling])
    if args.audio_buffer is not None:    set_kv('audio_buffer_length', args.audio_buffer)
    if args.controller_mode is not None: set_kv('controller_mode', CTRL_MODE_ARG[args.controller_mode])
    if args.slot is not None:            set_kv('current_slot', args.slot)
    if args.auto_haptics is not None:    set_kv('auto_haptics_enable', AUTO_HAP_ARG[args.auto_haptics])
    if args.auto_haptics_gain is not None: set_kv('auto_haptics_gain', args.auto_haptics_gain)
    if args.auto_haptics_lp is not None: set_kv('auto_haptics_lowpass', LOWPASS_ARG[args.auto_haptics_lp])
    if args.player is not None:
        set_kv('player_id', PLAYER_ARG[args.player])
        # 8-Player extension: picking P5..P8 defaults the BT mic off (airtime),
        # unless --bt-mic was given explicitly in the same invocation.
        if PLAYER_ARG[args.player] >= 5 and args.bt_mic is None:
            set_kv('bt_mic_enable', 0, label='bt_mic (auto-off for P5-P8)')
    if args.bt_mic is not None:
        set_kv('bt_mic_enable', 1 if args.bt_mic == 'on' else 0, label='bt_mic')
    if args.pair_lock is not None:
        set_kv('pair_lock', 1 if args.pair_lock == 'on' else 0)

    if changes:
        print("Updating config:")
        print("\n".join(changes))
        write_config(device, cfg)
        print("Saved to flash.")
        # Re-read to show what stuck after firmware validation
        cfg = get_config(device)
        print("\nNew config:")
    else:
        print("Current config:")
    print(fmt_cfg(cfg))


if __name__ == '__main__':
    main()
