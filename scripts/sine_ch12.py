#!/usr/bin/env python3
"""Send a 440 Hz sine wave on channels 1+2 only (speaker), silence on 3+4 (haptic).

Usage: python3 scripts/sine_ch12.py [seconds]
"""
import subprocess, struct, math, sys, re

RATE = 48000
FREQ = 440
DURATION = int(sys.argv[1]) if len(sys.argv) > 1 else 5
CHANNELS = 4
SAMPLES = RATE * DURATION

# Generate 4-channel S16_LE: sine on ch1+ch2, silence on ch3+ch4
data = bytearray()
for i in range(SAMPLES):
    val = int(6000 * math.sin(2 * math.pi * FREQ * i / RATE))   # ~-15 dBFS (was 32767 / 0 dBFS) — clipping test
    s = struct.pack('<h', val)
    data += s + s + b'\x00\x00' + b'\x00\x00'  # L, R, hapL=0, hapR=0

# Auto-detect the dongle's ALSA card — it enumerates as "DualSense Wireless
# Controller", and the card number shifts across reboots, so never hardcode it.
def find_dualsense_card():
    out = subprocess.check_output(['aplay', '-l'], text=True)
    for line in out.splitlines():
        m = re.match(r'card (\d+):', line)
        if m and 'DualSense' in line:
            return int(m.group(1))
    return None

card = find_dualsense_card()
if card is None:
    print("DualSense dongle not found in `aplay -l` — paired and enumerated?", file=sys.stderr)
    sys.exit(1)
device = f'hw:{card},0'
print(f"Playing to {device} (DualSense dongle)")

proc = subprocess.Popen(
    ['aplay', '-D', device, '-f', 'S16_LE', '-c', '4', '-r', '48000', '-'],
    stdin=subprocess.PIPE
)
proc.stdin.write(data)
proc.stdin.close()
proc.wait()
print(f"Played {DURATION}s of {FREQ} Hz sine on ch1+ch2 (speaker only)")
