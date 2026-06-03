#!/usr/bin/env python3
"""Decode raw Opus frames captured from firmware serial output.

Reads [OPUS_FRAME_N] hex lines from stdin or a file, decodes each frame
with libopus, and writes the result as a WAV file + prints a summary
(peak amplitude, zero-crossing rate) to diagnose encoder output quality.
"""
import ctypes
import struct
import sys
import wave
import re

lib = ctypes.cdll.LoadLibrary("libopus.so.0")

SAMPLE_RATE = 48000
CHANNELS = 2
FRAME_SIZE = 480  # 10ms at 48kHz

# Create decoder
err = ctypes.c_int(0)
decoder = lib.opus_decoder_create(SAMPLE_RATE, CHANNELS, ctypes.byref(err))
if err.value != 0:
    print(f"opus_decoder_create failed: {err.value}", file=sys.stderr)
    sys.exit(1)

infile = sys.argv[1] if len(sys.argv) > 1 else "/tmp/ds5_opus.log"
with open(infile) as f:
    lines = f.readlines()

frames = []
for line in lines:
    m = re.match(r'\[OPUS_FRAME_\d+\]\s+([0-9a-fA-F]+)', line.strip())
    if m:
        frames.append(bytes.fromhex(m.group(1)))

if not frames:
    print("No [OPUS_FRAME_N] lines found in input.", file=sys.stderr)
    sys.exit(1)

print(f"Found {len(frames)} Opus frames, decoding...")

all_pcm = b""
for i, frame_data in enumerate(frames):
    pcm = (ctypes.c_int16 * (FRAME_SIZE * CHANNELS))()
    ret = lib.opus_decode(
        decoder,
        frame_data, len(frame_data),
        pcm, FRAME_SIZE,
        0  # no FEC
    )
    if ret < 0:
        errstr = lib.opus_strerror(ret)
        print(f"  Frame {i+1}: DECODE ERROR {ret} ({ctypes.string_at(errstr).decode()})")
        continue

    samples = list(pcm)
    peak = max(abs(s) for s in samples)
    nonzero = sum(1 for s in samples if s != 0)
    print(f"  Frame {i+1}: {ret} samples decoded, peak={peak}, nonzero={nonzero}/{len(samples)}")
    print(f"    TOC=0x{frame_data[0]:02x}  first 8 bytes: {frame_data[:8].hex()}")
    all_pcm += struct.pack(f"<{ret * CHANNELS}h", *samples[:ret * CHANNELS])

outpath = "/tmp/ds5_opus_decoded.wav"
with wave.open(outpath, "w") as wf:
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(2)
    wf.setframerate(SAMPLE_RATE)
    wf.writeframes(all_pcm)

print(f"\nDecoded audio written to {outpath}")
print(f"Play with: aplay {outpath}")
print(f"View spectrogram: sox {outpath} -n spectrogram -o /tmp/ds5_opus_spectrogram.png")
