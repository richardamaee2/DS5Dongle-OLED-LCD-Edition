#!/usr/bin/env python3
"""Send a 440 Hz sine to the DualSense speaker over Bluetooth from a Linux host
(Raspberry Pi), via raw /dev/hidraw writes — replicating the DS5Dongle firmware's
0x36 audio report BYTE-FOR-BYTE (same Opus settings, same SetStateData, same CRC).

Purpose: test whether a NON-tunneled BT path (Pi's BlueZ over a real UART) delivers
clean audio to the DS5 speaker, vs the Pico 2 W's BT-over-gSPI tunnel. If clean here,
the Pico's tunnel is the crackle culprit.

Pair the DS5 first (bluetoothctl), then run as root (hidraw needs RW):
    sudo python3 pi_ds5_audio_test.py [seconds] [--headset] [--audbuf N]
"""
import ctypes, struct, math, sys, os, glob, time

# ---- libopus via ctypes (same loader style as decode_opus_dump.py) ----
opus = ctypes.cdll.LoadLibrary("libopus.so.0")
opus.opus_encoder_create.restype = ctypes.c_void_p
opus.opus_encoder_create.argtypes = [ctypes.c_int32, ctypes.c_int, ctypes.c_int,
                                      ctypes.POINTER(ctypes.c_int)]
opus.opus_encode_float.restype = ctypes.c_int32
opus.opus_encode_float.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float),
                                   ctypes.c_int, ctypes.POINTER(ctypes.c_ubyte),
                                   ctypes.c_int32]
# opus_encoder_ctl is variadic; leave argtypes unset and pass c_int values.
OPUS_APPLICATION_AUDIO      = 2049
OPUS_SET_BITRATE_REQUEST    = 4002
OPUS_SET_VBR_REQUEST        = 4006
OPUS_SET_COMPLEXITY_REQUEST = 4010

RATE, CH, FRAME, OPUS_BYTES = 48000, 2, 480, 200   # 480 = 10 ms; 200 B = 160 kbps CBR

def make_encoder():
    err = ctypes.c_int(0)
    enc = opus.opus_encoder_create(RATE, CH, OPUS_APPLICATION_AUDIO, ctypes.byref(err))
    if err.value != 0 or not enc:
        sys.exit(f"opus_encoder_create failed: {err.value}")
    enc_p = ctypes.c_void_p(enc)
    # match firmware core1_entry(): 160 kbps, CBR, complexity 0
    opus.opus_encoder_ctl(enc_p, OPUS_SET_BITRATE_REQUEST,    ctypes.c_int(OPUS_BYTES*8*100))
    opus.opus_encoder_ctl(enc_p, OPUS_SET_VBR_REQUEST,        ctypes.c_int(0))
    opus.opus_encoder_ctl(enc_p, OPUS_SET_COMPLEXITY_REQUEST, ctypes.c_int(0))
    return enc_p

# ---- 63-byte SetStateData, verbatim from firmware src/state_mgr.cpp ----
STATE_DATA = bytes([
    0xfd, 0xf7, 0x00, 0x00,
    0x7f, 0x64,                                            # VolHeadphonesMax, VolSpeaker
    0x40, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a,
    0x07, 0x00, 0x00, 0x02, 0x01,
    0x00,
    0xff, 0xd7, 0x00,                                      # RGB
]) + bytes(16)                                            # trailing zeros -> 63 total
assert len(STATE_DATA) == 63

# ---- CRC-32, 0xA2-seeded, verbatim from firmware src/utils.h ----
def ds_crc32(data):
    crc = (~0xEADA2D49) & 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0xEDB88320
            else:       crc >>= 1
    return (~crc) & 0xFFFFFFFF

REPORT_SIZE, SAMPLE_SIZE = 398, 64                        # original firmware layout

def build_packet(payload, seq, counter, headset, audbuf):
    pkt = bytearray(REPORT_SIZE)
    pkt[0]  = 0x36
    pkt[1]  = (seq & 0x0F) << 4
    pkt[2]  = 0x11 | 0x80                                  # 0x91
    pkt[3]  = 7
    pkt[4]  = 0xFE                                         # audio enable, mic OFF
    for i in range(5, 10): pkt[i] = audbuf & 0xFF
    pkt[10] = counter & 0xFF
    pkt[11] = 0x10 | 0x80                                  # 0x90 SetStateData
    pkt[12] = 63
    pkt[13:76] = STATE_DATA
    pkt[76] = 0x12 | 0x80                                  # 0x92 haptic
    pkt[77] = SAMPLE_SIZE                                  # haptic data pkt[78..141] = 0 (silent)
    pkt[142] = (0x16 if headset else 0x13) | 0x80          # 0x96 headset / 0x93 speaker
    pkt[143] = OPUS_BYTES
    pkt[144:144+OPUS_BYTES] = payload
    struct.pack_into("<I", pkt, REPORT_SIZE-4, ds_crc32(pkt[:REPORT_SIZE-4]))
    return bytes(pkt)

def find_ds5_hidraw():
    for ue in glob.glob("/sys/class/hidraw/hidraw*/device/uevent"):
        txt = open(ue).read().upper()
        if "054C" in txt and ("0CE6" in txt or "0DF2" in txt):
            return "/dev/" + ue.split("/")[4]
    return None

def main():
    headset = "--headset" in sys.argv
    audbuf  = int(sys.argv[sys.argv.index("--audbuf")+1]) if "--audbuf" in sys.argv else 64
    pos     = [a for a in sys.argv[1:] if not a.startswith("--") and a.isdigit()]
    secs    = int(pos[0]) if pos else 15

    node = find_ds5_hidraw()
    if not node:
        sys.exit("DualSense hidraw node not found — paired & connected? (bluetoothctl)")
    print(f"DS5 hidraw: {node} | output: {'HEADSET' if headset else 'SPEAKER'} | "
          f"AudBuf={audbuf} | {secs}s")

    enc = make_encoder()
    pcm = (ctypes.c_float * (FRAME*CH))()
    out = (ctypes.c_ubyte * OPUS_BYTES)()

    # Pre-build every packet so the timed send loop is pure I/O (no compute jitter).
    n_frames = secs * 100
    packets, seq, counter, samp = [], 0, 0, 0
    first_nb = None
    for _ in range(n_frames):
        for i in range(FRAME):
            v = 0.5 * math.sin(2*math.pi*440.0*samp/RATE)
            pcm[i*2] = v; pcm[i*2+1] = v; samp += 1
        nb = opus.opus_encode_float(enc, pcm, FRAME, out, OPUS_BYTES)
        if nb < 0: sys.exit(f"opus_encode_float error {nb}")
        if first_nb is None: first_nb = nb
        payload = bytes(out[:nb]) + bytes(OPUS_BYTES - nb) if nb < OPUS_BYTES else bytes(out[:OPUS_BYTES])
        packets.append(build_packet(payload, seq, counter, headset, audbuf))
        seq = (seq+1) & 0x0F; counter = (counter+1) & 0xFF
    print(f"Encoded {len(packets)} frames (first frame {first_nb} B — expect ~200 for CBR). Streaming...")

    fd = os.open(node, os.O_RDWR)
    short = 0
    t0 = time.monotonic()
    for n, pkt in enumerate(packets):
        try:
            w = os.write(fd, pkt)
        except OSError as e:
            print(f"write failed at frame {n}: {e}"); break
        if w != REPORT_SIZE: short += 1
        target = t0 + (n+1)*0.01                           # 10 ms grid -> 100 pkt/s
        dt = target - time.monotonic()
        if dt > 0: time.sleep(dt)
    os.close(fd)
    print(f"Done — {n+1} frames sent (~{(n+1)/100:.1f}s).", end="")
    print(f"  WARNING: {short} short writes (hidraw truncating the 398 B report!)" if short else "  All writes full-length.")

if __name__ == "__main__":
    main()
