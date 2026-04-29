"""
receive.py – Serial monitor + audio receiver for ESP32 (single COM port).

Usage:
    python receive.py COM7          # Windows
    python receive.py /dev/ttyUSB0  # Linux / macOS

The script acts as both a Serial Monitor and a WAV receiver:
  - All text lines from ESP32 are printed to the console as debug output.
  - When the binary frame arrives, it is captured and saved as a WAV file.
  - After saving, the script resumes printing debug text.

Wire protocol sent by mic_send_serial():
    "AUDIO_START\\n"     ASCII start marker
    uint32_t numSamples  4 bytes, little-endian
    uint32_t sampleRate  4 bytes, little-endian
    int16_t  samples[]   numSamples × 2 bytes, raw 16-bit mono PCM
    "AUDIO_END\\n"       ASCII end marker
"""

import serial
import struct
import wave
import os
import sys
from datetime import datetime

# ── Configuration ─────────────────────────────────────────────────
PORT    = sys.argv[1] if len(sys.argv) > 1 else "COM7"
BAUD    = 115200
OUT_DIR = os.path.dirname(os.path.abspath(__file__))

START_MARKER = b"AUDIO_START\n"
END_MARKER   = b"AUDIO_END\n"


def receive_audio(ser: serial.Serial) -> None:
    """Read metadata + PCM payload, then save as WAV."""
    meta = ser.read(8)
    if len(meta) < 8:
        print("[receive.py] ERROR: Incomplete metadata header.")
        return

    num_samples, sample_rate = struct.unpack("<II", meta)
    data_bytes = num_samples * 2
    print(f"[receive.py] {num_samples} samples @ {sample_rate} Hz "
          f"({data_bytes / 1024:.1f} KB) – receiving...")

    # Read raw PCM
    pcm = b""
    while len(pcm) < data_bytes:
        chunk = ser.read(min(4096, data_bytes - len(pcm)))
        if not chunk:
            print("[receive.py] ERROR: Timeout while reading PCM data.")
            return
        pcm += chunk

    # Consume end marker (may already be buffered)
    end = ser.read(len(END_MARKER))
    if end != END_MARKER:
        print(f"[receive.py] WARNING: Expected AUDIO_END, got {end!r}")

    # Save WAV
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path  = os.path.join(OUT_DIR, f"recording_{timestamp}.wav")
    with wave.open(out_path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(pcm)

    print(f"[receive.py] Saved → {out_path}  ({os.path.getsize(out_path):,} bytes)")


def main() -> None:
    print(f"[receive.py] Opening {PORT} @ {BAUD} baud  (Ctrl+C to quit)")

    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        # Slide a window over incoming bytes.
        # Text lines are decoded and printed; binary frames are captured.
        window = b""
        line   = b""

        while True:
            data = ser.read(1)
            if not data:
                continue   # 1-second read timeout, loop again

            byte = data
            window += byte

            # Keep window at most as long as START_MARKER
            if len(window) > len(START_MARKER):
                window = window[-len(START_MARKER):]

            # ── Detected AUDIO_START → switch to binary mode ──────
            if window == START_MARKER:
                line  = b""   # discard any partial text line
                window = b""
                receive_audio(ser)
                continue

            # ── Normal text byte → accumulate line ────────────────
            line += byte
            if byte == b"\n":
                try:
                    print(line.decode("utf-8", errors="replace"), end="")
                except Exception:
                    pass
                line = b""


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[receive.py] Stopped.")
