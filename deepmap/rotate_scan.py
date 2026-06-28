#!/usr/bin/env python3
"""
Feature 1 — Rotate robot 360° in steps and capture images.

The robot rotates in-place using the ESP32 Unified Controller over UART.
At each stop, a camera frame is saved and the current yaw angle is recorded.
Output: shots/ directory with images + angles.csv manifest for build_map.py.

Usage:
  python rotate_scan.py                          # 12 shots × 30° with defaults
  python rotate_scan.py --step-deg 45 --omega 0.8
  python rotate_scan.py --cam 1 --out-dir /tmp/scan
"""

import argparse
import csv
import os
import time

import cv2
import serial

ROOT = os.path.dirname(os.path.abspath(__file__))


def parse_args():
    p = argparse.ArgumentParser(description='Rotate robot 360° and capture depth-map frames')
    p.add_argument('--port',    default='/dev/ttyUSB0', help='ESP32 serial port')
    p.add_argument('--baud',    type=int, default=115200)
    p.add_argument('--step-deg', type=float, default=30.0,
                   help='Degrees per step (default 30 → 12 shots per full rotation)')
    p.add_argument('--omega',   type=float, default=0.5,
                   help='Turn speed in rad/s (default 0.5)')
    p.add_argument('--cam',     type=int, default=0, help='Camera device index')
    p.add_argument('--out-dir', default=os.path.join(ROOT, 'shots'))
    p.add_argument('--connect-timeout', type=float, default=10.0,
                   help='Seconds to wait for READY from ESP32')
    p.add_argument('--move-timeout',    type=float, default=30.0,
                   help='Seconds to wait for K ack per step')
    p.add_argument('--dry-run', action='store_true',
                   help='Capture images only, skip UART (for testing without robot)')
    return p.parse_args()


# ── Serial helpers ──────────────────────────────────────────────────────────────

def _read_line(ser, buf, timeout_abs):
    """Read from serial until newline or timeout. Returns (line_or_None, updated_buf)."""
    while time.time() < timeout_abs:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            if b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                line = raw.strip().decode('ascii', errors='ignore')
                return line, buf
    return None, buf


def wait_ready(ser, timeout_s):
    """Drain lines until READY or timeout. Returns True on success."""
    deadline = time.time() + timeout_s
    buf = b''
    while time.time() < deadline:
        line, buf = _read_line(ser, buf, deadline)
        if line is None:
            break
        if line == 'READY':
            return True
    return False


def wait_ack(ser, timeout_s):
    """
    Wait for K (motion-done ack). Also parse O lines to track last odometry theta.
    Returns (acked: bool, theta_deg: float|None).
    """
    deadline = time.time() + timeout_s
    buf = b''
    last_theta = None
    while time.time() < deadline:
        line, buf = _read_line(ser, buf, deadline)
        if line is None:
            break
        if line == 'K':
            return True, last_theta
        if line.startswith('O '):
            parts = line.split()
            if len(parts) == 4:
                try:
                    last_theta = float(parts[3])
                except ValueError:
                    pass
    return False, last_theta


def send(ser, msg: str):
    ser.write(msg.encode())


# ── Main ────────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    n_steps = round(360.0 / args.step_deg)
    print(f'Plan: {n_steps} steps × {args.step_deg}° = 360°, ω={args.omega} rad/s')

    # Open camera
    print(f'Opening camera {args.cam}...')
    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print(f'ERROR: Cannot open camera {args.cam}')
        return

    # Warm up camera (first frames are often dark/blurry)
    for _ in range(5):
        cap.read()

    ser = None
    if not args.dry_run:
        print(f'Connecting to ESP32 on {args.port} @ {args.baud}...')
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        if wait_ready(ser, args.connect_timeout):
            print('ESP32: READY')
        else:
            print('WARNING: READY not received — ESP32 may already be running, continuing')
        send(ser, 'R\n')   # reset odometry
        time.sleep(0.2)
    else:
        print('DRY RUN: UART disabled, capturing images only')

    manifest = []
    cumulative_deg = 0.0

    try:
        for i in range(n_steps):
            # Capture frame at current position
            ret, frame = cap.read()
            if not ret:
                print(f'  WARNING: Camera read failed at step {i}, skipping')
            else:
                fname = f'shot_{i:03d}.jpg'
                fpath = os.path.join(args.out_dir, fname)
                cv2.imwrite(fpath, frame)
                angle = cumulative_deg % 360.0
                manifest.append((fname, f'{angle:.2f}'))
                print(f'  [{i+1}/{n_steps}] {fname} @ {angle:.1f}°  saved')

            # Rotate to next position
            if ser is not None:
                send(ser, f'T {args.step_deg:.4f} {args.omega:.4f}\n')
                ok, theta = wait_ack(ser, args.move_timeout)
                if not ok:
                    print(f'  WARNING: Timeout waiting for ack at step {i}')
                elif theta is not None:
                    print(f'    ESP32 odometry theta = {theta:.1f}°')

            cumulative_deg += args.step_deg

    finally:
        if ser is not None:
            send(ser, 'S\n')
            ser.close()
        cap.release()

    # Write manifest
    csv_path = os.path.join(args.out_dir, 'angles.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['# filename', 'angle_deg'])
        w.writerows(manifest)

    print(f'\nDone. {len(manifest)} shots → {args.out_dir}')
    print(f'Manifest: {csv_path}')
    print(f'\nNext step:')
    print(f'  python build_map.py --shots {args.out_dir}')


if __name__ == '__main__':
    main()
