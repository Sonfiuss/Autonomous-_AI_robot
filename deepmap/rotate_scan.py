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

# Firmware-authoritative physical constants (motivation/esp32_unified_controller/PinConfig.h).
# Used only to log the EXPECTED commanded rotation alongside the measured odom theta,
# so estimated->actual rotation can be fine-tuned later from timing_log.csv.
ROBOT_RADIUS_M = 0.21
WHEEL_RADIUS_M = 0.055
STEPS_PER_REV  = 12800


def steps_for_turn(deg):
    """Per-wheel step count the ESP32 'T' command issues for a body rotation."""
    return (ROBOT_RADIUS_M / WHEEL_RADIUS_M) * (abs(deg) / 360.0) * STEPS_PER_REV


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
    p.add_argument('--test-round360', action='store_true',
                   help='Test mode: skip camera AND UART; use a fixed image '
                        '(--test-image) for every rotation frame')
    p.add_argument('--test-image',
                   default=os.path.join(ROOT, '..', 'depth-anything', 'assets', 'right_1.jpg'),
                   help='Image substituted for every frame in --test-round360')
    return p.parse_args()


# ── Test_round360: fixed-image rotation (no camera, no robot) ─────────────────

def run_test_round360(args):
    """Replace every 360° frame with a fixed image; write shots + manifest."""
    src = os.path.abspath(args.test_image)
    if not os.path.isfile(src):
        print(f'ERROR: test image not found: {src}')
        return
    img = cv2.imread(src)
    if img is None:
        print(f'ERROR: cannot read test image: {src}')
        return

    n_steps = round(360.0 / args.step_deg)
    print(f'TEST_ROUND360: {n_steps} frames all = {os.path.basename(src)} '
          f'(no camera, no UART)')

    manifest = []
    for i in range(n_steps):
        fname = f'shot_{i:03d}.jpg'
        cv2.imwrite(os.path.join(args.out_dir, fname), img)
        angle = (args.step_deg * i) % 360.0
        manifest.append((fname, f'{angle:.2f}'))
        print(f'  [{i+1}/{n_steps}] {fname} @ {angle:.1f}°  (fixed image)')

    csv_path = os.path.join(args.out_dir, 'angles.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['# filename', 'angle_deg'])
        w.writerows(manifest)
    print(f'\nDone (test). {len(manifest)} shots -> {args.out_dir}')
    print(f'Next: python build_map.py --shots {args.out_dir} --test-round360')


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


def write_timing_log(out_dir, timing):
    """Write per-step stats to timing_log.csv. Returns path or None if empty."""
    if not timing:
        return None
    path = os.path.join(out_dir, 'timing_log.csv')
    cols = ['step', 'file', 'cmd_step_deg', 'expected_deg', 'odom_theta_deg',
            'theta_err_deg', 'expected_steps_per_wheel', 'capture_ms', 'move_ms',
            'wall_ts']
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(timing)
    return path


# ── Main ────────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    if args.test_round360:
        run_test_round360(args)
        return

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
    timing = []                 # secondary log: per-step stats for fine-tuning
    cumulative_deg = 0.0

    try:
        for i in range(n_steps):
            # Capture frame at current position (timed)
            t_cap0 = time.perf_counter()
            ret, frame = cap.read()
            fname = ''
            if not ret:
                print(f'  WARNING: Camera read failed at step {i}, skipping')
                cap_ms = (time.perf_counter() - t_cap0) * 1000.0
            else:
                fname = f'shot_{i:03d}.jpg'
                fpath = os.path.join(args.out_dir, fname)
                cv2.imwrite(fpath, frame)
                cap_ms = (time.perf_counter() - t_cap0) * 1000.0
                angle = cumulative_deg % 360.0
                manifest.append((fname, f'{angle:.2f}'))
                print(f'  [{i+1}/{n_steps}] {fname} @ {angle:.1f}°  saved')

            # Rotate to next position (timed: command -> K ack)
            move_ms = 0.0
            theta = None
            expected_deg = (cumulative_deg + args.step_deg) % 360.0
            if ser is not None:
                t_mv0 = time.perf_counter()
                send(ser, f'T {args.step_deg:.4f} {args.omega:.4f}\n')
                ok, theta = wait_ack(ser, args.move_timeout)
                move_ms = (time.perf_counter() - t_mv0) * 1000.0
                if not ok:
                    print(f'  WARNING: Timeout waiting for ack at step {i}')
                elif theta is not None:
                    err = theta - expected_deg
                    print(f'    odom theta={theta:.1f}° (expected {expected_deg:.1f}°, '
                          f'err {err:+.1f}°)  move {move_ms:.0f} ms')

            timing.append({
                'step': i,
                'file': fname,
                'cmd_step_deg': args.step_deg,
                'expected_deg': round(expected_deg, 2),
                'odom_theta_deg': '' if theta is None else round(theta, 2),
                'theta_err_deg': '' if theta is None else round(theta - expected_deg, 2),
                'expected_steps_per_wheel': round(steps_for_turn(args.step_deg), 1),
                'capture_ms': round(cap_ms, 1),
                'move_ms': round(move_ms, 1),
                'wall_ts': round(time.time(), 3),
            })

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

    # Write timing log (secondary stats for fine-tuning estimated->actual rotation)
    timing_path = write_timing_log(args.out_dir, timing)

    print(f'\nDone. {len(manifest)} shots → {args.out_dir}')
    print(f'Manifest: {csv_path}')
    if timing_path:
        print(f'Timing log: {timing_path}')
        caps = [t['capture_ms'] for t in timing]
        moves = [t['move_ms'] for t in timing if t['move_ms'] > 0]
        if caps:
            print(f'  avg capture {sum(caps)/len(caps):.0f} ms'
                  + (f' | avg move {sum(moves)/len(moves):.0f} ms' if moves else ''))
    print(f'\nNext step:')
    print(f'  python build_map.py --shots {args.out_dir}')


if __name__ == '__main__':
    main()
