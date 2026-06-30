#!/usr/bin/env python3
"""Tim TILT_TRIM_DEG: gui lenh MOVE <pan> <tilt> toi ESP32 (firmware unified) va
xem camera. Goc TILT dua camera ve dung HOME chinh la gia tri dat cho
TILT_TRIM_DEG trong esp32_unified_controller.ino.

LUU Y: ESP32 phai dang chay firmware movement (esp32_unified_controller) va
/dev/ttyUSB0 phai RANH (khong chay stepper_ctrl / serial_bridge cung luc).

  python motivation/cam_pan.py --tilt 20     # gui MOVE 0 20  (thu 1 goc tilt)
  python motivation/cam_pan.py --sweep        # quet tilt -40..30, Enter giua cac buoc
"""
import argparse
import time

import serial


def send_move(ser, pan, tilt):
    ser.reset_input_buffer()
    ser.write(f'MOVE {pan:.1f} {tilt:.1f}\n'.encode())
    # firmware tra 'OK' sau SERVO_SETTLE_MS (~0.5s)
    t = time.time()
    while time.time() - t < 2.0:
        line = ser.readline().decode('ascii', 'ignore').strip()
        if line == 'OK':
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('pan', nargs='?', type=float, default=0.0)
    ap.add_argument('--tilt', type=float, default=0.0)
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--sweep', action='store_true',
                    help='quet pan -40..40 (buoc 10), Enter de sang goc tiep theo')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.3)
    time.sleep(0.3)

    if args.sweep:
        print('Quet TILT. Goc nao camera ve dung HOME (level) = TILT_TRIM_DEG.')
        for tilt in list(range(-40, 31, 10)):       # TILT_MIN -70 .. TILT_MAX 30
            ok = send_move(ser, args.pan, tilt)
            print(f'  pan={args.pan:.0f}  tilt={tilt:+d}  ({"OK" if ok else "no-ack"})  '
                  f'-> camera dung home chua? (Enter de tiep)')
            input()
    else:
        ok = send_move(ser, args.pan, args.tilt)
        print(f'MOVE {args.pan:.1f} {args.tilt:.1f}  ({"OK" if ok else "no-ack"})')
        print('Neu camera dung HOME -> dat TILT_TRIM_DEG = '
              f'{args.tilt:.1f} trong esp32_unified_controller.ino roi nap lai 1 lan.')
    ser.close()


if __name__ == '__main__':
    main()
