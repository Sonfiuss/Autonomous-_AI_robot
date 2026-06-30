#!/usr/bin/env python3
"""robot_drive — drive the robot via the project's own control binaries.

The ESP32's CMD_FORWARD ('F') path does not move the steppers; the working path
is per-motor 'W <idx> <steps> <hz>'. The project already has two tools for this:

  core-control/build/core_control "move forward 30"  -> per-wheel angles (deg)
  motivation/build/stepper_ctrl  <W1> <W2> <W3>       -> sends 'W' to the ESP32,
                                                         blocks until 'K' (done)

This module chains them so Python (e.g. slam/calibrate_scale.py) can drive the
robot with one call instead of the obsolete 'F' serial command.

NOTE: stepper_ctrl OWNS /dev/ttyUSB0 for the duration of the move. Do not run it
while serial_bridge (or anything else) holds the port.
"""
import os
import re
import subprocess

_DOCS = os.path.expanduser('~/Documents')
CORE_CONTROL = os.path.join(_DOCS, 'core-control', 'build', 'core_control')
STEPPER_CTRL = os.path.join(_DOCS, 'motivation', 'build', 'stepper_ctrl')

# Matches "  W1 (150°):  -363.07 °"  -> captures the signed angle
_W_RE = re.compile(r'W[123]\s*\([^)]*\):\s*([+-]?\d+(?:\.\d+)?)')


def wheel_angles(command_str):
    """Run core_control on a natural-language command -> [W1, W2, W3] degrees."""
    if not os.path.isfile(CORE_CONTROL):
        raise FileNotFoundError(
            f"core_control not built: {CORE_CONTROL}\n"
            f"  build: (cd core-control && mkdir -p build && cd build && cmake .. && make)")
    out = subprocess.check_output([CORE_CONTROL, command_str],
                                  text=True, stderr=subprocess.STDOUT)
    vals = _W_RE.findall(out)
    if len(vals) != 3:
        raise RuntimeError(f"could not parse 3 wheel angles from core_control:\n{out}")
    return [float(v) for v in vals]


def drive(command_str, port='/dev/ttyUSB0', hz=3000, timeout=70.0, verbose=True):
    """Resolve a command to wheel angles and execute it on the robot (blocking).

    Returns True if stepper_ctrl reported completion ('K'), False otherwise.
    """
    if not os.path.isfile(STEPPER_CTRL):
        raise FileNotFoundError(
            f"stepper_ctrl not built: {STEPPER_CTRL}\n"
            f"  build: (cd motivation && mkdir -p build && cd build && cmake .. && make)")
    w1, w2, w3 = wheel_angles(command_str)
    if verbose:
        print(f"[robot_drive] '{command_str}' -> W1={w1:.2f} W2={w2:.2f} W3={w3:.2f} deg")
    cmd = [STEPPER_CTRL, f'{w1:.4f}', f'{w2:.4f}', f'{w3:.4f}',
           '--hz', str(int(hz)), '--port', port]
    try:
        r = subprocess.run(cmd, timeout=timeout)
    except subprocess.TimeoutExpired:
        print('[robot_drive] stepper_ctrl timed out')
        return False
    return r.returncode == 0


def drive_forward(a_cm, **kw):
    """Drive straight forward a_cm centimetres (blocking)."""
    return drive(f'move forward {a_cm}', **kw)


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='Drive the robot via core_control + stepper_ctrl')
    ap.add_argument('command', help='e.g. "move forward 30" / "spin right 45"')
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--hz', type=int, default=3000)
    args = ap.parse_args()
    ok = drive(args.command, port=args.port, hz=args.hz)
    print('OK' if ok else 'FAILED')
    raise SystemExit(0 if ok else 1)
