import math

from src import kinematics
from config import settings


def test_forward_zero_speeds():
    vx, vy, omega = kinematics.forward_kinematics([0, 0, 0], [0, 0, 0], settings.ROBOT_RADIUS)
    assert vx == 0
    assert vy == 0
    assert omega == 0


def test_inverse_and_forward_roundtrip():
    vx, vy, omega = 0.1, 0.0, 0.2
    angles = [math.radians(a) for a in settings.WHEEL_ANGLES]
    speeds = kinematics.inverse_kinematics(vx, vy, omega, angles, settings.ROBOT_RADIUS)
    fx, fy, fo = kinematics.forward_kinematics(speeds, angles, settings.ROBOT_RADIUS)
    assert math.isclose(fx, vx, rel_tol=1e-6)
    assert math.isclose(fy, vy, rel_tol=1e-6)
    assert math.isclose(fo, omega, rel_tol=1e-6)
