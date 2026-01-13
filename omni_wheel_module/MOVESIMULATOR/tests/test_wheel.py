from src.wheel import Wheel
from config import settings


def test_wheel_accelerates():
    wheel = Wheel(angle=0.0, max_speed=settings.MAX_WHEEL_SPEED)
    wheel.update(dt=0.5, acceleration=True, reverse=False)
    assert wheel.speed > 0


def test_wheel_reverse():
    wheel = Wheel(angle=0.0, max_speed=settings.MAX_WHEEL_SPEED)
    wheel.update(dt=0.5, acceleration=False, reverse=True)
    assert wheel.speed < 0
