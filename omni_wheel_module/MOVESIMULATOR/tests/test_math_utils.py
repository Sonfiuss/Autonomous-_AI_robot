import math

from utils import math_utils


def test_normalize_angle_wraps():
    assert math_utils.normalize_angle(math.pi) <= math.pi
    assert math_utils.normalize_angle(-math.pi) >= -math.pi


def test_clamp():
    assert math_utils.clamp(5, 0, 10) == 5
    assert math_utils.clamp(-1, 0, 10) == 0
    assert math_utils.clamp(11, 0, 10) == 10
