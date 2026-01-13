from src.robot import Robot
from config import settings


def test_robot_reset_and_update():
    robot = Robot(start_pos=(0.0, 0.0))
    commands = [
        {"accelerate": True, "reverse": False},
        {"accelerate": False, "reverse": False},
        {"accelerate": False, "reverse": False},
    ]
    robot.update(settings.DT, commands)
    assert robot.position != [0.0, 0.0]
    robot.reset()
    assert robot.position == [0.0, 0.0]
