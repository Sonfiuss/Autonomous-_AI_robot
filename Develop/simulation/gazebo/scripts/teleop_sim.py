"""
Teleop Simulation

Điều khiển robot trong simulation bằng keyboard.
"""

# import rclpy
# from geometry_msgs.msg import Twist

KEY_BINDINGS = {
    "w": {"vx": 0.5, "vy": 0.0, "omega": 0.0},
    "s": {"vx": -0.5, "vy": 0.0, "omega": 0.0},
    "a": {"vx": 0.0, "vy": 0.5, "omega": 0.0},
    "d": {"vx": 0.0, "vy": -0.5, "omega": 0.0},
    "q": {"vx": 0.0, "vy": 0.0, "omega": 0.5},
    "e": {"vx": 0.0, "vy": 0.0, "omega": -0.5},
    " ": {"vx": 0.0, "vy": 0.0, "omega": 0.0},  # Stop
}


def teleop():
    """Điều khiển robot bằng keyboard."""
    # TODO: Implement với ROS2 Twist publisher
    # rclpy.init()
    # node = rclpy.create_node("teleop_sim")
    # pub = node.create_publisher(Twist, "/cmd_vel", 10)
    print("Teleop simulation - chưa cấu hình ROS2")
    print("Phím: W/S (tiến/lùi), A/D (trái/phải), Q/E (xoay), Space (dừng)")


if __name__ == "__main__":
    teleop()
