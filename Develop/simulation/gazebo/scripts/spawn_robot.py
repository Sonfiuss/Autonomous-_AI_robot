"""
Spawn Robot

Script spawn robot vào Gazebo simulation.
"""

# import rclpy
# from gazebo_msgs.srv import SpawnEntity


def spawn_robot(urdf_path: str, name: str = "robot", x: float = 0, y: float = 0):
    """
    Spawn robot vào Gazebo.

    Args:
        urdf_path: Đường dẫn file URDF
        name: Tên entity trong Gazebo
        x, y: Vị trí spawn
    """
    # TODO: Implement ROS2 service call
    # rclpy.init()
    # node = rclpy.create_node("spawn_robot")
    # client = node.create_client(SpawnEntity, "/spawn_entity")
    # ...
    print(f"Spawn robot '{name}' at ({x}, {y}) - chưa cấu hình ROS2")


if __name__ == "__main__":
    spawn_robot("../models/robot/robot.urdf.xacro")
