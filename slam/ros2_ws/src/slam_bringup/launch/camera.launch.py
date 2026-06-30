"""Mono camera on /dev/video0 -> /rgb/image_raw + /rgb/camera_info (plan A3).

Uses our own cam_publisher (OpenCV) instead of usb_cam, which SIGABRTs on this
Jetson. Standalone so it can also feed image-based tools:
  ros2 launch slam_bringup camera.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = DeclareLaunchArgument('device', default_value='0')
    fps = DeclareLaunchArgument('fps', default_value='15.0')

    cam = Node(
        package='slam_bringup', executable='cam_publisher', name='cam_publisher',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'fps': LaunchConfiguration('fps'),
            'frame_id': 'camera_optical_frame',
        }],
    )
    return LaunchDescription([device, fps, cam])
