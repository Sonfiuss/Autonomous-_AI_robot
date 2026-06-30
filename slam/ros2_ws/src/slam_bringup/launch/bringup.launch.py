"""Full mono RGB-D RTAB-Map SLAM stack (plan steps A3-A8 + Part B).

  source slam/env.sh
  ros2 launch slam_bringup bringup.launch.py            # mapping (needs scale.yaml)
  ros2 launch slam_bringup bringup.launch.py rtabmap_args:=--delete_db_on_start  # fresh seed map

Brings up: usb_cam -> /rgb/image_raw, static_extrinsics (TF), serial_bridge
(/odom + /cmd_teleop), depth_anything_node (/depth/image_raw), rtabmap (RGB-D).
Run slam/calibrate_scale.py FIRST (port-free) so scale.yaml exists; otherwise the
depth node falls back to calibration mode (raw relative depth, map will be wrong).
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_RTABMAP_YAML = os.path.expanduser('~/Documents/slam/config/rtabmap.yaml')


def generate_launch_description():
    share = get_package_share_directory('slam_bringup')

    encoder = DeclareLaunchArgument('encoder', default_value='vits')
    rtab_args = DeclareLaunchArgument('rtabmap_args', default_value='')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'camera.launch.py')))

    extrinsics = Node(package='slam_bringup', executable='static_extrinsics',
                      name='static_extrinsics', output='screen')

    serial = Node(package='serial_bridge', executable='serial_bridge',
                  name='serial_bridge', output='screen')

    depth = Node(package='depth_anything_node', executable='depth_anything_node',
                 name='depth_anything_node', output='screen',
                 parameters=[{'encoder': LaunchConfiguration('encoder')}])

    rtabmap = Node(
        package='rtabmap_ros', executable='rtabmap', name='rtabmap',
        output='screen',
        parameters=[_RTABMAP_YAML],
        arguments=[LaunchConfiguration('rtabmap_args')],
        remappings=[
            ('rgb/image', '/rgb/image_raw'),
            ('rgb/camera_info', '/rgb/camera_info'),
            ('depth/image', '/depth/image_raw'),
            ('odom', '/odom'),
        ],
    )

    return LaunchDescription([
        encoder, rtab_args, camera, extrinsics, serial, depth, rtabmap])
