#!/usr/bin/env python3
"""static_extrinsics — publish the camera TF chain (plan step A6).

  base_link --(extrinsics.yaml)--> camera_link --(optical rot)--> camera_optical_frame

camera_link uses the ROS body convention (x fwd, y left, z up); the optical frame
uses (x right, y down, z fwd). Getting this chain wrong is the classic source of a
90deg-rotated or mirrored map. Values come from slam/config/extrinsics.yaml so the
15.6 cm eccentric offset is exact (else static objects smear into arcs after a sweep).

No tf_transformations dependency — quaternions are built locally.
"""
import math
import os

import yaml
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

_EXTR = os.path.expanduser('~/Documents/slam/config/extrinsics.yaml')


def euler_to_quat(roll, pitch, yaw):
    """Intrinsic RPY (XYZ) -> (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def make_tf(stamp, parent, child, xyz, quat):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
    return t


class StaticExtrinsics(Node):
    def __init__(self):
        super().__init__('static_extrinsics')
        with open(_EXTR) as fh:
            cfg = yaml.safe_load(fh)
        c = cfg['base_link_to_camera']

        link_q = euler_to_quat(
            math.radians(c['roll_deg']),
            math.radians(c['pitch_deg']),
            math.radians(c['yaw_deg']))
        # camera_link -> camera_optical_frame : fixed (-90, 0, -90) deg
        opt_q = euler_to_quat(-math.pi / 2, 0.0, -math.pi / 2)

        stamp = self.get_clock().now().to_msg()
        self.bc = StaticTransformBroadcaster(self)
        self.bc.sendTransform([
            make_tf(stamp, 'base_link', 'camera_link',
                    (c['cam_x'], c['cam_y'], c['cam_z']), link_q),
            make_tf(stamp, 'camera_link', 'camera_optical_frame',
                    (0.0, 0.0, 0.0), opt_q),
        ])
        self.get_logger().info(
            f"static TF: base_link->camera_link "
            f"({c['cam_x']},{c['cam_y']},{c['cam_z']}) "
            f"pitch={c['pitch_deg']}deg, +optical frame")


def main(args=None):
    rclpy.init(args=args)
    node = StaticExtrinsics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
