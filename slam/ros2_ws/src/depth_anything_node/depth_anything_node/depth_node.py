#!/usr/bin/env python3
"""depth_anything_node — mono RGB-D depth for RTAB-Map (plan step A4 / B1).

Subscribes /rgb/image_raw, runs Depth Anything V2, publishes /depth/image_raw
(32FC1, metres) with the ORIGINAL RGB header copied verbatim (plan A7 — RTAB-Map
pairs depth<->odom by timestamp; stamping with now() breaks the pose match).

Behaviour by scale.yaml presence (plan A4):
  * PRESENT -> mapping mode: Z = k / d_relative, clamped to [near_clip, d_max].
  * ABSENT  -> calibration mode: publishes raw relative depth and logs that you
    must run  slam/calibrate_scale.py  to produce scale.yaml first.

Reuses the deepmap / depth-anything pipeline directly (DRY): CFGS, pick_device,
step_prep, step_infer from depth_to_3d_timed; apply_scale from scale_calib.
"""
import os
import sys

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# --- reuse existing pipeline without copying code --------------------------
_DOCS = os.path.expanduser('~/Documents')
sys.path.insert(0, os.path.join(_DOCS, 'depth-anything', 'src'))
sys.path.insert(0, os.path.join(_DOCS, 'deepmap'))

from depth_to_3d_timed import CFGS, pick_device, step_prep, step_infer  # noqa: E402
from depth_anything_v2.dpt import DepthAnythingV2                        # noqa: E402
from scale_calib import apply_scale                                      # noqa: E402
import torch                                                             # noqa: E402

_MODEL_DIR = os.path.join(_DOCS, 'depth-anything', 'model')
_SCALE_YAML = os.path.join(_DOCS, 'slam', 'config', 'scale.yaml')


# cv_bridge's compiled boost extension is ABI-broken against numpy 1.24 on this
# Jetson, so we convert sensor_msgs/Image <-> numpy by hand (only two encodings).
def imgmsg_to_bgr(msg):
    """sensor_msgs/Image (rgb8|bgr8) -> HxWx3 uint8 BGR (matches cv2.imread)."""
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        arr = arr[:, :, ::-1]            # RGB -> BGR
    return np.ascontiguousarray(arr)


def depth_to_imgmsg(depth, header):
    """HxW float32 metres -> sensor_msgs/Image 32FC1, header copied verbatim."""
    depth = np.ascontiguousarray(depth, dtype=np.float32)
    msg = Image()
    msg.header = header
    msg.height, msg.width = depth.shape
    msg.encoding = '32FC1'
    msg.is_bigendian = 0
    msg.step = msg.width * 4
    msg.data = depth.tobytes()
    return msg


class DepthNode(Node):
    def __init__(self):
        super().__init__('depth_anything_node')
        self.declare_parameter('encoder', 'vits')
        self.declare_parameter('input_size', 518)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('rgb_topic', '/rgb/image_raw')
        self.declare_parameter('depth_topic', '/depth/image_raw')
        self.declare_parameter('throttle_hz', 2.0)   # publish-rate cap (GPU budget)

        enc = self.get_parameter('encoder').value
        self.input_size = self.get_parameter('input_size').value
        self.device = pick_device(self.get_parameter('device').value)
        self._last_pub = self.get_clock().now()
        self._min_period = 1.0 / max(0.1, self.get_parameter('throttle_hz').value)

        # Load scale (mapping vs calibration mode)
        self.k = self.d_max = self.near_clip = None
        if os.path.isfile(_SCALE_YAML):
            with open(_SCALE_YAML) as fh:
                s = yaml.safe_load(fh)
            self.k = float(s['k'])
            self.d_max = float(s.get('d_max', 5.0))
            self.near_clip = float(s.get('near_clip_m', 0.26))
            self.get_logger().info(
                f"MAPPING mode: k={self.k:.4f}, d_max={self.d_max}, "
                f"near_clip={self.near_clip} m")
        else:
            self.get_logger().warn(
                f"{_SCALE_YAML} absent -> CALIBRATION mode (raw relative depth). "
                f"Run slam/calibrate_scale.py to lock metric scale before mapping.")

        # Load DA-V2
        ckpt = os.path.join(_MODEL_DIR, f'depth_anything_v2_{enc}.pth')
        if not os.path.isfile(ckpt):
            raise FileNotFoundError(f"DA-V2 checkpoint not found: {ckpt}")
        self.model = DepthAnythingV2(**CFGS[enc])
        self.model.load_state_dict(torch.load(ckpt, map_location='cpu'))
        self.model = self.model.to(self.device).eval()
        self.get_logger().info(f"DA-V2 '{enc}' loaded on {self.device}")

        depth_topic = self.get_parameter('depth_topic').value
        self.pub = self.create_publisher(Image, depth_topic, 10)
        self.create_subscription(
            Image, self.get_parameter('rgb_topic').value, self.on_rgb, 10)

    def on_rgb(self, msg):
        # Throttle to protect the GPU (RTAB-Map shares it)
        now = self.get_clock().now()
        if (now - self._last_pub).nanoseconds * 1e-9 < self._min_period:
            return
        self._last_pub = now

        bgr = imgmsg_to_bgr(msg)
        with torch.inference_mode():
            tensor, hw = step_prep(self.model, bgr, self.input_size)
            rel = step_infer(self.model, tensor, hw).astype(np.float32)

        if self.k is not None:
            depth = apply_scale(rel, self.k, self.d_max)
            depth[depth < self.near_clip] = 0.0   # Filter 1: near-clip floor band
        else:
            depth = rel  # calibration mode: raw relative inverse depth

        out = depth_to_imgmsg(depth, msg.header)   # header copied verbatim (A7)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
