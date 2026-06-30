#!/usr/bin/env python3
"""cam_publisher — minimal OpenCV camera node (plan A3, replaces usb_cam).

The ros-foxy-usb-cam build on this Jetson SIGABRTs ("terminate ... 'char*'") on
stream start for both MJPEG and YUYV. cv2.VideoCapture is proven to work on this
exact camera, so we publish frames ourselves:

  /rgb/image_raw   sensor_msgs/Image  (bgr8)
  /rgb/camera_info sensor_msgs/CameraInfo  (from slam/config/camera.yaml)

Both share one timestamp so RTAB-Map can pair them. No cv_bridge (ABI-broken vs
numpy 1.24) — Image is packed by hand, same as depth_anything_node.
"""
import os

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

_CAM_YAML = os.path.expanduser('~/Documents/slam/config/camera.yaml')


def load_camera_info(path, frame_id):
    info = CameraInfo()
    if not os.path.isfile(path):
        return info, 640, 480
    with open(path) as fh:
        c = yaml.safe_load(fh)
    info.width = int(c['image_width'])
    info.height = int(c['image_height'])
    info.distortion_model = c.get('distortion_model', 'plumb_bob')
    info.d = [float(x) for x in c['distortion_coefficients']['data']]
    info.k = [float(x) for x in c['camera_matrix']['data']]
    info.r = [float(x) for x in c['rectification_matrix']['data']]
    info.p = [float(x) for x in c['projection_matrix']['data']]
    info.header.frame_id = frame_id
    return info, info.width, info.height


class CamPublisher(Node):
    def __init__(self):
        super().__init__('cam_publisher')
        self.declare_parameter('device', 0)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        dev = int(self.get_parameter('device').value)   # launch passes str '0'
        fps = float(self.get_parameter('fps').value)
        frame_id = self.get_parameter('frame_id').value

        self.info, W, H = load_camera_info(_CAM_YAML, frame_id)
        self.frame_id = frame_id

        # Force the V4L2 backend (the default GStreamer path is slow here and
        # spams pipeline warnings).
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().fatal(
                f"cannot open camera index {dev} — is another node holding "
                f"/dev/video{dev}? (pkill -9 -f usb_cam_node_exe)")
            raise SystemExit(1)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # Force capture resolution to match the intrinsics in camera.yaml
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        aw = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (aw, ah) != (W, H):
            self.get_logger().warn(
                f"requested {W}x{H} but camera gave {aw}x{ah}; "
                f"regenerate camera.yaml: python slam/make_intrinsics.py "
                f"--hfov 60 --width {aw} --height {ah}")

        self.img_pub = self.create_publisher(Image, '/rgb/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/rgb/camera_info', 10)
        self.create_timer(1.0 / max(1.0, fps), self.tick)
        self.get_logger().info(f"cam_publisher: /dev/video{dev} {aw}x{ah} @ {fps} Hz")

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("camera read failed")
            return
        stamp = self.get_clock().now().to_msg()

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        self.img_pub.publish(msg)

        self.info.header.stamp = stamp
        self.info.header.frame_id = self.frame_id
        self.info_pub.publish(self.info)

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
