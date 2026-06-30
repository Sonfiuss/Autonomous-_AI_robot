#!/usr/bin/env python3
"""serial_bridge — sole owner of /dev/ttyUSB0 during SLAM (plan step A5).

Reader thread:  ESP32 'O x y theta_deg' @10Hz -> nav_msgs/Odometry on /odom
                + TF odom->base_link (theta deg -> rad -> quaternion).
Writer:         /cmd_teleop (std_msgs/String) -> ESP32 'M/F/T/V/S' line, lock-guarded.

Sole-ownership rule (interfaces.md): motivation / stereo-camera / deepmap must NOT
run while this node holds the port. The node asserts exclusivity and exits if busy.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import serial
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def yaw_to_quat(yaw):
    """Z-axis yaw (rad) -> (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # theta drifts most (open-loop rotation) -> larger yaw covariance
        self.declare_parameter('cov_xy', 0.05)
        self.declare_parameter('cov_yaw', 0.20)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        try:
            # exclusive=True -> fail fast if motivation/deepmap already hold the port
            self.ser = serial.Serial(port, baud, timeout=1.0, exclusive=True)
        except (serial.SerialException, OSError) as e:
            self.get_logger().fatal(
                f"Cannot open {port} exclusively ({e}). Is motivation/deepmap "
                f"running? Only ONE owner of {port} is allowed.")
            raise SystemExit(1)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_bc = TransformBroadcaster(self)
        self.create_subscription(String, '/cmd_teleop', self.on_teleop,
                                 QoSProfile(depth=10))

        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self.read_loop, daemon=True)
        self._reader.start()
        self.get_logger().info(f"serial_bridge owns {port} @ {baud} "
                               f"(odom->/odom, /cmd_teleop->ESP32)")

    # ---- writer ----------------------------------------------------------
    # ESP32 commands. The working drive path is per-motor 'C'/'W' (the high-level
    # 'F'/'T'/'M' paths do not move the steppers). Compute 'W <idx> <steps> <hz>'
    # angles with core_control + stepper_ctrl, or via slam/robot_drive.py, when a
    # human-friendly "move forward 30" interface is needed.
    _ALLOWED = set('CWSRVMFT')

    def on_teleop(self, msg):
        line = msg.data.strip()
        if not line:
            return
        if line[0] not in self._ALLOWED:
            self.get_logger().warn(f"ignoring non-protocol teleop: {line!r}")
            return
        with self._write_lock:
            self.ser.write((line + '\n').encode('ascii'))

    # ---- reader ----------------------------------------------------------
    def read_loop(self):
        cov_xy = self.get_parameter('cov_xy').value
        cov_yaw = self.get_parameter('cov_yaw').value
        while not self._stop.is_set() and rclpy.ok():
            try:
                raw = self.ser.readline().decode('ascii', 'ignore').strip()
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"serial read error: {e}")
                break
            if not raw or raw[0] != 'O':
                continue  # ignore P/K/READY here; odom only
            parts = raw.split()
            if len(parts) != 4:
                continue
            try:
                x, y, theta_deg = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                continue
            self.publish_odom(x, y, math.radians(theta_deg), cov_xy, cov_yaw)

    def publish_odom(self, x, y, yaw, cov_xy, cov_yaw):
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # 6x6 row-major: diag = [x, y, z, roll, pitch, yaw]
        c = [0.0] * 36
        c[0] = cov_xy; c[7] = cov_xy; c[14] = 1e6
        c[21] = 1e6;   c[28] = 1e6;   c[35] = cov_yaw
        odom.pose.covariance = c
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_bc.sendTransform(tf)

    def destroy_node(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
