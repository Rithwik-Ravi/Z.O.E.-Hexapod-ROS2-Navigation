#!/usr/bin/env python3
"""
odom_from_tf.py — Dynamic TF-derived Odometry Publisher for Legged Hardware.

When wheel encoders are unavailable on the physical hexapod, SLAM Toolbox
publishes `map -> base_footprint` via 2D LiDAR scan matching while a static
identity transform links `odom -> base_footprint`. This node samples
`map -> base_footprint` (falling back to `odom -> base_footprint`), computes
pose derivatives, rotates linear velocity into the `base_footprint` body frame
(per ROS REP-103 `nav_msgs/Odometry` twist semantics), and publishes `/odom`.
"""

import math
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import tf2_ros


def yaw_from_quaternion(q) -> float:
    """Extract planar yaw angle (radians) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_vel_to_body(dx_dt: float, dy_dt: float, yaw: float):
    """
    Rotate world-frame linear velocity `(dx/dt, dy/dt)` into the robot body frame.

    Args:
        dx_dt: World-frame X velocity (m/s).
        dy_dt: World-frame Y velocity (m/s).
        yaw: Robot heading in the parent frame (radians).

    Returns:
        Tuple `(vx_body, vy_body)` in the `base_footprint` frame (m/s).
    """
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    vx_body = dx_dt * cos_y + dy_dt * sin_y
    vy_body = -dx_dt * sin_y + dy_dt * cos_y
    return vx_body, vy_body


def normalize_angle(angle: float) -> float:
    """Wrap an angle in radians to `[-pi, pi]`."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class OdomFromTF(Node):
    """Publishes `nav_msgs/Odometry` on `/odom` by differentiating TF poses."""

    def __init__(self):
        super().__init__('odom_from_tf')

        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('rate', 20.0)

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        rate = self.get_parameter('rate').get_parameter_value().double_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self.publish_odom)

        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.last_time = None

        self.get_logger().info(
            f"odom_from_tf started: base_frame='{self.base_frame}', odom_frame='{self.odom_frame}'"
        )

    def publish_odom(self):
        """Sample latest TF transform and publish Odometry with body-frame twist."""
        tf_msg: TransformStamped = None
        for parent_frame in ['map', self.odom_frame]:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    parent_frame,
                    self.base_frame,
                    rclpy.time.Time()
                )
                break
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

        if tf_msg is None:
            return

        current_time = self.get_clock().now()
        stamp_sec = tf_msg.header.stamp.sec + tf_msg.header.stamp.nanosec * 1e-9

        x = tf_msg.transform.translation.x
        y = tf_msg.transform.translation.y
        z = tf_msg.transform.translation.z
        q = tf_msg.transform.rotation
        yaw = yaw_from_quaternion(q)

        vx = 0.0
        vy = 0.0
        wz = 0.0

        if self.last_time is not None:
            dt = stamp_sec - self.last_time
            if 0.001 < dt < 1.0:
                dx_dt = (x - self.last_x) / dt
                dy_dt = (y - self.last_y) / dt
                dyaw = normalize_angle(yaw - self.last_yaw)

                # Rotate world velocity into child_frame_id (base_footprint)
                vx, vy = world_vel_to_body(dx_dt, dy_dt, yaw)
                wz = dyaw / dt

        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw
        self.last_time = stamp_sec

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Reflects LiDAR-SLAM uncertainty without wheel encoders (~10 cm, 0.05 rad)
        pose_cov = [
            0.10, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.10, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        twist_cov = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        self.pub.publish(odom)


def main(args=None):
    """Entry point for the `odom_from_tf` ROS 2 node."""
    rclpy.init(args=args)
    node = OdomFromTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
