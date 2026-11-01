#!/usr/bin/env python3
"""
hexapod_ik_controller.py — Hexapod Inverse Kinematics Gait Controller (ros2_control version).

Computes 18-DOF inverse kinematics for a 6-legged hexapod and publishes
JointTrajectory messages to /hexapod_controller/joint_trajectory.
"""

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HexapodIKController(Node):

    def __init__(self):
        super().__init__('ik_controller')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/hexapod_controller/joint_trajectory',
            10
        )

        # -------- Timing --------
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.update)

        self.step_time = 1.0
        self.step_timer = 0.0
        self.step_index = 0
        self.num_steps = 4

        self.get_logger().info("IK Controller (ros2_control version) started")

        # -------- Joint order --------
        self.joint_names = [
            "NCCUP1_joint", "NTHIGH1_joint", "NFOOT1_joint",
            "NCCUP2_joint", "NTHIGH2_joint", "NFOOT2_joint",
            "NCCUP3_joint", "NTHIGH3_joint", "NFOOT3_joint",
            "NCCUP4_joint", "NTHIGH4_joint", "NFOOT4_joint",
            "NCCUP5_joint", "NTHIGH5_joint", "NFOOT5_joint",
            "NCCUP6_joint", "NTHIGH6_joint", "NFOOT6_joint"
        ]

        self.positions = [0.0] * 18

        # -------- Gait table (same as Arduino) --------
        self.gait = {
            5: [(80, 55, 150), (80, 80, 150), (80, 55, 120), (80, 30, 150)],
            4: [(77.8, 77.8, 150), (59.8, 95.8, 150), (77.8, 77.8, 120), (95.8, 59.8, 150)],
            3: [(55, 80, 150), (30, 80, 150), (55, 80, 120), (80, 80, 150)],
            2: [(80, 55, 150), (80, 30, 150), (80, 55, 120), (80, 80, 150)],
            1: [(77.8, 77.8, 150), (95.8, 59.8, 150), (77.8, 77.8, 120), (59.8, 95.8, 150)],
            6: [(55, 80, 150), (80, 80, 150), (55, 80, 120), (30, 80, 150)],
        }

        self.tripod_A = [1, 3, 5]
        self.tripod_B = [2, 4, 6]

    # -------- Inverse Kinematics --------
    def inverse_kinematics(self, x, y, z):
        J1L = 35.0
        J2L = 75.0
        J3L = 160.0

        J1 = math.atan2(x, y)

        H = math.sqrt(x * x + y * y)
        L = math.sqrt(z * z + (H - J1L) * (H - J1L))

        def clamp(v):
            return max(-1.0, min(1.0, v))

        J3 = 1.57 - math.acos(clamp((J3L * J3L + J2L * J2L - L * L) / (2 * J2L * J3L)))
        B = math.acos(clamp((L * L + J2L * J2L - J3L * J3L) / (2 * L * J2L)))
        A = math.atan2(z, H - J1L)
        J2 = (B - A)

        return J1, J2, J3

    # -------- Gait update --------
    def update_gait(self):
        self.step_timer += self.dt
        if self.step_timer >= self.step_time:
            self.step_timer = 0.0
            self.step_index = (self.step_index + 1) % self.num_steps

    # -------- Main loop --------
    def update(self):
        self.update_gait()

        for leg_id in range(1, 7):
            if leg_id in self.tripod_A:
                idx = self.step_index
            else:
                idx = (self.step_index + self.num_steps // 2) % self.num_steps

            x_mm, y_mm, z_mm = self.gait[leg_id][idx]
            q1, q2, q3 = self.inverse_kinematics(x_mm, y_mm, z_mm)

            base = (leg_id - 1) * 3
            self.positions[base] = q1
            self.positions[base + 1] = q2
            self.positions[base + 2] = q3

        self.publish_trajectory()

    # -------- Publish command --------
    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start.sec = 1

        msg.points.append(point)
        self.publisher.publish(msg)


# Alias for backward compatibility
IKController = HexapodIKController


def main(args=None):
    rclpy.init(args=args)
    node = HexapodIKController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
