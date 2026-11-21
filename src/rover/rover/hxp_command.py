#!/usr/bin/env python3
"""
hxp_command.py — Tripod-Gait Inverse Kinematics Controller for 18-DOF Hexapod.

Subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`, e.g., from Nav2 or teleop),
maps velocity commands into discrete alternating-tripod walking gaits (`f/b/l/r/s`),
solves 3-DOF analytical leg inverse kinematics (coxa, femur, tibia), and publishes:
  - `/hexapod_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`, 8 Hz)
  - `/joint_states` (`sensor_msgs/msg/JointState`, 8 Hz) for `robot_state_publisher` & RViz2

Also supports optional interactive keyboard input when attached to a TTY.
"""

import math
import signal
import sys
import threading

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Leg kinematic link lengths in millimeters (Coxa J1L, Femur/Thigh J2L, Tibia/Foot J3L)
COXA_LENGTH_MM = 35.0
FEMUR_LENGTH_MM = 75.0
TIBIA_LENGTH_MM = 160.0

# Canonical leg-grouped joint order matching URDF (`hexa.urdf`) and `esp.py`
HEXAPOD_JOINT_NAMES = [
    'NCCUP1_joint', 'NTHIGH1_joint', 'NFOOT1_joint',
    'NCCUP2_joint', 'NTHIGH2_joint', 'NFOOT2_joint',
    'NCCUP3_joint', 'NTHIGH3_joint', 'NFOOT3_joint',
    'NCCUP4_joint', 'NTHIGH4_joint', 'NFOOT4_joint',
    'NCCUP5_joint', 'NTHIGH5_joint', 'NFOOT5_joint',
    'NCCUP6_joint', 'NTHIGH6_joint', 'NFOOT6_joint',
]


def solve_leg_ik(x_mm: float, y_mm: float, z_mm: float):
    """
    Compute 3-DOF analytical inverse kinematics for one hexapod leg.

    Args:
        x_mm: Foot target X offset from coxa pivot (mm).
        y_mm: Foot target Y offset from coxa pivot (mm).
        z_mm: Foot downward vertical reach from coxa pivot (mm, positive down).

    Returns:
        Tuple `(j1_coxa, j2_femur, j3_tibia)` joint angles in radians.
    """
    j1 = math.atan2(x_mm, y_mm)

    horizontal_reach = math.hypot(x_mm, y_mm)
    eff_reach = horizontal_reach - COXA_LENGTH_MM
    leg_diag = math.hypot(z_mm, eff_reach)

    def clamp_unit(val: float) -> float:
        return max(-1.0, min(1.0, val))

    # Guard against zero-division if target collapses onto coxa pivot
    denom_j3 = max(1e-6, 2.0 * FEMUR_LENGTH_MM * TIBIA_LENGTH_MM)
    cos_j3 = clamp_unit((TIBIA_LENGTH_MM ** 2 + FEMUR_LENGTH_MM ** 2 - leg_diag ** 2) / denom_j3)
    j3 = 1.57 - math.acos(cos_j3)

    denom_b = max(1e-6, 2.0 * leg_diag * FEMUR_LENGTH_MM)
    cos_b = clamp_unit((leg_diag ** 2 + FEMUR_LENGTH_MM ** 2 - TIBIA_LENGTH_MM ** 2) / denom_b)
    b_angle = math.acos(cos_b)
    a_angle = math.atan2(z_mm, eff_reach)

    j2 = b_angle - a_angle
    return j1, j2, j3


class IKController(Node):
    """ROS 2 node converting `/cmd_vel` commands into 18-DOF tripod gait IK."""

    def __init__(self):
        super().__init__('ik_controller')

        self.running = True

        # ── Publishers ────────────────────────────────────────────────────────
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/hexapod_controller/joint_trajectory',
            10,
        )
        self.js_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10,
        )

        # ── Subscriber: /cmd_vel from Nav2 velocity_smoother / teleop ─────────
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )
        self.last_cmd_time = None
        self.cmd_timeout = 0.6  # Seconds without cmd_vel before reverting to stance

        # ── Control loop timing (8 Hz update, 0.5 s per gait waypoint) ────────
        self.dt = 0.125
        self.timer = self.create_timer(self.dt, self.update)

        self.step_time = 0.5
        self.step_timer = 0.0
        self.step_index = 0
        self.num_steps = 4

        # Default 's' stands still until cmd_vel or keyboard command is received
        self.command = 's'

        self.joint_names = list(HEXAPOD_JOINT_NAMES)
        self.positions = [0.0] * 18

        # ── Foot-tip waypoint tables in leg-local coordinates (mm) ────────────
        self.forward_gait = {
            1: [(77.8, 77.8, 150), (95.8, 59.8, 150), (77.8, 77.8, 100), (59.8, 95.8, 150)],
            2: [(55, 80, 150), (80, 80, 150), (55, 80, 100), (30, 80, 150)],
            3: [(80, 55, 150), (80, 80, 150), (80, 55, 100), (80, 30, 150)],
            4: [(77.8, 77.8, 150), (59.8, 95.8, 150), (77.8, 77.8, 100), (95.8, 59.8, 150)],
            5: [(55, 80, 150), (30, 80, 150), (55, 80, 100), (80, 80, 150)],
            6: [(80, 55, 150), (80, 30, 150), (80, 55, 100), (80, 80, 150)],
        }

        self.right_gait = {
            leg: [(77.8, 77.8, 150), (59.3, 95.0, 150), (77.8, 77.8, 100), (95.0, 59.3, 150)]
            for leg in range(1, 7)
        }

        self.initial_pose = {
            1: (77.8, 77.8, 150),
            2: (55.0, 80.0, 150),
            3: (80.0, 55.0, 150),
            4: (77.8, 77.8, 150),
            5: (55.0, 80.0, 150),
            6: (80.0, 55.0, 150),
        }

        # Alternating tripod support/swing groups (180 deg phase offset)
        self.tripod_A = [1, 3, 5]
        self.tripod_B = [2, 4, 6]

        threading.Thread(target=self.keyboard_input, daemon=True).start()
        self.get_logger().info('Hexapod IK Controller started (listening on /cmd_vel)')

    def cmd_vel_callback(self, msg: Twist):
        """Discretize continuous `/cmd_vel` Twist into `f/b/l/r/s` gait states."""
        self.last_cmd_time = self.get_clock().now()
        vx = msg.linear.x
        wz = msg.angular.z

        lin_threshold = 0.03
        ang_threshold = 0.08
        prev_cmd = self.command

        # Prioritize turning when angular velocity dominates linear velocity
        if abs(wz) >= ang_threshold and (abs(vx) < lin_threshold or abs(wz) > 1.2 * abs(vx)):
            self.command = 'l' if wz > 0.0 else 'r'
        elif abs(vx) >= lin_threshold:
            self.command = 'f' if vx > 0.0 else 'b'
        elif abs(wz) >= ang_threshold:
            self.command = 'l' if wz > 0.0 else 'r'
        else:
            self.command = 's'

        if self.command != prev_cmd:
            self.get_logger().info(
                f'Nav2 gait command: {prev_cmd} -> {self.command} (vx={vx:.2f}, wz={wz:.2f})'
            )

    def keyboard_input(self):
        """Read manual `f/b/l/r/s/q` commands when stdin is an interactive TTY."""
        if not sys.stdin or not sys.stdin.isatty():
            return

        while self.running:
            try:
                cmd = input('Enter command (f/b/l/r/s, q to quit): ').strip().lower()
                if cmd in ['f', 'b', 'l', 'r', 's']:
                    self.command = cmd
                    self.last_cmd_time = None
                elif cmd in ['q', 'quit']:
                    self.running = False
                    break
            except (EOFError, KeyboardInterrupt):
                break

    def get_gait(self):
        """Return the active 4-step foot-waypoint dictionary for the current command."""
        if self.command == 'f':
            return self.forward_gait
        if self.command == 'b':
            return {leg: list(reversed(pts)) for leg, pts in self.forward_gait.items()}
        if self.command == 'r':
            return self.right_gait
        if self.command == 'l':
            return {leg: list(reversed(pts)) for leg, pts in self.right_gait.items()}
        return self.forward_gait

    def inverse_kinematics(self, x: float, y: float, z: float):
        """Solve 3-DOF analytical leg IK (wrapper around `solve_leg_ik`)."""
        return solve_leg_ik(x, y, z)

    def update_gait(self):
        """Advance the 4-step tripod phase index at `step_time` intervals."""
        self.step_timer += self.dt
        if self.step_timer >= self.step_time:
            self.step_timer = 0.0
            self.step_index = (self.step_index + 1) % self.num_steps

    def update(self):
        """Periodic 8 Hz control tick: enforce watchdog, solve IK, publish targets."""
        if self.last_cmd_time is not None:
            elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if elapsed > self.cmd_timeout:
                self.command = 's'

        if self.command == 's':
            for leg_id in range(1, 7):
                x, y, z = self.initial_pose[leg_id]
                q1, q2, q3 = self.inverse_kinematics(x, y, z)
                base = (leg_id - 1) * 3
                self.positions[base] = q1
                self.positions[base + 1] = q2
                self.positions[base + 2] = q3
            self.publish_trajectory()
            self.publish_joint_states()
            return

        self.update_gait()
        gait = self.get_gait()

        for leg_id in range(1, 7):
            idx = (
                self.step_index
                if leg_id in self.tripod_A
                else (self.step_index + self.num_steps // 2) % self.num_steps
            )
            x, y, z = gait[leg_id][idx]
            q1, q2, q3 = self.inverse_kinematics(x, y, z)

            base = (leg_id - 1) * 3
            self.positions[base] = q1
            self.positions[base + 1] = q2
            self.positions[base + 2] = q3

        self.publish_trajectory()
        self.publish_joint_states()

    def publish_trajectory(self):
        """Publish 18-DOF target angles on `/hexapod_controller/joint_trajectory`."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = list(self.positions)
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.publisher.publish(msg)

    def publish_joint_states(self):
        """Publish `/joint_states` (18 hexapod joints + sim wheel dummies) for RSP/RViz."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names + ['wheel_front_right_joint', 'wheel_front_left_joint']
        msg.position = list(self.positions) + [0.0, 0.0]
        self.js_pub.publish(msg)

    def reset_to_initial_pose(self):
        """Send standing pose targets on shutdown so hardware settles safely."""
        self.get_logger().info('Setting robot to initial standing position...')
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()

        for leg_id in range(1, 7):
            x, y, z = self.initial_pose[leg_id]
            q1, q2, q3 = self.inverse_kinematics(x, y, z)
            base = (leg_id - 1) * 3
            self.positions[base] = q1
            self.positions[base + 1] = q2
            self.positions[base + 2] = q3

        if rclpy.ok():
            for _ in range(5):
                self.publish_trajectory()
                self.publish_joint_states()
                try:
                    rclpy.spin_once(self, timeout_sec=0.1)
                except Exception:
                    pass
        self.get_logger().info('Robot returned to initial standing position.')


def main(args=None):
    """Entry point for the `hxp_cmd` (`ik_controller`) ROS 2 node."""
    rclpy.init(args=args)
    node = IKController()

    def sig_handler(_sig, _frame):
        node.get_logger().info('Stop signal received, shutting down...')
        node.running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        while node.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.reset_to_initial_pose()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
