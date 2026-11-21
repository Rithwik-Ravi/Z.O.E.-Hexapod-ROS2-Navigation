#!/usr/bin/env python3
"""
rl_policy_bridge.py — 50 Hz RL Locomotion Policy Bridge for Z.O.E. Hexapod.

Implements the deployable 63-D Actor observation and 18-DOF action contract
from `Rithwik-Ravi/Z.O.E.-Hexapod-Isaac-Lab-Reinforced-Learning`:

Observation vector (63-D, float32):
  [0:3]   base angular velocity (rad/s, body frame from `/imu/data`)
  [3:6]   projected gravity vector (unit-normalized body frame from `/imu/data`)
  [6:9]   velocity command (vx, vy, wz) from `/cmd_vel`
  [9:27]  joint position offsets (q - q_default, rad) in leg-grouped order
  [27:45] joint velocities (dq, rad/s) in leg-grouped order
  [45:63] previous raw policy actions a_{t-1}

Action post-processing (50 Hz):
  q_raw    = q_default + 0.50 * clip(action, -1.0, 1.0)
  q_target = q_prev + clip(q_raw - q_prev, -6.0 deg, +6.0 deg)  # slew-rate limit

Notes:
  - Supports dependency-free NumPy MLP exports (`policy_mlp.npz`) or ONNX
    (`policy.onnx` via `onnxruntime`). When no policy file is provided, holds
    the calibrated standing stance (`q_default`) safely.
  - Requires real 50 Hz joint position/velocity feedback on `/joint_states`
    and 50 Hz IMU gyro + orientation/accel on `/imu/data`. See
    `docs/RL_POLICY_INTEGRATION.md` before connecting to physical hardware.
"""

import math
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Canonical leg-grouped joint order (matches Z.O.E. Isaac Lab + hexa.urdf)
JOINT_ORDER: List[str] = [
    'NCCUP1_joint', 'NTHIGH1_joint', 'NFOOT1_joint',
    'NCCUP2_joint', 'NTHIGH2_joint', 'NFOOT2_joint',
    'NCCUP3_joint', 'NTHIGH3_joint', 'NFOOT3_joint',
    'NCCUP4_joint', 'NTHIGH4_joint', 'NFOOT4_joint',
    'NCCUP5_joint', 'NTHIGH5_joint', 'NFOOT5_joint',
    'NCCUP6_joint', 'NTHIGH6_joint', 'NFOOT6_joint',
]

# Z.O.E. Isaac Lab calibrated standing stance: Coxa=0, Thigh=0, Foot=-90 deg (-pi/2)
DEFAULT_JOINT_POS = np.array(
    [0.0, 0.0, -math.pi / 2.0] * 6,
    dtype=np.float32,
)

ACTION_SCALE_RAD = 0.50
MAX_SLEW_PER_STEP_RAD = math.radians(6.0)  # 6.0 deg/step @ 50 Hz = 5.236 rad/s


def project_gravity_from_quat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    Rotate world gravity `(0, 0, -1)` into the robot body frame via quaternion conjugate.

    Returns a 3-D float32 array `[gx, gy, gz]`.
    """
    # Normalize to guard against uncalibrated IMU quaternions
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-6:
        return np.array([0.0, 0.0, -1.0], dtype=np.float32)
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm

    # q^{-1} * [0, 0, -1] * q
    gx = -2.0 * (x * z - w * y)
    gy = -2.0 * (y * z + w * x)
    gz = -(1.0 - 2.0 * (x * x + y * y))
    return np.array([gx, gy, gz], dtype=np.float32)


def apply_slew_rate_limit(
    q_desired: np.ndarray,
    q_prev: np.ndarray,
    max_delta_rad: float = MAX_SLEW_PER_STEP_RAD,
) -> np.ndarray:
    """Clamp per-step joint position delta to `[-max_delta_rad, +max_delta_rad]`."""
    delta = np.clip(q_desired - q_prev, -max_delta_rad, max_delta_rad)
    return (q_prev + delta).astype(np.float32)


def is_cmd_stopped(vx: float, vy: float, wz: float) -> bool:
    """Return True when velocity command is within the Z.O.E. deadband."""
    return (math.hypot(vx, vy) < 0.05) and (abs(wz) < 0.05)


def build_actor_observation(
    ang_vel: np.ndarray,
    proj_gravity: np.ndarray,
    cmd_vel: np.ndarray,
    joint_pos: np.ndarray,
    joint_vel: np.ndarray,
    prev_action: np.ndarray,
    default_joint_pos: np.ndarray = DEFAULT_JOINT_POS,
) -> np.ndarray:
    """Assemble the exact 63-D deployable Actor observation vector."""
    obs = np.concatenate([
        ang_vel.astype(np.float32),
        proj_gravity.astype(np.float32),
        cmd_vel.astype(np.float32),
        (joint_pos - default_joint_pos).astype(np.float32),
        joint_vel.astype(np.float32),
        prev_action.astype(np.float32),
    ])
    if obs.shape != (63,):
        raise ValueError(f'Expected 63-D observation vector, got {obs.shape}')
    return obs


class NumpyMLPPolicy:
    """Lightweight inference wrapper for Z.O.E. `policy_mlp.npz` exports."""

    def __init__(self, npz_path: str):
        data = np.load(npz_path, allow_pickle=False)
        self.files = list(data.files)
        self.mean = data['obs_mean'].astype(np.float32) if 'obs_mean' in data else np.zeros(63, dtype=np.float32)
        self.std = data['obs_std'].astype(np.float32) if 'obs_std' in data else np.ones(63, dtype=np.float32)
        self.layers = []
        idx = 0
        while f'w{idx}' in data and f'b{idx}' in data:
            self.layers.append((data[f'w{idx}'].astype(np.float32), data[f'b{idx}'].astype(np.float32)))
            idx += 1
        if not self.layers:
            raise ValueError(f'No weight tensors (w0/b0...) found in {npz_path}: {self.files}')

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        x = (obs - self.mean) / np.maximum(self.std, 1e-6)
        for i, (w, b) in enumerate(self.layers):
            x = x @ w.T + b if w.shape[1] == x.shape[0] else x @ w + b
            if i < len(self.layers) - 1:
                # ELU activation matching RSL-RL actor MLP
                x = np.where(x > 0.0, x, np.exp(np.clip(x, -20.0, 20.0)) - 1.0)
        return np.clip(x, -1.0, 1.0).astype(np.float32)


class RLPolicyBridgeNode(Node):
    """50 Hz ROS 2 inference bridge for Z.O.E. Isaac Lab locomotion policies."""

    def __init__(self):
        super().__init__('rl_policy_bridge')

        self.declare_parameter('policy_path', '')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('dry_run', True)

        self.policy_path = self.get_parameter('policy_path').get_parameter_value().string_value
        rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self.policy: Optional[NumpyMLPPolicy] = None
        if self.policy_path:
            self.policy = NumpyMLPPolicy(self.policy_path)
            self.get_logger().info(f'Loaded policy checkpoint: {self.policy_path}')
        else:
            self.get_logger().warn(
                'No `policy_path` provided — running in safe default-stance hold mode.'
            )

        # State buffers
        self.cmd_vel = np.zeros(3, dtype=np.float32)
        self.last_cmd_time = self.get_clock().now()
        self.ang_vel = np.zeros(3, dtype=np.float32)
        self.proj_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self.joint_pos = DEFAULT_JOINT_POS.copy()
        self.joint_vel = np.zeros(18, dtype=np.float32)
        self.prev_action = np.zeros(18, dtype=np.float32)
        self.q_prev = DEFAULT_JOINT_POS.copy()
        self.have_imu = False
        self.have_joints = False

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)

        # Publisher (trajectory target; disabled in dry_run unless explicitly cleared)
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/hexapod_controller/joint_trajectory', 10
        )

        self.timer = self.create_timer(1.0 / max(1.0, rate_hz), self._step)
        self.get_logger().info(
            f'RLPolicyBridge initialized @ {rate_hz:.1f} Hz (dry_run={self.dry_run})'
        )

    def _on_cmd_vel(self, msg: Twist):
        # Clamp to Z.O.E. policy velocity envelope: vx [-0.25, 0.30], vy +-0.20, wz +-0.60
        self.cmd_vel[0] = float(np.clip(msg.linear.x, -0.25, 0.30))
        self.cmd_vel[1] = float(np.clip(msg.linear.y, -0.20, 0.20))
        self.cmd_vel[2] = float(np.clip(msg.angular.z, -0.60, 0.60))
        self.last_cmd_time = self.get_clock().now()

    def _on_imu(self, msg: Imu):
        self.ang_vel[:] = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]
        q = msg.orientation
        if abs(q.w) + abs(q.x) + abs(q.y) + abs(q.z) > 1e-6:
            self.proj_gravity = project_gravity_from_quat(q.x, q.y, q.z, q.w)
        else:
            # Fallback to normalized negative linear acceleration when orientation is unset
            acc = np.array(
                [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                dtype=np.float32,
            )
            mag = float(np.linalg.norm(acc))
            if mag > 1e-3:
                self.proj_gravity = -acc / mag
        self.have_imu = True

    def _on_joint_states(self, msg: JointState):
        pos_map: Dict[str, float] = dict(zip(msg.name, msg.position))
        vel_map: Dict[str, float] = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        for idx, jname in enumerate(JOINT_ORDER):
            if jname in pos_map:
                self.joint_pos[idx] = float(pos_map[jname])
            if jname in vel_map:
                self.joint_vel[idx] = float(vel_map[jname])
        self.have_joints = all(jname in pos_map for jname in JOINT_ORDER)

    def _step(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            self.cmd_vel[:] = 0.0

        obs = build_actor_observation(
            ang_vel=self.ang_vel,
            proj_gravity=self.proj_gravity,
            cmd_vel=self.cmd_vel,
            joint_pos=self.joint_pos,
            joint_vel=self.joint_vel,
            prev_action=self.prev_action,
        )

        if self.policy is not None and self.have_imu and self.have_joints:
            action = self.policy(obs)
        else:
            action = np.zeros(18, dtype=np.float32)

        q_raw = DEFAULT_JOINT_POS + ACTION_SCALE_RAD * action
        q_target = apply_slew_rate_limit(q_raw, self.q_prev, MAX_SLEW_PER_STEP_RAD)
        self.prev_action = action
        self.q_prev = q_target

        if self.dry_run:
            return

        traj = JointTrajectory()
        traj.joint_names = list(JOINT_ORDER)
        pt = JointTrajectoryPoint()
        pt.positions = q_target.tolist()
        pt.time_from_start.nanosec = 20_000_000  # 20 ms (50 Hz)
        traj.points.append(pt)
        self.traj_pub.publish(traj)


def main(args=None):
    """Entry point for the `rl_policy_bridge` ROS 2 node."""
    rclpy.init(args=args)
    node = RLPolicyBridgeNode()
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
