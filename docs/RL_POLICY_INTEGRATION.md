# Z.O.E. Hexapod — Isaac Lab RL Locomotion Policy Integration Guide

This document defines the deployment contract and migration checklist for replacing the open-loop tripod IK controller (`hxp_command.py`) with the 50 Hz learned locomotion policy trained in [`Rithwik-Ravi/Z.O.E.-Hexapod-Isaac-Lab-Reinforced-Learning`](https://github.com/Rithwik-Ravi/Z.O.E.-Hexapod-Isaac-Lab-Reinforced-Learning).

---

## 1. Policy Interface Contract

| Signal | ROS 2 Topic / Node | Spec |
|---|---|---|
| Velocity Command | `/cmd_vel` (`geometry_msgs/msg/Twist`) | $v_x \in [-0.25, 0.30]\text{ m/s}$, $v_y \in [-0.20, 0.20]\text{ m/s}$, $\omega_z \in [-0.60, 0.60]\text{ rad/s}$ (body frame) |
| Proprioception (50 Hz) | `/joint_states` (`sensor_msgs/msg/JointState`) | 18 joint positions (rad) + 18 joint velocities (rad/s), leg-grouped `NCCUP{1..6}_joint`, `NTHIGH{1..6}_joint`, `NFOOT{1..6}_joint` |
| Inertial (50 Hz) | `/imu/data` (`sensor_msgs/msg/Imu`) | Gyro angular velocity $\boldsymbol{\omega}_{xyz}$ (rad/s) + gravity direction $\mathbf{g}_{\text{proj}}$ (unit vector in body frame) |
| Action Target (50 Hz) | `/hexapod_controller/joint_trajectory` (`JointTrajectory`) | $q_{\text{target}} = q_{\text{default}} + 0.50 \cdot a_t$, slew-rate clamped to $6.0^\circ/\text{step}$ ($5.236\text{ rad/s}$), $q_{\text{default}} = [0, 0, -\pi/2] \times 6$ |

Implemented in [`src/rover/rover/rl_policy_bridge.py`](../src/rover/rover/rl_policy_bridge.py) (entry point `ros2 run rover rl_policy_bridge`).

---

## 2. Nodes to Disable & Topic Collisions

| Component | Action Required | Reason |
|---|---|---|
| `hxp_cmd` (`hxp_command.py`) | **Disable** in `real_navigation.launch.py` & `sim_navigation.launch.py` | Publishes conflicting 8 Hz IK targets to `/hexapod_controller/joint_trajectory` and fake `/joint_states`. |
| `esp_bdg` (`esp.py`) | **Replace** with bidirectional 50 Hz ST3215 / ESP32 driver | `esp.py` is write-only ASCII @ 115200 baud (zero position/velocity readback) and applies `hxp_command`-specific `j1_neutral` offsets (`34.51°–55.49°`) and Leg 3 inversions. |
| `gz DiffDrive` (`hexa.urdf`) | **Remove** wheeled base & `DiffDrive` plugin for legged sim | `/cmd_vel` currently drives hidden wheels under `base_link`; replace with `gz_ros2_control` `JointGroupPositionController` + `JointStateBroadcaster` at ≥50 Hz. |
| `tool_dispatcher_node` | **Clamp / Route via `cmd_vel_nav`** | `_tool_dance` (`wz = ±1.0`) and `_tool_drive_distance` bypass `velocity_smoother` and exceed policy `wz = ±0.60`. |

---

## 3. Required Nav2 Parameter Updates (`nav2_real_params.yaml` / `nav2_params.yaml`)

1. **Enable holonomic `DWBLocalPlanner`:**
   - `max_vel_x: 0.30`, `min_vel_x: -0.25` (or `0.0`), `max_speed_xy: 0.30`
   - `max_vel_y: 0.20`, `min_vel_y: -0.20`, `vy_samples: 5` (currently `1`), `acc_lim_y: 0.6`, `decel_lim_y: -0.6`
   - `max_vel_theta: 0.6`, `controller_frequency: 20.0` (up from `5.0`)
2. **Update `velocity_smoother`:**
   - `max_velocity: [0.30, 0.20, 0.6]`, `min_velocity: [-0.25, -0.20, -0.6]`
   - `max_accel: [0.8, 0.6, 1.2]`, `max_decel: [-0.8, -0.6, -1.2]`, `smoothing_frequency: 20.0`
3. **Update `amcl` (when map localization is used):**
   - `robot_model_type: "nav2_amcl::OmniMotionModel"`

---

## 4. Hardware Prerequisites Before Field Deployment

1. **Actuator Feedback:** Migrate from open-loop PWM servos to **Waveshare ST3215 12V 1 Mbaud serial bus servos** (or add encoder feedback) so `/joint_states` publishes measured `position` and `velocity` at 50 Hz.
2. **Onboard IMU:** Add a ROS 2 IMU driver publishing `/imu/data` at ≥50 Hz (hardware-calibrated axes aligned to `base_link`).
3. **Joint Axis & Zero-Pose Verification:** Confirm `hexa.urdf` per-leg joint axes (`NTHIGH*`, `NFOOT*`) match the Isaac Lab USD (`hexapod/hexa/hexa.usd`) with robot suspended in a test rig before setting `dry_run:=false`.

