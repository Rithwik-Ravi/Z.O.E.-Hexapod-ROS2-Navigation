# ROS 2 Workspace Read-Only Inventory (`gz_hex`)

**Audit Date:** 2026-11-21  
**Target Workspace:** `/home/rithwik/Downloads/gz_hexa/gz_hex` (nested inside `/home/rithwik/Downloads/gz_hexa`)  
**Audit Method:** Read-only inspection of source, configs, URDFs, launch files, build logs, TF snapshots, and git history; non-hardware CLI checks (`colcon list`, `git log`, Python import checks). No nodes were launched, no serial ports were opened, and no hardware was touched.  
**Verification Legend:**
- **VERIFIED** — Directly observed in a read file (`path:line`) or command output in this session.
- **INFERRED** — Deduced from multiple verified facts, comments, or standard ROS 2 / Nav2 behavior (not directly executed/measured here).

---

## 1. Identity

### 1.1 Packages (`colcon list` inside `gz_hex/`)

| Package | Path | Build Type | Version | Maintainer / Author | License | Source |
|---|---|---|---|---|---|---|
| `rover` | `src/rover` | `ros.ament_python` | `0.0.0` | `aanimesh <aanimesh@todo.todo>` (`package.xml`) / `your@email.com` (`setup.py`) | `TODO: License declaration` (`package.xml:8`) vs `Apache-2.0` (`setup.py:28`) vs `Apache 2.0` badge (`README.md:6`) | **VERIFIED** (`colcon list`, `gz_hex/src/rover/package.xml:3-9`, `gz_hex/src/rover/setup.py:8-28`) |
| `rover_interfaces` | `src/rover_interfaces` | `ros.ament_cmake` | `0.0.0` | `vishal <vishal@todo.todo>` | `Apache-2.0` | **VERIFIED** (`colcon list`, `gz_hex/src/rover_interfaces/package.xml:3-8`) |
| `rplidar_ros` | `src/rplidar_ros` | `ros.ament_cmake` | `2.1.4` | Maintainer: `Wang DeYou <deyou.wang@slamtec.com>`; Author: `Slamtec ROS Maintainer <ros@slamtec.com>` | `BSD` (3-clause BSD in `src/rplidar_ros/LICENSE:1-24`) | **VERIFIED** (`colcon list`, `gz_hex/src/rplidar_ros/package.xml:3-12`) |

### 1.2 Git Remote, History, and Working Tree State
- **Git repository root:** `/home/rithwik/Downloads/gz_hexa/gz_hex/.git` (parent `/home/rithwik/Downloads/gz_hexa` is *not* a git repository) (**VERIFIED**: `git status` at root failed code 128; `find gz_hex -name '.git'` -> `gz_hex/.git`).
- **Remote (`git -C gz_hex remote -v`):** `origin  git@github.com:TeamArtemisPPG/Hexapod.git` (fetch & push) (**VERIFIED**).
- **Commit History (all 5 commits in repo; <20 exist):** (**VERIFIED**: `git -C gz_hex log -n 20`)
  1. `c5c0210` (2026-11-01 19:42:35 +0530, `HEAD -> main, origin/main`, `TeamArtemisPPG <teamartemis@vit.ac.in>`): `fix(jazzy): ensure full ROS 2 Jazzy compatibility for launch files and nodes` — migrated `2_gazebo.launch.py` to `ros_gz_sim`/`ros_gz_bridge`, added `teleop.py`, updated `hexapod_ik_controller.py`, updated `real_navigation.launch.py` static TF flags, updated `package.xml`/`setup.py`/`hexa.urdf`.
  2. `a08b048` (2026-10-26 08:38:00 +0530, `Ishan <170653513+IB-007@users.noreply.github.com>`): `Store STEP and MAPDL logs byte-for-byte (no EOL conversion)`.
  3. `61da5c3` (2026-10-26 08:37:22 +0530, `Ishan <170653513+IB-007@users.noreply.github.com>`): `Add hexapod CAD (STEP) and Ansys FEA results`.
  4. `840df2d` (2026-10-25 23:27:37 +0530, `TeamArtemisPPG <teamartemis@vit.ac.in>`): `Add Autonomous Hexapod ROS 2 stack with NVIDIA NIM AI brain, Nav2, SLAM and hardware bridges` (210 files, +30,109 lines).
  5. `46bf4a1` (2026-10-25 23:05:59 +0530, `TeamArtemisPPG <teamartemis@vit.ac.in>`): `Initial commit` (`README.md`).
- **Uncommitted Local Modifications (`git -C gz_hex status -s`):** (**VERIFIED**)
  - `M src/rover/rover/esp.py`: default `port` changed `"/dev/ttyUSB1"` -> `"/dev/ttyUSB0"`; added Leg 3 J2/J3 inversion (`servo_deg = 90.0 - angle_deg`); removed old commented-out `JointStateToESP` block.
  - `M src/rover/rover/hxp_command.py`: removed `if leg_id == 5: q2 = -q2; q3 = -q3` sign negation in `update()` and `reset_to_initial_pose()`.
  - `M src/rover/rover/nvidia_brain_node.py`: updated `SYSTEM_PROMPT` from Gazebo simulation to `"operating in the REAL WORLD on a physical hexapod"` without predefined landmarks, and expanded `.brain_secrets` search paths.
  - `?? src/rover/launch/real_autonomous.launch.py`: untracked hardware brain launch file.

### 1.3 README Summary
`gz_hex/README.md:1-137` (**VERIFIED**) presents the **Hexapod ("Zoe")** autonomous 6-legged robot platform built on **ROS 2 Jazzy**, **Gazebo Harmonic**, **SLAM Toolbox**, **Nav2**, and an **NVIDIA NIM** LLM/VLM agentic brain (`meta/llama-3.2-11b-vision-instruct` / `meta/llama-3.1-70b-instruct`), with an ESP32 serial bridge for 18 servos, Slamtec RPLidar driver, HSV color object detection, frontier exploration, and Google STT / `spd-say` voice interface. No standalone `LICENSE` file exists at `gz_hex/LICENSE` (**VERIFIED**: `ls -la gz_hex`).

### 1.4 Target OS, ROS Distro, Python, and Host Mismatch
- **Target environment (code/docs/build artifacts):** **Ubuntu 24.04 LTS**, **ROS 2 Jazzy Jalisco**, **Python 3.12**, **Gazebo Harmonic (`ros_gz_sim`)** (**VERIFIED**: `gz_hex/README.md:3-67`, `gz_hex/log/latest_build/events.log` showing `ROS_DISTRO=jazzy`, `/opt/ros/jazzy/...`, `python3.12/site-packages`, and `gz_hex/src/rover/rover/__pycache__/*.cpython-312.pyc`).
- **Legacy artifacts in repo:** `gz_hex/autonomous_brain_plan.md:3` mentions `ROS2 Humble`, and earlier `.cpython-310.pyc` caches dated Feb–Apr 2026 exist in `src/rover/rover/__pycache__/` (**VERIFIED**).
- **Current audit machine (`cat /etc/os-release`, `python3 --version`):** **Ubuntu 22.04.5 LTS (Jammy)**, **Python 3.10.12**, **no `/opt/ros/` directory**, `ros2: command not found`, `rclpy` missing, `openai` missing, `speech_recognition` missing (`serial 3.5`, `cv2 4.11.0`, `numpy 1.26.4`, `yaml 5.4.1`, `pytest 6.2.5` present; only `/usr/share/doc/ros-humble-*` docs exist) (**VERIFIED**).

### 1.5 Build Status & Tests
- **Local `colcon build` into temp dir:** **Cannot execute in this environment** because ROS 2 (`ament_cmake`, `rosidl_default_generators`, `rclcpp`, `rclpy`) is not installed (`/opt/ros/` absent) (**VERIFIED**).
- **Recorded build status (`gz_hex/log/latest_build` -> `build_2026-11-16_23-26-40`):** Built cleanly on target host (`HOME=/home/vishal`, ROS 2 Jazzy) with `colcon build` exiting `0` in ~7.0 seconds (`[7.295s] ... finished with '0'`, zero `stderr.log` bytes across `rover`, `rover_interfaces`, `rplidar_ros`) (**VERIFIED**: `gz_hex/log/latest_build/logger_all.log`, `events.log`).
- **Tests present and run status:**
  - `gz_hex/src/rover/package.xml:38-41` declares `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest` (`setup.py:29` has `tests_require=['pytest']`), **but no `test/` folder or unit tests exist** in `src/rover`, `src/rover_interfaces`, or `src/rplidar_ros` (**VERIFIED**: `find gz_hex/src -iname '*test*'`).
  - Root-level manual integration scripts exist: `gz_hex/test_brain_e2e.py` (spawns `sensor_bridge`, `tool_dispatcher`, `nvidia_brain` and sends `'Rotate 30 degrees to inspect the room'` — note this lacks the `'Hey Zoe'` wake word required by `nvidia_brain_node.py:495-505`!) and `gz_hex/test_voice_talk.py` (interactive mic STT/TTS utility). `src/rover/rover/test_esp.py` is a `JointState`->ESP calibration node (`cal_servo`), not a test. None can execute here without ROS 2/`rclpy` (**VERIFIED**).

---

## 2. Which Robot Does It Drive?

### 2.1 Real Hardware vs. Gazebo Simulation Mismatch
- **On real hardware (`real_navigation.launch.py`, `real_autonomous.launch.py`, `1_rviz.launch.py`):** Drives the **18-servo 6-legged hexapod** (`NCCUP1..6`, `NTHIGH1..6`, `NFOOT1..6`) via `hxp_cmd` -> `/hexapod_controller/joint_trajectory` -> `esp_bdg` -> ESP32 serial (**VERIFIED**: `real_navigation.launch.py:145-162`).
- **In Gazebo simulation (`2_gazebo.launch.py`, `3_gazebo.launch.py`, `sim_navigation.launch.py`, `autonomous.launch.py`):** Spawns `src/rover/urdf/hexa.urdf` (`robot name="hexa_body"`), which is a **hybrid wheeled differential-drive base with the 18-DOF hexapod bolted 0.25 m above it!** (**VERIFIED**: `hexa.urdf:10-430`).
  - `hexa.urdf:256-268` runs `gz::sim::systems::DiffDrive` on `/cmd_vel` driving `wheel_front_left_joint` and `wheel_front_right_joint` (`wheel_separation=0.3 m`, `wheel_radius=0.05 m`) supported by `front_castor_wheel` and `back_castor_wheel`.
  - Meanwhile, `sim_navigation.launch.py:108-114` launches `hxp_cmd` (`hxp_command.py`), which computes tripod gait IK and publishes `/joint_states` (for RViz/RSP visualization) and `/hexapod_controller/joint_trajectory`, **but no `ros2_control` `controller_manager` is launched in `sim_navigation.launch.py` and `hexa.urdf:1142-1264` has `<ros2_control>` without a Gazebo plugin block loading `controllers.yaml`** (**VERIFIED**). Therefore, in Gazebo simulation, locomotion physics is actually performed by **the hidden differential-drive wheels**, while the legs only animate visually in RViz (or hang above the wheels in Gazebo) (**INFERRED** from `hexa.urdf` + `sim_navigation.launch.py`).

### 2.2 URDF Inventory & Comparison with `NCCUP/NTHIGH/NFOOT 1-6`

| File | `robot name` | Links / Joints | Total Mass (kg) | Joints & Limits | Plugins / Sensors | Source |
|---|---|---|---|---|---|---|
| `urdf/hexa.urdf` *(active)* | `hexa_body` | 30 / 29 | `58.5084` | 18 revolute (`NCCUP{1..6}_joint`, `NTHIGH{1..6}_joint`, `NFOOT{1..6}_joint`, `lower=-1.5`, `upper=1.5` rad, `effort=30`, `velocity=6`, `damping=10.0`, `friction=1.0`), 2 continuous wheels (`wheel_front_right_joint`, `wheel_front_left_joint`), 9 fixed (`base_footprint_joint`, 2 casters, `camera_joint`, `lidar_joint`, `camera_optical_joint`, `imu_joint`, `wheel_fixed_joint` z=+0.25m, `TOP_PLATE_joint`) | `DiffDrive`, `Sensors`, `gz_ros2_control` (no plugin tag); `depth` camera, `gpu_lidar`, `imu_sensor`, `imu_sensor_base` | **VERIFIED** (`hexa.urdf:1-1266`) |
| `urdf/hexa_b1.urdf` | `hexa_body` | 21 / 20 | `14.5941` | `world_fixed_joint` + `TOP_PLATE_joint` + 18 revolute `NCCUP/NTHIGH/NFOOT{1..6}_joint` (`[-1.5, 1.5]` rad, `effort=3.0`, `velocity=6`) | `libgazebo_ros2_control.so` (Gazebo Classic) | **VERIFIED** (`hexa_b1.urdf`) |
| `urdf/hexa1.urdf` | `hexa_body` | 20 / 19 | `12.7581` | 18 `continuous` CAD-named joints (`base_link-v1_Revolute-*`, `NCCUP*_Revolute-*`, `NTHIGH*_Revolute-*`), no `<limit>`, 1 fixed | None | **VERIFIED** (`hexa1.urdf`) |
| `urdf/rover.urdf` | `rover` | 7 / 6 | `11.0000` | 2 continuous wheels + 4 fixed (pure wheeled rover) | Gazebo Classic `diff_drive`, `camera_plugin`, `ray` lidar | **VERIFIED** (`rover.urdf`) |
| `grip.urdf`, `grip3.urdf`, `grip_final.urdf` | `grip` | 9–10 / 8–9 | `4.0000` / `22.5900` | 2 wheels + 5-DOF arm (`base_link_Revolute-10`..`14`, `[-3.14, 3.14]`) | Gazebo Classic `diff_drive`, `gazebo_ros2_control`, `camera` | **VERIFIED** |
| `grip2.urdf` | *(XML parse error)* | — | — | Invalid XML token at line 241, col 12 | — | **VERIFIED** |

**`hexa.urdf` Mass Breakdown** (**VERIFIED**):
- Wheeled base: `base_link` (2.0), `wheel_front_right` (2.0), `wheel_front_left` (2.0), `front_castor_wheel` (2.0), `back_castor_wheel` (2.0), `camera` (0.5), `lidar` (0.5), `imu_link` (0.1) = **11.10 kg**.
- Hexapod chassis: `base_link-v1` (8.8091), `top-plate-v2` (5.0858), 6×`NCCUPi` (2.0586 each = 12.3518), 6×`NTHIGHi` (1.5224 each = 9.1343), 6×`NFOOTi` (2.0046 each = 12.0273) = **47.4084 kg** (note `hexa_b1.urdf` has realistic ~0.28/0.21/0.27 kg per leg segment totaling 14.59 kg, whereas `hexa.urdf` scales link masses ~7.4× heavier!).

**Joint Naming & Axis Comparison with `NCCUP/NTHIGH/NFOOT 1-6`:**
- Link names match `NCCUP1..6`, `NTHIGH1..6`, `NFOOT1..6` exactly; joint names add `_joint` (`NCCUP1_joint`..`NFOOT6_joint`) (**VERIFIED**: `hexa.urdf:450-1101`).
- **Critical axis asymmetry:** All 6 `NCCUP*_joint` have `axis="0 0 1"` with `rpy="pi 0 0"`. However, `NTHIGH*_joint` and `NFOOT*_joint` axes vary per leg due to CAD assembly orientation (`NTHIGH1`: `0.707107 0 0.707107`; `NTHIGH2`: `1 0 0`; `NTHIGH3`: `0 0 -1`; `NTHIGH4`: `-0.707107 0 -0.707107`; `NTHIGH5`: `-1 0 0`; `NTHIGH6`: `0 0 1`, and `NFOOT1..6` similarly rotate/flip signs) (**VERIFIED**: `hexa.urdf:958-1101`).

---

## 3. Runtime Graph

### 3.1 Launch Files & Entry Points (`src/rover/setup.py:30-51`, `src/rover/launch/*.py`)

| Launch File | Nodes Launched (pkg / executable / node name) | Mode / Notes |
|---|---|---|
| `1_rviz.launch.py` | `robot_state_publisher`, `joint_state_publisher_gui`, `rover/cal_servo` (`jointstate_to_esp`, if `hardware:=true`), `rviz2` (`view_robot.rviz`) | Manual joint slider calibration to ESP32 (**VERIFIED**: `1_rviz.launch.py:36-72`) |
| `2_gazebo.launch.py` | `robot_state_publisher`, `ros_gz_sim` (`sim_world.sdf`), `ros_gz_sim/create` (`rover` at `z=0.25`), `ros_gz_bridge/parameter_bridge`, `rviz2` (optional) | Jazzy Gazebo Harmonic test (**VERIFIED**: `2_gazebo.launch.py:69-130`) |
| `3_gazebo.launch.py` | Same as `2_gazebo.launch.py` without launch args (`use_sim_time=True` hardcoded) | Basic Gazebo Harmonic sim (**VERIFIED**: `3_gazebo.launch.py:28-95`) |
| `mapping.launch.py` | `slam_toolbox/async_slam_toolbox_node` (`slam_toolbox`, LifecycleNode auto-configured & activated) | Included by `sim_navigation.launch.py` (**VERIFIED**: `mapping.launch.py:31-66`) |
| `sim_navigation.launch.py` | `robot_state_publisher`, `ros_gz_sim`, `ros_gz_sim/create`, `ros_gz_bridge/parameter_bridge`, `rover/hxp_cmd` (`ik_controller`), `slam_toolbox` (via `mapping.launch.py`), Nav2 (`controller_server`, `smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`), `nav2_lifecycle_manager` (t=15s), `rviz2` (`nav2.rviz`) | Full Gazebo Nav2+SLAM (**VERIFIED**: `sim_navigation.launch.py:57-231`) |
| `real_navigation.launch.py` | `robot_state_publisher`, `rplidar_ros/rplidar_node`, `tf2_ros/static_transform_publisher` (`odom->base_footprint`), `rover/odom_from_tf`, `rover/esp_bdg` (`esp_bridge`), `rover/hxp_cmd` (`hexapod_controller`), `slam_toolbox`, 7 Nav2 nodes, `nav2_lifecycle_manager` (t=8s), `rviz2` (optional) | Real robot Nav2+SLAM (**VERIFIED**: `real_navigation.launch.py:91-308`) |
| `autonomous.launch.py` | Includes `sim_navigation.launch.py`, plus `rover/sensor_bridge` (t=20s), `rover/tool_dispatcher` (t=21s), `rover/nvidia_brain` (t=22s), `rover/voice_bridge` (t=23s, if `enable_voice:=true`) | Sim AI Brain stack (**VERIFIED**: `autonomous.launch.py:54-122`) |
| `real_autonomous.launch.py` | Includes `real_navigation.launch.py`, loads `.brain_secrets`, plus `sensor_bridge` (t=25s), `tool_dispatcher` (t=26s), `nvidia_brain` (t=27s), `voice_bridge` (t=28s, optional) | Real robot AI Brain stack (**VERIFIED**: `real_autonomous.launch.py:125-217`) |

**Unused Entry Points in `setup.py:31-51` (not in any launch file):**
- `teleop` (`rover.teleop:main`), `armc` (`rover.arm_control:main`), `arm_ik` (`rover.arm_ik_commander:main`), `object_detector` (`rover.object_detector:main`), `box_tracker` (`rover.box_tracker:main`), `tripod_gait` (`rover.tripod_gait:main`), `hxp_ik` (`rover.hexapod_ik_controller:main`), `imu_stabilizer` (`rover.imu_stabalizer:main`), `rviz_ik` (`rover.rviz_hexa_ik:main`), `hxp_nav_ctrl` (`rover.hexapod_nav_controller:main`) (**VERIFIED**). Note `rplidar_ros` also builds `rplidar_composition` and `rplidar_client` (`src/rplidar_ros/CMakeLists.txt:59-80`).

### 3.2 Topics, Services, Actions, and QoS

| Interface | Type | Publisher / Server | Subscriber / Client | Rate | QoS | Source |
|---|---|---|---|---|---|---|
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | `controller_server` (`20/5 Hz`), `behavior_server` (`10 Hz`) | `velocity_smoother` | 5–20 Hz | Default ROS 2 | **VERIFIED** (`real_navigation.launch.py:226-257`) |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `velocity_smoother` (`20/10 Hz`), `tool_dispatcher` (`20 Hz` direct), `teleop`, `box_tracker` | `hxp_cmd` (`ik_controller` / `hexapod_controller`), Gazebo `DiffDrive` bridge (sim) | 10–20 Hz (on event) | Depth 10, Reliable, Volatile | **VERIFIED** (`hxp_command.py:38`, `tool_dispatcher_node.py:100`, `teleop.py:103`) |
| `/hexapod_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | `hxp_cmd` (`8 Hz`), `imu_stabilizer` (`20 Hz`, unused) | `esp_bdg` (`esp_bridge`, real) | 8 Hz | Depth 10, Reliable, Volatile | **VERIFIED** (`hxp_command.py:25-49`, `esp.py:26-31`) |
| `/joint_states` | `sensor_msgs/msg/JointState` | `hxp_cmd` (`8 Hz`), `joint_state_publisher_gui` (`1_rviz`) | `robot_state_publisher`, `cal_servo` (`1_rviz`) | 8 Hz | Depth 10, Reliable, Volatile | **VERIFIED** (`hxp_command.py:31-35`, `test_esp.py:27-32`) |
| `/scan` | `sensor_msgs/msg/LaserScan` | `rplidar_node` (real) / `ros_gz_bridge` (sim, 10 Hz) | `slam_toolbox`, `local_costmap`, `global_costmap`, `sensor_bridge`, `tool_dispatcher` | ~6–10 Hz | `KeepLast(10)` (`rplidar_node.cpp:440`); default 10 in Python | **VERIFIED** |
| `/odom` | `nav_msgs/msg/Odometry` | `odom_from_tf` (real, `20 Hz`) / `ros_gz_bridge` (sim, `50 Hz`) | `bt_navigator`, `velocity_smoother`, `sensor_bridge` | 20 Hz (real) / 50 Hz (sim) | Depth 10 | **VERIFIED** (`odom_from_tf.py:41-42`, `frames_*.gv`) |
| `/tf` & `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher`, `slam_toolbox` (`map->odom`), `static_transform_publisher` (real), `ros_gz_bridge` (sim) | `odom_from_tf`, Nav2, `rviz2` | 10–50 Hz / latched | Default TF2 | **VERIFIED** |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `slam_toolbox` (`1.0 s` sim / `1.5 s` real) | `global_costmap`, `sensor_bridge`, `tool_dispatcher`, `rviz2` | 0.67–1 Hz | Reliable, `TRANSIENT_LOCAL`, depth 1 | **VERIFIED** (`sensor_bridge_node.py:95-106`, `tool_dispatcher_node.py:126-134`) |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | *(none launched)* | `sensor_bridge` | N/A | Depth 10 | **VERIFIED** (`sensor_bridge_node.py:107-110`) |
| `/camera/camera/image_raw` | `sensor_msgs/msg/Image` | *(not bridged in sim; no real driver)* | `sensor_bridge`, `object_detector`, `box_tracker` | N/A | Depth 10 | **VERIFIED** (`sensor_bridge_node.py:104-105`) |
| `/imu/data` | `sensor_msgs/msg/Imu` | *(not bridged in sim; no real driver)* | `imu_stabilizer` (unused) | N/A | Depth 10 | **VERIFIED** (`imu_stabalizer.py:18-23`) |
| `/brain/query` & `/brain/response` | `std_msgs/msg/String` | `voice_bridge` -> `/brain/query`; `nvidia_brain` & `tool_dispatcher` -> `/brain/response` | `nvidia_brain` <- `/brain/query`; `voice_bridge` <- `/brain/response` | Event-driven | Depth 10 | **VERIFIED** |
| `/get_sensor_snapshot` | `rover_interfaces/srv/SensorSnapshot` | `sensor_bridge_node` | `nvidia_brain_node`, `tool_dispatcher_node` | On demand | Service | **VERIFIED** |
| `/execute_tool` | `rover_interfaces/srv/ExecuteTool` | `tool_dispatcher_node` | `nvidia_brain_node` | On demand | Service | **VERIFIED** |
| `/start_motor`, `/stop_motor` | `std_srvs/srv/Empty` | `rplidar_node` | *(external CLI/client)* | On demand | Service | **VERIFIED** (`rplidar_node.cpp:442-445`) |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `bt_navigator` | `tool_dispatcher_node`, RViz2 | On demand | Action | **VERIFIED** (`tool_dispatcher_node.py:90-92`) |

### 3.3 Data Flow Diagram (Goal -> Servo Commands & Sensor Feedback)

```text
User ("Hey Zoe...") / RViz 2D Goal Pose
  │
  ▼ (if AI Brain active: /brain/query -> nvidia_brain -> /execute_tool -> tool_dispatcher)
Nav2 (bt_navigator -> planner_server [Navfn] -> controller_server [DWB @ 5Hz real / 20Hz sim])
  │                                  │
  │ /cmd_vel_nav (Twist)             │ (direct bypass on /cmd_vel during drive_distance / rotate / dance)
  ▼                                  ▼
velocity_smoother (10Hz real / 20Hz sim) ──▶ /cmd_vel (geometry_msgs/Twist)
                                               │
                 ┌─────────────────────────────┴──────────────────────────────┐
                 ▼ (REAL HARDWARE)                                            ▼ (GAZEBO SIM)
     hxp_command.py (8 Hz timer)                                  ros_gz_bridge (/cmd_vel)
     - Discretizes vx, wz -> f/b/l/r/s                                        │
     - 4-step Tripod Gait + 3-DOF IK                                          ▼
       ├──▶ /joint_states (8 Hz) -> RSP -> TF                     gz DiffDrive (wheels!)
       └──▶ /hexapod_controller/joint_trajectory (8 Hz)           (hxp_command animates RViz
                 │                                                 /joint_states only; legs
                 ▼                                                 not ros2_control driven)
     esp.py (esp_bridge)
     - rad -> deg + j1_neutral offsets + Leg3 flip + [0,180] clamp
                 │
                 ▼ USB Serial @ 115200 baud ("a1 a2 ... a18\n")
     ESP32 ──▶ 18 Hobby Servos (open-loop; NO feedback read back)

================ SENSOR & LOCALIZATION FLOW BACK ================
REAL HARDWARE:
  RPLidar A1 (/dev/ttyUSB*, 115200) -> rplidar_node -> /scan (frame: 'lidar')
    ├──▶ slam_toolbox -> /map (0.67 Hz) & TF (map -> odom @ 20 Hz)
    │                      └──▶ static_odom_broadcaster (odom -> base_footprint = I)
    │                      └──▶ odom_from_tf (20 Hz diff of map->base_footprint) -> /odom
    └──▶ Nav2 local/global costmaps, sensor_bridge_node, tool_dispatcher_node
  IMU: NONE (not present on real HW)   |   Camera: NONE (no real driver)

GAZEBO SIMULATION:
  gz gpu_lidar (10 Hz) -> ros_gz_bridge -> /scan -> slam_toolbox -> /map & TF (map -> odom)
  gz DiffDrive (50 Hz) -> ros_gz_bridge -> /odom & TF (odom -> base_footprint)
  gz IMU (100 Hz) & Depth Cam (30 Hz) -> NOT BRIDGED in ros_gz_bridge!
```

---

## 4. Locomotion Layer

- **Active Controller:** `src/rover/rover/hxp_command.py` (entry point `hxp_cmd`, launched in both `real_navigation.launch.py:157-162` and `sim_navigation.launch.py:108-114`).
  - *Unused/legacy alternatives present:* `hexapod_nav_controller.py` (`hxp_nav_ctrl`, 10 Hz continuous `target_vx/wz`, retains `leg_id==5` negation), `hexapod_ik_controller.py` (`hxp_ik`, open-loop 10 Hz forward walk), `rviz_hexa_ik.py` (`rviz_ik`, open-loop RViz), `tripod_gait.py` (`tripod_gait`, 1 Hz hardcoded joint angles).
- **Algorithm (`hxp_command.py:71-246`):**
  - **Command discretization (`cmd_vel_callback`, lines 111-133):** Maps continuous `(vx, wz)` to 5 discrete states `'f'`, `'b'`, `'l'`, `'r'`, `'s'` using `lin_threshold = 0.03` m/s, `ang_threshold = 0.08` rad/s, and turning priority if `abs(wz) > 1.2 * abs(vx)`. **`msg.linear.y` is completely ignored** (no lateral walking/strafing). Velocity magnitude does not scale stride length or cadence.
  - **Tripod gait table (`lines 72-104`):** Tripod A = `[1, 3, 5]`, Tripod B = `[2, 4, 6]` (phase offset `2` out of `4` steps). Each leg cycles through 4 `(x, y, z)` foot targets in **millimeters** (stance `z=150 mm`, lift `z=100 mm`, stride sweep ~`±25 mm` in `x`/`y`). Note: `right_gait` uses identical foot coordinates `(77.8,77.8,150)->(59.3,95.0,150)->(77.8,77.8,100)->(95.0,59.3,150)` for all 6 legs; `'b'` and `'l'` reverse the 4-point lists.
  - **3-DOF Analytical IK (`inverse_kinematics`, lines 172-193):** Link lengths `J1L=35.0 mm`, `J2L=75.0 mm`, `J3L=160.0 mm`. Computes `J1 = atan2(x, y)`, `H = hypot(x, y)`, `L = hypot(z, H - J1L)`, `J3 = 1.57 - acos(...)`, `J2 = B - A` (radians).
- **Rates & Messages:**
  - Timer `dt = 0.125 s` (**8 Hz**), `step_time = 0.5 s` -> advances 1 waypoint every 4 ticks (**2.0 s per 4-step gait cycle**).
  - Publishes `trajectory_msgs/msg/JointTrajectory` on `/hexapod_controller/joint_trajectory` (1 `JointTrajectoryPoint`, `time_from_start.sec = 1`, `positions` length 18 in **radians**) and `sensor_msgs/msg/JointState` on `/joint_states` (18 leg joints + `"wheel_front_right_joint"`, `"wheel_front_left_joint"` padded with `0.0` at `hxp_command.py:266-267`).
  - **Joint order (`lines 60-67`):** Leg-grouped `[NCCUP1_joint, NTHIGH1_joint, NFOOT1_joint, ..., NCCUP6_joint, NTHIGH6_joint, NFOOT6_joint]`.
- **ESP32 Serial Bridge (`src/rover/rover/esp.py`):**
  - Subscribes `/hexapod_controller/joint_trajectory` (`lines 26-31`), expects 18 positions (`lines 66-71`).
  - Converts radians to degrees and applies **calibrated offsets** (`lines 77-100`):
    - Coxa (`i % 3 == 0`): `servo_deg = 90.0 + (deg - j1_neutral[leg_id])`, where `j1_neutral` = `{1: 45.0, 2: 34.51, 3: 55.49, 4: 45.0, 5: 34.51, 6: 55.49}` deg.
    - Thigh/Foot (`i % 3 in (1, 2)`): for **Leg 3 only**, `servo_deg = 90.0 - deg`; for Legs 1,2,4,5,6, `servo_deg = 90.0 + deg`.
  - Clamps `[0.0, 180.0]` (`line 99`), rounds to 2 decimals, formats `"a1 a2 ... a18\n"` (`line 103`), writes over `pyserial` (`serial.Serial(port, 115200, timeout=1)`).
- **Servo Feedback / Telemetry:** **NONE.** `esp.py` and `test_esp.py` never read from the serial port; no joint position, velocity, temperature, load/current, or battery voltage is read or published (**VERIFIED**: `esp.py:1-127`).
- **Safety Logic:**
  - `hxp_command.py:45,208-211`: `cmd_timeout = 0.6 s` reverts `self.command = 's'` (standing pose) if `/cmd_vel` stops arriving.
  - `hxp_command.py:272-319`: `SIGINT`/`SIGTERM` triggers `reset_to_initial_pose()` (publishes stand pose 5 times).
  - `hxp_command.py:183-184`: `acos` argument clamped to `[-1.0, 1.0]`.
  - `esp.py:98-99`: servo output clamped to `[0.0, 180.0]` deg; reconnect attempt on write exception (`lines 45-58`).
  - `tool_dispatcher_node.py:364-381`: Dynamic Obstacle Shield halts `drive_distance` if LiDAR front ±35° < `0.40 m` and rotates ±45°.
  - **Missing safety:** No hardware e-stop, no servo rate/acceleration limiter, no trajectory interpolation, no ESP32-side watchdog if USB disconnects mid-stride (**VERIFIED**).

---

## 5. Nav2 Configuration

- **Version:** ROS 2 Jazzy Nav2 (`nav2_bringup` et al. declared in `package.xml:27-35`, exact patch version not pinned; built under `/opt/ros/jazzy` in `events.log`) (**VERIFIED**).
- **Key YAML Files:** `src/rover/config/nav2_real_params.yaml` (real hardware) and `src/rover/config/nav2_params.yaml` (Gazebo sim) (**VERIFIED**).
- **Plugins & Architecture:**
  - **Planner:** `nav2_navfn_planner::NavfnPlanner` (`use_astar: false`, `tolerance: 0.5`, `allow_unknown: true`).
  - **Controller:** `dwb_core::DWBLocalPlanner` (`FollowPath`). Critics: `["Oscillation", "BaseObstacle", "ObstacleFootprint", "PathAlign", "PathDist", "GoalDist"]` (`RotateToGoal` & `GoalAlign` explicitly removed so the hexapod doesn't spin at goal).
  - **Smoother:** `nav2_smoother::SimpleSmoother`.
  - **Behaviors:** `nav2_behaviors::Spin`, `BackUp`, `DriveOnHeading`, `Wait`.
  - **Behavior Tree:** `bt_navigator` (`NavigateToPoseNavigator`, `NavigateThroughPosesNavigator`, `bt_loop_duration: 10`, `default_server_timeout: 20`; uses Nav2 default BT XMLs — no custom XML in repo).
  - **Collision Monitor:** **Not configured / not launched.**
- **Costmap Layers & Sensors:**
  - Footprint (both): `[[0.27, 0.0], [0.14, 0.24], [-0.14, 0.24], [-0.27, 0.0], [-0.14, -0.24], [0.14, -0.24]]` (0.54 m × 0.48 m foot span), `resolution: 0.05`, `inflation_radius: 0.48`, `cost_scaling_factor: 4.0`.
  - `local_costmap`: `odom` frame, `3x3 m` rolling window, `5.0 Hz` update / `2.0 Hz` publish; **Sim** uses `voxel_layer` + `inflation_layer`; **Real** uses `obstacle_layer` + `inflation_layer`, both from `/scan` (`LaserScan`, `raytrace_max_range: 3.5` real / `3.0` sim, `obstacle_max_range: 3.0` real / `2.5` sim).
  - `global_costmap`: `map` frame, `1.0 Hz` update/publish, `static_layer` + `obstacle_layer` (`/scan`) + `inflation_layer`.
- **Differential vs. Holonomic, Limits, Smoother, Tolerances (`nav2_params.yaml` vs `nav2_real_params.yaml`):**
  - Configured as **differential-drive** (`max_vel_y: 0.0`, `acc_lim_y: 0.0`, `vy_samples: 1` real / `5` sim; `amcl.robot_model_type: DifferentialMotionModel`).
  - **Sim limits:** DWB `vx=[0.0, 0.5]`, `wz=[-1.0, 1.0]`, `acc_lim=[2.5, 0.0, 3.2]`; `velocity_smoother` (20 Hz, `OPEN_LOOP`) `max=[0.5, 0.0, 1.0]`, `min=[-0.5, 0.0, -1.0]`; `xy_goal_tolerance: 0.25`, `yaw_goal_tolerance: 3.15`.
  - **Real limits (field-tuned — explicitly commented in `nav2_real_params.yaml`):**
    - `controller_frequency: 5.0` (down from 20.0), `debug_trajectory_details: False` (CPU)
    - `max_vel_x: 0.12`, `max_speed_xy: 0.12`, `max_vel_theta: 0.5` (`"slightly below hexapod capability for stability"`, line 85)
    - `acc_lim_x: 0.3`, `acc_lim_theta: 0.8`, `vx_samples: 8`, `vtheta_samples: 12`, `sim_time: 1.5`, `trans_stopped_velocity: 0.05`
    - `velocity_smoother` (`10.0 Hz`, `OPEN_LOOP`): `max_velocity: [0.15, 0.0, 0.6]`, `min_velocity: [-0.10, 0.0, -0.6]`, `max_accel: [0.5, 0.0, 1.0]`
    - `progress_checker`: `required_movement_radius: 0.2`, `movement_time_allowance: 30.0` (`"hexapod is slow; was 15s"`, line 72)
    - `general_goal_checker`: `xy_goal_tolerance: 0.30` (`"slightly more forgiving"`, line 77), `yaw_goal_tolerance: 3.15` (`"any heading accepted"`, line 78)
    - `transform_tolerance: 0.8` across `controller_server`, `FollowPath`, `behavior_server` (`"raised: absorb SSH TF lag"`, lines 64, 103, 218).
- **Twist vs `TwistStamped`, Namespaces & Remappings:**
  - Uses **`geometry_msgs/msg/Twist`** (unstamped) everywhere (`enable_stamped_cmd_vel` is absent -> Jazzy default `false`).
  - Root namespace (`''`); `controller_server` and `behavior_server` remap `cmd_vel -> cmd_vel_nav`; `velocity_smoother` remaps `cmd_vel -> cmd_vel_nav`, `cmd_vel_smoothed -> cmd_vel` (`real_navigation.launch.py:226-257`).

---

## 6. Localization & Odometry

- **Active Localization:** `slam_toolbox` (`async_slam_toolbox_node`, `mode: mapping`) in both sim (`mapping.launch.py`) and real (`real_navigation.launch.py:165-200`). `nav2_amcl` blocks exist in both Nav2 YAMLs (and `sensor_bridge_node.py` subscribes to `/amcl_pose`), **but AMCL is never launched** (**VERIFIED**).
- **Odometry Source:**
  - **Simulation:** Gazebo `DiffDrive` (`hexa.urdf:262-265`) publishes `/odom` and TF `odom -> base_footprint` at ~50 Hz (**VERIFIED**: `frames_2026-10-05_23.46.30.gv:23`).
  - **Real Hardware:** **No wheel encoders, no leg kinematics odom, and no IMU.** `static_odom_broadcaster` publishes a static identity transform `odom -> base_footprint` (`real_navigation.launch.py:119-129`). `slam_toolbox` runs pure LiDAR scan-matching (`minimum_travel_distance: 0.0`, `minimum_travel_heading: 0.0` in `slam_toolbox_real_params.yaml:61-62`) and publishes `map -> odom` at 20 Hz (`transform_publish_period: 0.05`). Then `odom_from_tf` (`odom_from_tf.py:55-137`) looks up `map -> base_footprint` at 20 Hz, finite-differences pose to `(vx, vy, wz)`, and publishes `/odom` with `header.frame_id = "odom"` (aliasing map-frame pose/velocity into `odom` for Nav2) (**VERIFIED**).
- **TF Tree:**
  - **Sim (`frames_2026-10-05_23.46.30.gv` & `frames_2026-10-06_00.01.52.gv`):** `map -> odom` (`slam_toolbox`, ~10.3 Hz) -> `base_footprint` (`gz DiffDrive`, ~50.3 Hz) -> `base_link` -> `{wheel_front_left, wheel_front_right, front_castor_wheel, back_castor_wheel, camera->camera_link_optical, imu_link, lidar, base_link-v1 -> {top-plate-v2, NCCUP1..6 -> NTHIGH1..6 -> NFOOT1..6}}` (leg TFs ~10.2–10.3 Hz from `hxp_command`/RSP; static joints `10000.0` Hz) (**VERIFIED**).
  - **Real (`real_navigation.launch.py`):** Same URDF subtree below `base_footprint`, except `odom -> base_footprint` is static identity (`static_transform_publisher`), `map -> odom` is `slam_toolbox` at 20 Hz, and leg TFs update at 8 Hz from `hxp_command.py` -> `/joint_states` -> `robot_state_publisher` (**VERIFIED**).
- **Map Files:** `src/rover/maps/room_map.yaml` & `room_map.pgm` (201 × 202 px, `0.05 m/px`, origin `[-5.0174, -5.0736, 0.0]`, ~10×10 m matching `sim_world.sdf`, created Oct 20 02:37) (**VERIFIED**).

---

## 7. Sensors

| Sensor | Hardware / Sim Model | Driver / Source | Topic & Frame | Rate | Notes / Gaps |
|---|---|---|---|---|---|
| **LiDAR** | Real: **Slamtec RPLidar A1** (`real_navigation.launch.py:6`). Sim: Gazebo `gpu_lidar` | Real: `rplidar_ros/rplidar_node` (SDK v2.1.4, `115200` baud, `scan_mode: Sensitivity`, `angle_compensate: True`, `0.12–12.0 m`). Sim: `ros_gz_bridge` | `/scan` (`LaserScan`), frame `lidar` | Real: ~10 Hz (SLAM `min_time_interval: 0.15 s` -> ≤6.7 Hz); Sim: `10.0 Hz`, 360 samples | **VERIFIED** (`real_navigation.launch.py:103-116`, `hexa.urdf:332-356`, `slam_toolbox_real_params.yaml:52-54`) |
| **IMU** | Real: **None in workspace**. Sim: 2× Gazebo `imu` sensors (`imu_sensor` on `imu_link`, `imu_sensor_base` on `base_link-v1`) | Real: **No driver**. Sim: **Not bridged in `ros_gz_bridge`!** | `imu_stabalizer.py` expects `/imu/data` (`sensor_msgs/Imu`) | Sim SDF: `100 Hz` | `imu_stabalizer.py` reads `orientation` quaternion (NOT gyro/accel) and is unused (**VERIFIED**: `imu_stabalizer.py:18-63`, `2_gazebo.launch.py:111-117`) |
| **Camera** | Real: **None in workspace**. Sim: Gazebo `depth` camera (`640×480`, `R8G8B8`, `1.047 rad` FOV) | Real: **No driver**. Sim: **Not bridged in `ros_gz_bridge`!** | `sensor_bridge_node`, `object_detector`, `box_tracker` expect `/camera/camera/image_raw` | Sim SDF: `30 Hz` | Sim camera orientation is `rpy="0 0 3.14"` on `camera_joint` (**VERIFIED**: `hexa.urdf:191,313-328`) |
| **Audio** | USB/system microphone & speaker (`arecord` 16 kHz S16_LE mono, `spd-say`) | `voice_bridge_node.py` + Google Speech Recognition API | `/brain/query`, `/brain/response` | 8 s chunks | Requires internet + ALSA (`voice_bridge_node.py:89-128`) (**VERIFIED**) |

**USB Device Names & udev Rules:**
- `src/rplidar_ros/scripts/rplidar.rules:3`: `KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"` (CP210x USB-UART). **Warning:** Not used by `real_navigation.launch.py`, and if the ESP32 also uses a CP2102 (`10c4:ea60`), this rule creates a symlink race (**VERIFIED** / **INFERRED**).
- `real_navigation.launch.py:43-51` & `1_rviz.launch.py:16-21`: Scan `/dev/serial/by-path/*` matching PCI USB bus IDs `'05:00.3'` -> LiDAR (fallback `/dev/ttyUSB2` or `/dev/ttyUSB0`) and `'07:00.3'` -> ESP32 (fallback `/dev/ttyUSB1` or `/dev/ttyUSB0`).

---

## 8. Hardware I/O

- **Boards:** Host computer (`Jetson` / `RPi5` referenced in comments) + **ESP32** microcontroller (`real_navigation.launch.py:7`) + **18 servos** (inferred hobby PWM 0–180° servos from `esp.py:98-99` and `J1/J2/J3` neutral at `90°`).
- **Firmware Source:** **NOT FOUND** in repository (no `.ino`, `.c`, `.cpp`, PlatformIO, or ESP-IDF files) (**VERIFIED**).
- **Serial Ports & Baud Rates:**
  - ESP32: `/dev/ttyUSB0` or `/dev/ttyUSB1` or `/dev/serial/by-path/*07:00.3*`, **`115200` baud**, `timeout=1` (`esp.py:17-18,38`, `real_navigation.launch.py:45-51,152`).
  - RPLidar A1: `/dev/ttyUSB0` or `/dev/ttyUSB2` or `/dev/serial/by-path/*05:00.3*`, **`115200` baud** (`real_navigation.launch.py:44-51,110`).
- **Bus Protocol:**
  - ESP32: Unframed ASCII line of 18 space-separated float degrees (`0.00`–`180.00`) terminated by `\n` (`esp.py:100-103`); `test_esp.py:96-99` sends 18 integer degrees + `\n`. No header, checksum, sequence number, or readback.
  - RPLidar: Slamtec binary serial protocol via `rplidar_ros/sdk`.
- **Control & Publish Rates:** `hxp_command` 8 Hz (`dt=0.125s`); `esp_bridge` event-driven (~8 Hz); `odom_from_tf` 20 Hz; `slam_toolbox` TF 20 Hz, scan min interval 0.15 s; `nav2_real` controller 5 Hz, local costmap 5 Hz, global costmap 1 Hz, velocity smoother 10 Hz (**VERIFIED**).
- **Documented Latencies:** No measured latency logs exist. Qualitative timing tolerances tuned for SSH/Jetson lag: `bond_timeout: 10.0 s` (`real_navigation.launch.py:272`), `transform_tolerance: 0.8 s` (`nav2_real_params.yaml:64,103,218`), `slam transform_timeout: 0.5 s` (`slam_toolbox_real_params.yaml:55`), `snapshot_cache_sec: 0.5 s` (`brain_config.yaml:24`), staggered Nav2 launch `t=8 s` and brain launch `t=25–28 s` (`real_autonomous.launch.py:213-216`).

---

## 9. Field-Test Evidence

- **Rosbags / Runtime Logs:** **None found** (`*.bag`, `*.db3` excluded by `.gitignore:39-40` and absent from disk) (**VERIFIED**).
- **Artifacts & Code Evidence of Testing:**
  1. **Gazebo TF captures:** `gz_hex/frames_2026-10-05_23.46.30.gv/.pdf` and `frames_2026-10-06_00.01.52.gv/.pdf` show live sim TF trees (`map->odom` @ 10.3 Hz, `odom->base_footprint` @ 50.3 Hz, leg joints @ 10.2–10.3 Hz) (**VERIFIED**).
  2. **Saved map:** `gz_hex/src/rover/maps/room_map.pgm` & `room_map.yaml` (Oct 20, 2026; 201×202 @ 0.05 m/px = 10.05×10.10 m, origin `[-5.0174, -5.0736]`) — matches the 10×10 m `sim_world.sdf` room (**INFERRED** saved from Gazebo sim).
  3. **Build history:** 25 colcon build logs in `gz_hex/log/build_*` spanning 2026-10-05 to 2026-11-16 (**VERIFIED**).
  4. **Uncommitted real-hardware bringup evidence (`git -C gz_hex diff` + `nav2_real_params.yaml` + `slam_toolbox_real_params.yaml` + `real_navigation.launch.py`):**
     - Hardware servo calibration fixes: Leg 3 J2/J3 inversion (`esp.py:93-94`), removal of Leg 5 negation (`hxp_command.py`), `j1_neutral` per-leg offsets (`esp.py:77-84`).
     - Real-robot stability & compute tuning: `max_vel_x` lowered `0.5 -> 0.12 m/s`, `movement_time_allowance` raised `15 -> 30 s`, `xy_goal_tolerance` `0.25 -> 0.30 m`, `controller_frequency` `20 -> 5 Hz`, `bond_timeout` `4 -> 10 s`, `transform_tolerance` `-> 0.8 s`, PCI USB path pinning `05:00.3` / `07:00.3`.
     - **No terrain tests, speed benchmarks, battery life measurements, or failure reports** are documented anywhere (**VERIFIED**).

---

## 10. Compute

- **Jetson Model & Power Mode:** **Exact Jetson model and `nvpmodel` / `jetson_clocks` configuration NOT FOUND.** Comments reference `"Jetson"` (`slam_toolbox_real_params.yaml:50,54`, `real_navigation.launch.py:260`), `"RPi5 / Jetson"` (`real_autonomous.launch.py:15`), and `"Jetson Orin"` only as a future local-NIM target (`autonomous_brain_plan.md:575`) (**VERIFIED**). *(Note: PCI bus IDs `05:00.3`/`07:00.3` in `/dev/serial/by-path/*` are commonly seen on AMD x86/laptop USB host controllers rather than Jetson Orin/Nano, suggesting either a test laptop or specific carrier setup — **INFERRED**).*
- **Expected CPU / RAM Load:** No measurements recorded. Config comments indicate CPU and SSH/network pressure on the robot prompted reducing `controller_frequency` (`20 -> 5 Hz`), DWB sampling (`20×20 -> 8×12`), `debug_trajectory_details: False`, `slam_toolbox` `map_update_interval` (`1.0 -> 1.5 s`), `transform_publish_period` (`0.02 -> 0.05 s`), and `minimum_time_interval` (`0.1 -> 0.15 s`). LLM inference runs off-device via HTTPS (`integrate.api.nvidia.com/v1`), while `sensor_bridge` + `tool_dispatcher` + `frontier_exploration` use OpenCV/NumPy (**VERIFIED**).
- **Real-Time Settings:** **None.** No `PREEMPT_RT`, `SCHED_FIFO`, `chrt`, `mlockall`, or CPU affinity isolation is configured anywhere (**VERIFIED**).

---

## 11. Integration with a 50 Hz Learned Locomotion Policy

**Policy Interface:** Subscribes `/cmd_vel` (`vx ∈ [-0.25, 0.30]` m/s, `vy ∈ [-0.20, 0.20]` m/s, `wz ∈ [-0.6, 0.6]` rad/s, body frame); reads **18 joint positions + velocities** and **IMU (gyro rad/s + gravity direction)** at **50 Hz**; publishes **18 joint position targets at 50 Hz**.

1. **Existing Nodes to Disable / Replace:**
   - **Real & Sim:** Disable `hxp_cmd` (`rover.hxp_command:main`, launched as `hexapod_controller` in `real_navigation.launch.py:157-162` and `ik_controller` in `sim_navigation.launch.py:108-114`). Never launch `imu_stabilizer`, `hxp_nav_ctrl`, `hxp_ik`, `tripod_gait`, or `rviz_ik`.
   - **Real Hardware Bridge (`esp_bdg` / `esp.py`):** **Cannot be used unmodified.** It subscribes to `JointTrajectory` (not a fast 50 Hz array/JointState target), injects `hxp_command`-specific `j1_neutral` offsets (`34.5°–55.5°`) and Leg 3 inversions, logs `INFO` on every packet, and **reads zero feedback** (no joint positions/velocities). Must be replaced with a bidirectional 50 Hz hardware interface (and ESP32 firmware must support reading/reporting joint states—impossible with standard open-loop hobby PWM servos without encoders/bus servos!).
   - **Simulation (`hexa.urdf` + `sim_navigation.launch.py`):** Must remove `gz::sim::systems::DiffDrive` (`hexa.urdf:256-268`) and the wheeled base/casters (`hexa.urdf:33-164`), lower `base_link-v1` to `base_link`, and enable a 50+ Hz `gz_ros2_control` `JointGroupPositionController` + `JointStateBroadcaster` (and fix `hexa.urdf`'s inflated ~58.5 kg masses back to physical ~3–15 kg!).

2. **Topic & Hardware Collisions:**
   - `/cmd_vel`: In sim, `DiffDrive` subscribes to `/cmd_vel` (`hexa.urdf:261`) — collides directly with policy locomotion. In autonomous mode, `tool_dispatcher_node` (`_tool_drive_distance`, `_tool_rotate`, `_tool_dance`, `_tool_find_object`, `_tool_stop`) and `box_tracker` publish directly to `/cmd_vel`, bypassing `velocity_smoother` and commanding up to `wz = ±1.0 rad/s` (exceeding policy `±0.6 rad/s`).
   - `/hexapod_controller/joint_trajectory` & `/joint_states`: `hxp_command` publishes both; must be disabled so `robot_state_publisher` receives true measured `/joint_states` from the hardware/sim bridge.
   - `/imu/data`: Currently absent in real hardware and unbridged in `sim_navigation.launch.py` (`ros_gz_bridge`). Policy needs a real 50+ Hz IMU driver (publishing gyro `angular_velocity` + `linear_acceleration`/orientation for gravity vector) and `/imu/data` bridged in sim.
   - **USB / udev:** `rplidar.rules` matches all `10c4:ea60` CP210x devices; if the ESP32/IMU also uses CP210x, udev symlinks collide.

3. **Joint Order & Angle Conventions vs. Policy:**
   - **Joint order & units:** `hxp_command.py:60-67` and `controllers.yaml:13-31` already use **leg-grouped `NCCUP{i}_joint, NTHIGH{i}_joint, NFOOT{i}_joint` (`i=1..6`) in radians** — matches policy spec!
   - **CRITICAL SIGN & ZERO-OFFSET MISMATCHES:**
     1. **`hexa.urdf` joint axes are non-uniform across legs** (`hexa.urdf:958-1101`): e.g., `NTHIGH1..6` and `NFOOT1..6` have different signs/rotations per leg (`NTHIGH3` axis `0 0 -1` vs `NTHIGH6` `0 0 1`; `NTHIGH2` `1 0 0` vs `NTHIGH5` `-1 0 0`). Verify whether the policy was trained on this exact `hexa.urdf` or a symmetric URDF (`hexa_b1.urdf` has identical axes to `hexa.urdf`, whereas `hexa1.urdf` differs completely!).
     2. **`esp.py:77-96` applies gait-specific offsets and sign flips** (`90 + (deg - j1_neutral[leg])` where `j1_neutral ∈ {34.51°, 45°, 55.49°}`, plus Leg 3 `90 - deg`). Sending URDF-zero-aligned policy targets through `esp.py` will misalign all 6 coxae by 34.5°–55.5°!
     3. **`hexa.urdf` joint limits** are `[-1.5, 1.5]` rad (`±85.9°`), `effort=30`, `velocity=6 rad/s`.

4. **Which Odometry Becomes Invalid:**
   - **Sim Gazebo `DiffDrive` `/odom` & `odom -> base_footprint` TF:** Completely disappears once wheels are removed; must be replaced by IMU + leg-kinematic / contact-aided odometry (or SLAM TF).
   - **Real `odom_from_tf.py` `/odom`:**
     1. `odom_from_tf.py:96-98` computes `vx = dx/dt`, `vy = dy/dt` in the **global `map` frame**, NOT `base_footprint` body frame! Once the policy walks fast or holonomically (`vy != 0`), Nav2 `velocity_smoother` / costmaps expecting body-frame `twist` receive rotated velocities.
     2. Pure 2D LiDAR scan-matching (`slam_toolbox` with `static_odom_broadcaster` identity `odom->base_footprint`) degrades at higher speeds (`0.30 m/s`, `0.6 rad/s`), lateral motion (`vy = ±0.20 m/s`), and dynamic body roll/pitch tilting the 2D LiDAR plane. A true `odom -> base_footprint` source (or EKF fusing IMU + policy/joint odometry) will be needed (**INFERRED**).

5. **Which Nav2 Parameters Must Change (`nav2_real_params.yaml` & `nav2_params.yaml`):**
   - **`controller_server` (`FollowPath`, DWB):**
     - Enable holonomic motion: `max_vel_y: 0.20`, `min_vel_y: -0.20`, `vy_samples: 5` (or `7`, up from `1`!), `acc_lim_y: 0.6`, `decel_lim_y: -0.6`.
     - Update X/theta limits: `max_vel_x: 0.30`, `min_vel_x: -0.25` (if backward Nav2 backing up/pathing desired; else `0.0`), `max_speed_xy: 0.30` (or `0.36`), `max_vel_theta: 0.6`, `acc_lim_x: 0.8`, `decel_lim_x: -0.8`, `acc_lim_theta: 1.2`, `decel_lim_theta: -1.2`.
     - Increase `controller_frequency` from `5.0` -> `10.0–20.0` Hz (CPU permitting) so DWB reacts smoothly to 50 Hz policy locomotion.
   - **`velocity_smoother`:**
     - `max_velocity: [0.30, 0.20, 0.6]`, `min_velocity: [-0.25, -0.20, -0.6]`, `max_accel: [0.8, 0.6, 1.2]`, `max_decel: [-0.8, -0.6, -1.2]`, `smoothing_frequency: 20.0` (or `50.0`).
   - **`amcl` (if ever enabled):** `robot_model_type: "nav2_amcl::OmniMotionModel"` (currently `DifferentialMotionModel`).
   - **`progress_checker`:** Tighten `movement_time_allowance` (`30.0 -> 10.0–15.0 s`) / `required_movement_radius` (`0.2 -> 0.3 m`) for faster walking.
   - **`tool_dispatcher_node.py`:** Clamp `dance` (`wz = ±1.0 -> ±0.6`), `drive_distance` (`vx ∈ [-0.25, 0.30]`), and ideally publish to `cmd_vel_nav` so `velocity_smoother` / `collision_monitor` stays in the loop.

---

## 12. Top 10 Risks & Unknowns (Ranked)

| Rank | Risk / Unknown | Evidence | Cheapest Way to Resolve |
|---|---|---|---|
| **1** | **No 50 Hz Joint Position/Velocity Feedback & Missing ESP32 Firmware:** `esp.py` is write-only ASCII; ESP32 firmware source is absent. Standard hobby PWM servos cannot report position/velocity at 50 Hz—blocking a closed-loop RL policy! | `esp.py:45-110`, firmware search = 0 files | Inspect physical servos/ESP32 wiring and firmware repo; verify whether servos have analog pot taps / serial bus (Feetech/Dynamixel) or if open-loop estimator is forced. |
| **2** | **No Real IMU Driver & Unbridged Sim IMU:** Policy requires 50 Hz IMU (gyro rad/s + gravity vector); real stack has zero IMU drivers, and Gazebo `hexa.urdf` IMU is not bridged. | `real_navigation.launch.py`, `2_gazebo.launch.py:111-117`, `imu_stabalizer.py:18-23` | Identify physical IMU chip (BNO085/MPU6050/ICM20948/RealSense IMU) and add ROS 2 driver + add `/imu/data` to `ros_gz_bridge`. |
| **3** | **Sim URDF Uses Hidden Diff-Drive Wheels (`hexa.urdf`) & Inflated Mass (58.5 kg):** Sim Nav2 never tested legged physics (`DiffDrive` drives wheels while `hxp_command` fakes `/joint_states`), and `hexa.urdf` leg masses are ~7.4× heavier than `hexa_b1.urdf` (14.6 kg). | `hexa.urdf:33-164,256-268,434-939`, `sim_navigation.launch.py:93-114` | Inspect `hexa.urdf` vs `hexa_b1.urdf` vs physical robot weight on a scale; strip `DiffDrive`/wheels and add `gz_ros2_control` plugin for sim validation. |
| **4** | **URDF Axis & Hardware Servo Offset Mismatches (`esp.py` vs `hexa.urdf`):** `hexa.urdf` has per-leg mirrored axes (`NTHIGH*`/`NFOOT*`), and `esp.py:77-96` subtracts `j1_neutral` (`34.5°–55.5°`) and flips Leg 3. Policy trained in Isaac/MuJoCo may expect different zero poses/signs. | `hexa.urdf:958-1101`, `esp.py:77-96`, `git diff hxp_command.py` | Compare policy training URDF/MJCF joint axes & zero-pose (`q=0`) against `hexa.urdf` and run a single-joint sign check (`1_rviz` with robot suspended). |
| **5** | **Serial Bandwidth & Jitter at 115200 baud / 50 Hz:** 18 floats as ASCII (`~130 B/line` × 50 Hz = 6.5 KB/s) with `esp.py` logging every line, no checksum, no `PREEMPT_RT`, and USB hub latency may drop/corrupt frames. | `esp.py:103-109`, `real_navigation.launch.py:152` | Switch ESP32 protocol to fixed binary packet + CRC16 (or increase baud to `921600`) and benchmark loopback round-trip jitter on the Jetson. |
| **6** | **No Odometry Fallback + Body-Frame Twist Bug in `odom_from_tf.py`:** Real stack relies 100% on 2D LiDAR `slam_toolbox` (`odom->base_footprint` = identity), and `odom_from_tf` outputs world-frame `dx/dt, dy/dt` as `twist`. Dynamic walking tilt + `vy` strafing will break SLAM/Nav2. | `real_navigation.launch.py:119-142`, `odom_from_tf.py:85-118` | Fix `odom_from_tf.py` to rotate twist into `base_footprint` frame, and test LiDAR scan-matching stability during fast/lateral/tilted gait. |
| **7** | **Unknown Jetson Model, Power Mode, & CPU Headroom:** Real stack already struggled at 20 Hz Nav2 / 20 Hz SLAM (had to drop DWB to 5 Hz), and 50 Hz policy + PyTorch/ONNX inference + OpenCV + SLAM may overload CPU/thermals. | `slam_toolbox_real_params.yaml:49-55`, `nav2_real_params.yaml:59-82` | Run `cat /etc/nv_tegra_release`, `nvpmodel -q`, `tegrastats` on the target Jetson while running SLAM + Nav2 + policy inference. |
| **8** | **Zero Automated Tests & Untested `test_brain_e2e.py` Bug:** No unit tests exist; `test_brain_e2e.py:46` sends a query without `'Hey Zoe'`, failing the wake-word check in `nvidia_brain_node.py:495-505`. | `find gz_hex/src -iname '*test*'`, `test_brain_e2e.py:46` | Add pytest unit tests for IK, `esp` angle mapping, `odom_from_tf` frame math, and policy joint-mapping; fix `test_brain_e2e.py`. |
| **9** | **No Hardware E-Stop, Watchdog, or Collision Monitor:** `tool_dispatcher` bypasses Nav2 `/cmd_vel_nav` during `drive_distance`/`dance`, `esp.py` has no timeout stop if ROS dies, and Nav2 `collision_monitor` is absent. | `tool_dispatcher_node.py:100,338-498`, `esp.py` | Add ESP32 firmware command watchdog (e.g. 200 ms timeout -> damp/hold), hardware kill switch on servo power rail, and Nav2 `collision_monitor`. |
| **10** | **USB Device Port Fragility (`by-path` vs `rplidar.rules`):** `real_navigation.launch.py:48-51` hardcodes PCI paths `'05:00.3'` and `'07:00.3'`, while `rplidar.rules` matches generic CP210x `10c4:ea60`. Replugging USB or switching SBCs breaks device detection. | `real_navigation.launch.py:43-51`, `rplidar.rules:3` | Query `udevadm info -a /dev/ttyUSB*` on the target Jetson and create serial-number-specific udev rules (`/dev/rplidar`, `/dev/esp32_hex`). |

---

## One-Page Summary

The `gz_hex` ROS 2 workspace (`TeamArtemisPPG/Hexapod`, targeting **ROS 2 Jazzy / Ubuntu 24.04 / Python 3.12**) contains three packages—`rover` (Python nodes, launch files, configs, URDFs, CAD/FEA), `rover_interfaces` (`SensorSnapshot.srv`, `ExecuteTool.srv`), and `rplidar_ros` (v2.1.4)—which last built cleanly on the author's Jazzy host (Nov 16, 2026) but cannot be built on this Ubuntu 22.04 machine where `/opt/ros/` is absent. No automated tests or rosbags exist.

On real hardware, the stack drives an **18-DOF hexapod** (`NCCUP/NTHIGH/NFOOT 1-6`) via `hxp_command.py` (an 8 Hz open-loop 4-waypoint tripod-gait IK node converting `/cmd_vel` into discrete `f/b/l/r/s` commands, ignoring `linear.y`) and `esp.py` (a write-only 115200-baud ASCII serial bridge to an **ESP32** whose firmware is missing from the repo). Localization relies solely on 2D LiDAR (`rplidar_node` for RPLidar A1) feeding `slam_toolbox` with an identity `odom -> base_footprint` static TF and `odom_from_tf.py` aliasing `map -> base_footprint` back onto `/odom`. Nav2 (`nav2_real_params.yaml`) is heavily field-tuned downward for stability and CPU/SSH lag (`DWB` @ 5 Hz, `max_vel_x=0.12 m/s`, `max_vel_y=0.0`, `xy_goal_tolerance=0.30 m`, `yaw_goal_tolerance=3.15 rad`). By contrast, Gazebo simulation (`hexa.urdf`, 58.5 kg) secretly drives a **differential-drive wheeled chassis** (`gz::sim::systems::DiffDrive`) bolted beneath the hexapod while `hxp_command` only animates RViz `/joint_states`; both the Gazebo IMU and depth camera are unbridged.

Integrating a **50 Hz holonomic learned locomotion policy** is feasible in joint naming/units (already leg-grouped `NCCUP/NTHIGH/NFOOT 1-6` in radians) and Nav2 reconfiguration (enabling `vy=±0.20 m/s`, `vy_samples`, `vx ∈ [-0.25, 0.30]`, `wz=±0.6`), **but faces four immediate hardware/software blockers**: (1) **zero joint position/velocity feedback** and missing ESP32 firmware, (2) **no real IMU driver** (and unbridged sim IMU), (3) **non-uniform URDF joint axes** (`hexa.urdf`) plus hardcoded gait offsets/Leg-3 sign flips in `esp.py`, and (4) **no legged sim actuation** (`DiffDrive` wheels in `hexa.urdf` must be removed and `gz_ros2_control` wired up).

---

## 10 Questions That Could Not Be Answered From the Code

1. **What servos and ESP32 firmware are on the physical robot?** Do the 18 servos provide 50 Hz position/velocity feedback (e.g., Feetech/Dynamixel serial bus or analog pot ADC), or are they open-loop hobby PWM servos? Where is the ESP32 firmware repo?
2. **What is the physical IMU hardware (if any)?** Does the robot carry a standalone IMU (e.g., BNO085, MPU6050, ICM-20948) or an IMU inside a camera (e.g., RealSense D435i), and over which bus (I2C/SPI/UART/USB) is it wired?
3. **What exact Jetson (or RPi5) SBC and `nvpmodel` power mode are deployed on the robot**, and why do the launch files look for PCI USB paths `'05:00.3'` and `'07:00.3'`?
4. **Was the learned locomotion policy trained on `hexa.urdf` (with its per-leg mirrored joint axes and 58.5 kg mass), `hexa_b1.urdf` (14.6 kg), or a separate MuJoCo/Isaac MJCF/URDF?** What is the policy's exact zero-angle (`q=0`) physical leg posture?
5. **What is the true measured mass and link inertia of the physical hexapod** (is it ~14.6 kg like `hexa_b1.urdf`, or lighter), and what is the servo stall torque / max continuous current?
6. **Is there a physical camera on the real hexapod**, or were `/camera/camera/image_raw` and `object_detector`/`box_tracker` only used in Gazebo / planned?
7. **Was `room_map.pgm` captured in Gazebo `sim_world.sdf` or in a physical lab**, and what real-world terrain/flooring has the robot actually walked on?
8. **What battery chemistry, voltage, regulator rails (5–7.4 V servo BEC vs Jetson rail), and runtime per charge** does the robot have?
9. **Are there physical foot contact sensors or compliant rubber feet**, and what servo zero-trim calibration offsets are baked into the ESP32 firmware vs `esp.py` (`j1_neutral`, Leg 3 flip)?
10. **Were the uncommitted edits in `esp.py`, `hxp_command.py`, `nvidia_brain_node.py`, and `real_autonomous.launch.py` tested successfully on the physical robot**, and why was Leg 5 negation removed in `hxp_command.py` while Leg 3 negation was added in `esp.py`?

