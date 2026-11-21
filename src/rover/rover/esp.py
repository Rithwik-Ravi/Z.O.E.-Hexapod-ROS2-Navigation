#!/usr/bin/env python3
"""
esp.py — ESP32 Serial Hardware Bridge for 18-DOF Hexapod Servos.

Subscribes to `/hexapod_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`),
converts the 18 joint angles from radians to calibrated servo degrees `[0.0, 180.0]`,
and streams space-separated ASCII frames (`"a1 a2 ... a18\\n"`) over USB serial at
115200 baud to the onboard ESP32 servo controller.
"""

import math
from typing import List

import rclpy
from rclpy.node import Node
import serial
from trajectory_msgs.msg import JointTrajectory


# Calibrated neutral J1 atan2(x, y) angles (deg) matching `hxp_command.py` stance
J1_NEUTRAL_DEG = {
    1: 45.0,
    2: math.degrees(math.atan2(55.0, 80.0)),
    3: math.degrees(math.atan2(80.0, 55.0)),
    4: 45.0,
    5: math.degrees(math.atan2(55.0, 80.0)),
    6: math.degrees(math.atan2(80.0, 55.0)),
}


def positions_rad_to_servo_deg(positions_rad: List[float]) -> List[float]:
    """
    Convert 18 leg-grouped joint positions (radians) to calibrated servo degrees `[0, 180]`.

    Joint ordering: `[NCCUP1, NTHIGH1, NFOOT1, ..., NCCUP6, NTHIGH6, NFOOT6]`.
    Leg 3 thigh/foot servos are inverted (`90 - deg`) to match physical horn mounting.
    """
    if len(positions_rad) != 18:
        raise ValueError(f'Expected 18 joint positions, got {len(positions_rad)}')

    angles_deg: List[float] = []
    for i, pos_rad in enumerate(positions_rad):
        angle_deg = math.degrees(pos_rad)
        leg_id = (i // 3) + 1
        joint_type = i % 3

        if joint_type == 0:
            # Coxa (J1): center physical 90 deg around calibrated leg neutral angle
            servo_deg = 90.0 + (angle_deg - J1_NEUTRAL_DEG[leg_id])
        elif leg_id == 3 and joint_type in (1, 2):
            # Leg 3 thigh/foot servo orientation is mechanically mirrored
            servo_deg = 90.0 - angle_deg
        else:
            servo_deg = 90.0 + angle_deg

        servo_deg = max(0.0, min(180.0, servo_deg))
        angles_deg.append(round(servo_deg, 2))
    return angles_deg


class ESPBridge(Node):
    """Streams calibrated 18-servo degree commands to an ESP32 over USB serial."""

    def __init__(self):
        super().__init__('esp_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.ser = None

        self.connect_serial()

        self.sub = self.create_subscription(
            JointTrajectory,
            '/hexapod_controller/joint_trajectory',
            self.callback,
            10,
        )

        self.get_logger().info(f'ESP32 Hardware Bridge started on {self.port} @ {self.baud} baud')

    def connect_serial(self):
        """Open the USB serial connection to the ESP32 controller."""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info(f'Connected to {self.port}')
        except Exception as exc:
            self.get_logger().error(f'Serial connection failed on {self.port}: {exc}')
            self.ser = None

    def send_data(self, data: str):
        """Write an ASCII servo frame to serial, attempting reconnect on failure."""
        if self.ser is None:
            self.connect_serial()
            return

        try:
            self.ser.write(data.encode())
        except Exception as exc:
            self.get_logger().warn(f'Serial write error: {exc}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def callback(self, msg: JointTrajectory):
        """Convert incoming 18-joint trajectory point to degrees and transmit."""
        if not msg.points:
            return

        positions = msg.points[0].positions
        if len(positions) != 18:
            self.get_logger().warn(f'Expected 18 joints, got {len(positions)}')
            return

        angles_deg = positions_rad_to_servo_deg(list(positions))
        data = ' '.join(map(str, angles_deg)) + '\n'
        self.send_data(data)
        self.get_logger().debug(f'Sent: {data.strip()}')


def main(args=None):
    """Entry point for the `esp_bdg` (`esp_bridge`) ROS 2 node."""
    rclpy.init(args=args)
    node = ESPBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.ser is not None:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
