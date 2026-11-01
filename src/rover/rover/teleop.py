#!/usr/bin/env python3
"""
teleop.py — Keyboard Teleoperation Node for Hexapod Rover.

Publishes geometry_msgs/msg/Twist to /cmd_vel based on keyboard inputs.
Supports linear velocities (x/y), angular turning (yaw), speed adjustments,
and instant stop. Compatible with ROS 2 Jazzy and hexapod IK locomotion controllers.
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
=====================================================
   HEXAPOD ROVER KEYBOARD TELEOPERATION (ROS 2 Jazzy)
=====================================================
Moving around:
        w
   a    s    d

   w / Up arrow    : Forward
   s / Down arrow  : Backward
   a / Left arrow  : Turn Left
   d / Right arrow : Turn Right
   q               : Strafe Left
   e               : Strafe Right

Speed Control:
   + / = : Increase linear & angular speed by 10%
   - / _ : Decrease linear & angular speed by 10%

Stopping:
   space / x : Stop immediately (publish 0 velocity)

Quit:
   Ctrl-C or 'k'

-----------------------------------------------------
"""

MOVE_BINDINGS = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 0.0, 1.0),
    'd': (0.0, 0.0, -1.0),
    'q': (0.0, 1.0, 0.0),
    'e': (0.0, -1.0, 0.0),
    # Arrow keys (handled after escape sequence)
    'UP': (1.0, 0.0, 0.0),
    'DOWN': (-1.0, 0.0, 0.0),
    'LEFT': (0.0, 0.0, 1.0),
    'RIGHT': (0.0, 0.0, -1.0),
}

SPEED_BINDINGS = {
    '+': (1.1, 1.1),
    '=': (1.1, 1.1),
    '-': (0.9, 0.9),
    '_': (0.9, 0.9),
}


def get_key(settings, timeout=0.1):
    """Read a single keypress or key sequence from terminal."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # Escape sequence for arrow keys
            rlist_seq, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist_seq:
                seq = sys.stdin.read(2)
                if seq == '[A':
                    key = 'UP'
                elif seq == '[B':
                    key = 'DOWN'
                elif seq == '[C':
                    key = 'RIGHT'
                elif seq == '[D':
                    key = 'LEFT'
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.declare_parameter('speed', 0.25)
        self.declare_parameter('turn', 0.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.speed = float(self.get_parameter('speed').value)
        self.turn = float(self.get_parameter('turn').value)
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.get_logger().info(
            f"Teleop node started. Publishing to '{cmd_vel_topic}' (speed={self.speed}, turn={self.turn})"
        )

    def publish_twist(self, vx, vy, wz):
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(wz)
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    # Verify stdin is an interactive terminal
    if not sys.stdin.isatty():
        node.get_logger().warn("Teleop requires an interactive terminal (stdin is not a tty).")
        node.destroy_node()
        rclpy.shutdown()
        return

    orig_settings = termios.tcgetattr(sys.stdin)
    print(BANNER)
    print(f"Current limits: speed = {node.speed:.2f} m/s | turn = {node.turn:.2f} rad/s\n")

    vx = 0.0
    vy = 0.0
    wz = 0.0

    try:
        while rclpy.ok():
            key = get_key(orig_settings, timeout=0.1)

            if key in MOVE_BINDINGS:
                x_dir, y_dir, th_dir = MOVE_BINDINGS[key]
                vx = x_dir * node.speed
                vy = y_dir * node.speed
                wz = th_dir * node.turn
                node.publish_twist(vx, vy, wz)
                print(f"\rCmd: vx={vx:+.2f} m/s, vy={vy:+.2f} m/s, wz={wz:+.2f} rad/s   ", end="", flush=True)

            elif key in SPEED_BINDINGS:
                s_factor, t_factor = SPEED_BINDINGS[key]
                node.speed = max(0.05, min(2.0, node.speed * s_factor))
                node.turn = max(0.1, min(4.0, node.turn * t_factor))
                print(f"\rUpdated: speed = {node.speed:.2f} m/s | turn = {node.turn:.2f} rad/s   ", end="", flush=True)

            elif key in (' ', 'x'):
                vx, vy, wz = 0.0, 0.0, 0.0
                node.publish_twist(0.0, 0.0, 0.0)
                print("\rSTOPPED (0.0, 0.0, 0.0)                                  ", end="", flush=True)

            elif key in ('\x03', 'k', 'K'):  # Ctrl-C or k to quit
                print("\nExiting teleoperation...")
                break

            rclpy.spin_once(node, timeout_sec=0.0)

    except Exception as e:
        print(f"\nTeleop error: {e}")
    finally:
        # Publish zero velocity on exit to stop the robot safely
        try:
            node.publish_twist(0.0, 0.0, 0.0)
        except Exception:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
