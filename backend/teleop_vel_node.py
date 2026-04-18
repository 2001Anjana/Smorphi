#!/usr/bin/env python3
"""
teleop_vel_node.py
------------------
A persistent ROS2 node that publishes cmd_vel at 10 Hz.
Velocity is updated via a simple HTTP-like stdin protocol:
  Each line on stdin: "linear_x,linear_y,angular_z\n"

Run with:
  source /opt/ros/humble/setup.bash
  source ~/smorphi_ws/install/setup.bash
  python3 teleop_vel_node.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import threading


class TeleopVelNode(Node):
    def __init__(self):
        super().__init__('teleop_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_vel)  # 10 Hz
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self._lock = threading.Lock()
        self.get_logger().info('TeleopVelNode started, reading velocity from stdin.')

        # Thread to read velocity updates from stdin
        self._stdin_thread = threading.Thread(target=self._read_stdin, daemon=True)
        self._stdin_thread.start()

    def _read_stdin(self):
        """Read velocity commands from stdin. Format: linear_x,linear_y,angular_z"""
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                parts = line.split(',')
                lx = float(parts[0])
                ly = float(parts[1])
                az = float(parts[2])
                with self._lock:
                    self.linear_x = lx
                    self.linear_y = ly
                    self.angular_z = az
            except Exception:
                pass  # Ignore malformed lines

    def publish_vel(self):
        msg = Twist()
        with self._lock:
            msg.linear.x = self.linear_x
            msg.linear.y = self.linear_y
            msg.angular.z = self.angular_z
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = TeleopVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
