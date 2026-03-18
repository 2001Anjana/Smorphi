#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class SimRobotNode(Node):
    """
    Simulated differential drive robot.
    Subscribes to:
        /cmd_vel (Twist)
        /shape_need (Int32)
    Publishes:
        /odom (Odometry)
        /current_shape (Int32)
    Broadcasts TF:
        odom -> base_link
    """
    def __init__(self):
        super().__init__('sim_robot_node')

        self.declare_parameter('init_x', 0.5)
        self.declare_parameter('init_y', 0.5)
        self.declare_parameter('init_yaw', 0.0)

        self.x = self.get_parameter('init_x').value
        self.y = self.get_parameter('init_y').value
        self.yaw = self.get_parameter('init_yaw').value

        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0

        # Start in O shape (0)
        self.current_shape = 0

        self.last_time = self.get_clock().now()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.shape_pub = self.create_publisher(Int32, 'current_shape', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32, 'shape_need', self.shape_need_callback, 10)

        # Timer to integrate odometry at 20Hz
        self.timer = self.create_timer(0.05, self.update_odom)
        self.get_logger().info(f"SimRobotNode started at x={self.x}, y={self.y}")

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vtheta = msg.angular.z

    def shape_need_callback(self, msg):
        requested_shape = msg.data
        if requested_shape != self.current_shape:
            self.get_logger().info(f"Transforming robot shape to: {'I' if requested_shape == 1 else 'O'}")
            self.current_shape = requested_shape

    def update_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Holonomic integration just in case
        delta_x = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        delta_y = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        delta_yaw = self.vtheta * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        quat = quaternion_from_euler(0, 0, self.yaw)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta

        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

        # Publish current shape
        shape_msg = Int32()
        shape_msg.data = self.current_shape
        self.shape_pub.publish(shape_msg)

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = SimRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
