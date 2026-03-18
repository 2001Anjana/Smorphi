#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from PIL import Image

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class SimLidarNode(Node):
    """
    Simulated 2D LiDAR.
    Loads a PGM map and raycasts from the robot's /odom pose.
    Publishes /scan.
    """
    def __init__(self):
        super().__init__('sim_lidar_node')

        self.declare_parameter('map_file', 'map/sim_map.pgm')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('num_rays', 360)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('range_min', 0.15)

        map_file = self.get_parameter('map_file').value
        self.resolution = self.get_parameter('resolution').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        
        self.num_rays = self.get_parameter('num_rays').value
        self.range_max = self.get_parameter('range_max').value
        self.range_min = self.get_parameter('range_min').value

        # Load map
        try:
            img = Image.open(map_file)
            # Convert to numpy array
            self.map_data = np.array(img)
            self.map_h, self.map_w = self.map_data.shape
            self.get_logger().info(f"Loaded map {self.map_w}x{self.map_h} from {map_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            self.map_data = None

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.scan_pub = self.create_publisher(LaserScan, self.get_parameter('scan_topic').value, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        timer_period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(timer_period, self.publish_scan)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def world_to_map(self, wx, wy):
        """Convert world coordinates to map pixel coordinates."""
        mx = int((wx - self.origin_x) / self.resolution)
        my = self.map_h - 1 - int((wy - self.origin_y) / self.resolution)
        return mx, my

    def is_occupied(self, mx, my):
        """Check if map pixel is occupied (black/dark)."""
        if mx < 0 or mx >= self.map_w or my < 0 or my >= self.map_h:
            return True # out of bounds is blocked
        # PGM: 254 is free, 0 is occupied. Threshold around 200
        return self.map_data[my, mx] < 200

    def raycast(self, angle):
        """Simple DDA or step-based raycast."""
        step_size = self.resolution / 2.0
        max_steps = int(self.range_max / step_size)
        
        dx = math.cos(angle) * step_size
        dy = math.sin(angle) * step_size
        
        cx = self.robot_x
        cy = self.robot_y
        
        distance = 0.0
        
        for _ in range(max_steps):
            cx += dx
            cy += dy
            distance += step_size
            
            mx, my = self.world_to_map(cx, cy)
            if self.is_occupied(mx, my):
                return distance
                
        return float('inf')

    def publish_scan(self):
        if self.map_data is None:
            return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / self.num_rays
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        ranges = []
        for i in range(self.num_rays):
            angle = scan.angle_min + i * scan.angle_increment + self.robot_yaw
            # normalize angle
            angle = math.atan2(math.sin(angle), math.cos(angle))
            r = self.raycast(angle)
            if r < self.range_min or r > self.range_max:
                ranges.append(float('inf'))
            else:
                ranges.append(r)
                
        scan.ranges = ranges
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = SimLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
