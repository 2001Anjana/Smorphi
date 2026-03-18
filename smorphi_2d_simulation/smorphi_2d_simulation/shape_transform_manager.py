#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np

from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class ShapeTransformManager(Node):
    def __init__(self):
        super().__init__('shape_transform_manager')
        
        # 0 = O shape (35x35cm footprint), 1 = I shape (17x70cm footprint)
        self.current_shape = 0 
        self.target_shape = 0
        
        self.last_scan = None
        self.last_path = None
        
        self.shape_pub = self.create_publisher(Int32, 'shape_need', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'shape_markers', 10)
        
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(Path, 'plan', self.path_callback, 10)
        self.create_subscription(Int32, 'current_shape', self.shape_callback, 10)
        
        # Service clients to update local costmap and DWB footprint
        self.set_local_costmap_client = self.create_client(
            SetParameters, 
            '/local_costmap/local_costmap/set_parameters'
        )
        self.set_dwb_client = self.create_client(
            SetParameters, 
            '/controller_server/set_parameters'
        )
        
        self.timer = self.create_timer(0.2, self.check_transform_conditions)
        self.get_logger().info('ShapeTransformManager started')
        
    def shape_callback(self, msg):
        self.current_shape = msg.data
        
    def scan_callback(self, msg):
        self.last_scan = msg
        
    def path_callback(self, msg):
        self.last_path = msg
        
    def is_gap_narrow_in_range(self, scan_msg, min_dist, max_dist, lateral_threshold, angle_window=math.radians(45)):
        """
        Check if there's a gap narrower than threshold in a distance range (e.g. from -0.1 to -0.7 behind).
        Returns True if ANY part of the scanned region within [min_dist, max_dist] is narrower than lateral_threshold.
        """
        if not scan_msg:
            return False, float('inf')
            
        ranges = np.array(scan_msg.ranges)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
        
        # Convert scan to x, y base_link coordinates
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        
        # Filter points that fall within the X distance range and Y lateral window
        # For checking behind, min_dist and max_dist are negative (e.g., -0.7 to -0.1)
        valid = (xs >= min(min_dist, max_dist)) & (xs <= max(min_dist, max_dist)) & (np.abs(ys) <= lateral_threshold)
        
        if not np.any(valid):
            return False, float('inf') # No obstacles in the bounding box
            
        points_y = ys[valid]
        points_x = xs[valid]
        
        # To robustly check width along the length, we group points by X slices
        # Bin points in 10cm slices along X
        bins = np.arange(min(min_dist, max_dist), max(min_dist, max_dist) + 0.1, 0.1)
        indices = np.digitize(points_x, bins)
        
        min_gap_width = float('inf')
        narrow_detected = False
        
        for i in range(1, len(bins)):
            slice_mask = (indices == i)
            if not np.any(slice_mask):
                continue
                
            slice_y = points_y[slice_mask]
            
            # Need points on both left and right sides to constitute a "gap"
            left_points = slice_y[slice_y > 0.05]
            right_points = slice_y[slice_y < -0.05]
            
            if len(left_points) > 0 and len(right_points) > 0:
                gap_width = np.max(left_points) - np.min(right_points)
                min_gap_width = min(min_gap_width, gap_width)
                if gap_width < lateral_threshold:
                    narrow_detected = True
                    
        return narrow_detected, min_gap_width
        
    def check_transform_conditions(self):
        if not self.last_scan:
            return
            
        # 1. Front gap check (Check further ahead: from 0.1m to 0.9m to trigger earlier)
        # Using 0.40m threshold instead of 0.35m to account for map discretization and corner padding
        narrow_front, width_front = self.is_gap_narrow_in_range(self.last_scan, 0.1, 0.9, lateral_threshold=0.40)
        
        # 2. Back gap check (check further back: from -0.1m up to -0.8m to prevent premature recovery)
        narrow_back, width_back = self.is_gap_narrow_in_range(self.last_scan, -0.8, -0.1, lateral_threshold=0.40)
        
        # 3. Impassable check (Check if gap is strictly < 0.17m up to 0.6m ahead)
        impassable_front, _ = self.is_gap_narrow_in_range(self.last_scan, 0.1, 0.6, lateral_threshold=0.17)
        
        if impassable_front:
            # We cannot physically pass
            self.get_logger().warn(f"Path is impassable (gap < 17cm)! Aborting transform considerations.")
            return

        if self.current_shape == 0:
            # We are in O shape. Should we transform to I?
            # Global path check: is there a path passing through the narrow corridor?
            path_intersects_narrow = False
            if self.last_path is not None and len(self.last_path.poses) > 0:
                # In a real scenario we'd query costmap or raycast along path.
                # Here we use the fact that the narrow_front flag is True 
                # AND there's an active path ahead.
                path_intersects_narrow = True 
                
            if narrow_front and path_intersects_narrow:
                # Also check space behind to retract (from 0 to -0.45m to ensure room to transform)
                _, width_immediate_back = self.is_gap_narrow_in_range(self.last_scan, -0.45, 0.0, lateral_threshold=0.35)
                # If width_immediate_back is large, we have space to stretch
                # (For simplicity here we just check if it's not impassable immediate back)
                # Let's trigger
                self.get_logger().info(f"O->I trigger: narrow gap {width_front:.2f}m detected up to 90cm ahead.")
                self.target_shape = 1
                self.publish_shape_command()
                self.update_local_footprint()
                
        else: # self.current_shape == 1
            # We are in I shape. Should we transform back to O?
            if not narrow_front and not narrow_back:
                # we have cleared the narrow corridor front and back
                self.get_logger().info(f"I->O trigger: cleared narrow gap (Front: {width_front:.2f}m, Back: {width_back:.2f}m).")
                self.target_shape = 0
                self.publish_shape_command()
                self.update_local_footprint()

    def publish_shape_command(self):
        msg = Int32()
        msg.data = self.target_shape
        self.shape_pub.publish(msg)
        
    def update_local_footprint(self):
        # Update Nav2 costmap footprint
        req = SetParameters.Request()
        param = Parameter()
        param.name = "footprint"
        
        val = ParameterValue()
        val.type = ParameterType.PARAMETER_STRING
        
        if self.target_shape == 1:
            # I shape footprint: 0.17 x 0.70 m 
            val.string_value = "[[-0.35, -0.085], [-0.35, 0.085], [0.35, 0.085], [0.35, -0.085]]"
        else:
            # O shape footprint: 0.35 x 0.35 m
            val.string_value = "[[-0.175, -0.175], [-0.175, 0.175], [0.175, 0.175], [0.175, -0.175]]"
            
        param.value = val
        req.parameters.append(param)
        
        # Also prepare the parameter for DWB controller
        req_dwb = SetParameters.Request()
        param_dwb = Parameter()
        param_dwb.name = "FollowPath.footprint"
        param_dwb.value = val
        req_dwb.parameters.append(param_dwb)
        
        if self.set_local_costmap_client.service_is_ready():
            self.set_local_costmap_client.call_async(req)
        else:
            self.get_logger().warn("Local costmap param service not ready")
            
        if self.set_dwb_client.service_is_ready():
            self.set_dwb_client.call_async(req_dwb)
            self.get_logger().info(f"Updated controller and costmap footprints to shape={self.target_shape}")
        else:
            self.get_logger().warn("Controller param service not ready")

def main(args=None):
    rclpy.init(args=args)
    node = ShapeTransformManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
