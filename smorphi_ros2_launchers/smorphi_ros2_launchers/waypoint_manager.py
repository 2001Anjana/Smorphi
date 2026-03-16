#!/usr/bin/env python3
"""
Waypoint Manager Node
Allows user to save labeled waypoints by clicking on the map in RViz.
Waypoints are saved to ~/smorphi_waypoints.yaml (persists across colcon build)
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from threading import Thread


# Default path: ~/smorphi_waypoints.yaml (safe from colcon build overwrites)
DEFAULT_WAYPOINT_FILE = os.path.expanduser('~/smorphi_waypoints.yaml')


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        # Declare parameters
        self.declare_parameter('waypoint_file', DEFAULT_WAYPOINT_FILE)

        self.waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value

        # Subscribe to clicked point from RViz
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        # TF buffer (available for future use, e.g. saving robot's current pose)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'Waypoint Manager started.')
        self.get_logger().info(f'Waypoints will be saved to: {self.waypoint_file}')
        self.get_logger().info('In RViz, use the "Publish Point" tool and click on the map to save a waypoint.')

        # Clear old waypoints — new mapping session means new coordinates
        if os.path.exists(self.waypoint_file):
            os.remove(self.waypoint_file)
            self.get_logger().info('Cleared old waypoints file (new mapping session).')

        self.waypoints = {}

    def load_waypoints(self):
        """Load waypoints from YAML file if it exists."""
        if os.path.exists(self.waypoint_file):
            try:
                with open(self.waypoint_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data is None:
                        return {}
                    self.get_logger().info(f'Loaded {len(data)} existing waypoints: {list(data.keys())}')
                    return data
            except Exception as e:
                self.get_logger().error(f'Error loading waypoints: {e}')
                return {}
        else:
            self.get_logger().info('No existing waypoint file found. Starting fresh.')
            return {}

    def save_waypoints(self):
        """Save waypoints to YAML file."""
        try:
            os.makedirs(os.path.dirname(self.waypoint_file), exist_ok=True)
            with open(self.waypoint_file, 'w') as f:
                yaml.dump(self.waypoints, f, default_flow_style=False)
            self.get_logger().info(f'Waypoints saved to: {self.waypoint_file}')
        except Exception as e:
            self.get_logger().error(f'Error saving waypoints: {e}')

    def clicked_point_callback(self, msg):
        """Handle clicked point from RViz."""
        self.get_logger().info(
            f'Point clicked at x={msg.point.x:.3f}, y={msg.point.y:.3f}'
        )
        # Get label from user in a separate thread to avoid blocking the ROS spin
        thread = Thread(target=self.get_label_and_save, args=(msg,))
        thread.daemon = True
        thread.start()

    def get_label_and_save(self, msg):
        """Prompt user for a label and save the waypoint."""
        try:
            label = input('\nEnter label for this waypoint: ').strip()

            if not label:
                self.get_logger().warn('Empty label — waypoint not saved.')
                return

            waypoint_data = {
                'position': {
                    'x': float(msg.point.x),
                    'y': float(msg.point.y),
                    'z': float(msg.point.z)
                },
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'w': 1.0
                }
            }

            self.waypoints[label] = waypoint_data
            self.save_waypoints()
            self.get_logger().info(f'Waypoint "{label}" saved!')

        except Exception as e:
            self.get_logger().error(f'Error saving waypoint: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
