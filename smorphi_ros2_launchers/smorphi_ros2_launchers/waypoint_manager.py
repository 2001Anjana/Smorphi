#!/usr/bin/env python3
"""
Waypoint Manager Node
Allows user to save labeled waypoints by clicking on the map in RViz.
Waypoints are saved to ~/smorphi_waypoints.yaml (persists across colcon build)
Publishes MarkerArray on /waypoints so labels appear on the RViz map.

Labels are auto-generated as wp_1, wp_2, etc.
To rename labels, edit ~/smorphi_waypoints.yaml after mapping.
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener


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

        # Publisher for visualization markers in RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 10)

        # TF buffer (available for future use, e.g. saving robot's current pose)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'Waypoint Manager started.')
        self.get_logger().info(f'Waypoints will be saved to: {self.waypoint_file}')
        self.get_logger().info(
            'In RViz, use the "Publish Point" tool and click on the map to save a waypoint.'
        )
        self.get_logger().info(
            'Labels are auto-generated (wp_1, wp_2, ...). '
            'To rename, edit the YAML file after mapping.'
        )

        # Clear old waypoints — new mapping session means new coordinates
        if os.path.exists(self.waypoint_file):
            os.remove(self.waypoint_file)
            self.get_logger().info('Cleared old waypoints file (new mapping session).')

        self.waypoints = {}
        self.waypoint_counter = 0

        # Timer to periodically re-publish markers (so RViz picks them up if started late)
        self.marker_timer = self.create_timer(2.0, self.publish_markers)

    def save_waypoints(self):
        """Save waypoints to YAML file."""
        try:
            # Ensure directory exists (handles ~/smorphi_waypoints.yaml case too)
            dir_path = os.path.dirname(self.waypoint_file)
            if dir_path:
                os.makedirs(dir_path, exist_ok=True)
            with open(self.waypoint_file, 'w') as f:
                yaml.dump(self.waypoints, f, default_flow_style=False)
            self.get_logger().info(f'Waypoints saved to: {self.waypoint_file}')
        except Exception as e:
            self.get_logger().error(f'Error saving waypoints: {e}')

    def publish_markers(self):
        """Publish all waypoints as MarkerArray for RViz visualization.

        Uses stable IDs (no DELETEALL) — same pattern as gap_width_node
        which displays correctly in RViz2.
        """
        if not self.waypoints:
            return

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for idx, (label, wp) in enumerate(self.waypoints.items()):
            # Sphere marker at waypoint position
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = now
            sphere.ns = 'waypoint_spheres'
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wp['position']['x']
            sphere.pose.position.y = wp['position']['y']
            sphere.pose.position.z = wp['position']['z'] + 0.05
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.15
            sphere.scale.y = 0.15
            sphere.scale.z = 0.15
            sphere.color.r = 0.0
            sphere.color.g = 1.0
            sphere.color.b = 0.0
            sphere.color.a = 1.0
            sphere.lifetime.sec = 0
            sphere.lifetime.nanosec = 0
            marker_array.markers.append(sphere)

            # Text marker with label name above the sphere
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = now
            text.ns = 'waypoint_labels'
            text.id = idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = wp['position']['x']
            text.pose.position.y = wp['position']['y']
            text.pose.position.z = wp['position']['z'] + 0.3
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2  # text height
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 0.0
            text.color.a = 1.0
            text.text = label
            text.lifetime.sec = 0
            text.lifetime.nanosec = 0
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)

    def clicked_point_callback(self, msg):
        """Handle clicked point from RViz — auto-generates label and saves immediately."""
        self.waypoint_counter += 1
        label = f'wp_{self.waypoint_counter}'

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
        self.get_logger().info(
            f'Waypoint "{label}" saved at x={msg.point.x:.3f}, y={msg.point.y:.3f}'
        )

        # Immediately publish markers so label appears on RViz right away
        self.publish_markers()


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
