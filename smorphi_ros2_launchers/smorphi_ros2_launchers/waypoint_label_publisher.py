#!/usr/bin/env python3
"""
Waypoint Label Publisher Node
Reads saved waypoints from ~/smorphi_waypoints.yaml and publishes
MarkerArray on /waypoints so labels appear on the RViz map during navigation.
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


# Default path: ~/smorphi_waypoints.yaml (matches waypoint_manager save location)
DEFAULT_WAYPOINT_FILE = os.path.expanduser('~/smorphi_waypoints.yaml')


class WaypointLabelPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_label_publisher')

        # Declare parameters
        self.declare_parameter('waypoint_file', DEFAULT_WAYPOINT_FILE)

        self.waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value

        # Publisher for visualization markers in RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 10)

        self.get_logger().info(f'Waypoint Label Publisher started.')
        self.get_logger().info(f'Loading waypoints from: {self.waypoint_file}')

        # Load waypoints
        self.waypoints = self.load_waypoints()

        if not self.waypoints:
            self.get_logger().warn(
                'No waypoints found. If you saved waypoints during mapping, '
                'make sure the file exists at: ' + self.waypoint_file
            )
        else:
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints: {list(self.waypoints.keys())}'
            )

        # Timer to periodically publish markers (so RViz picks them up)
        self.marker_timer = self.create_timer(2.0, self.publish_markers)

    def load_waypoints(self):
        """Load waypoints from YAML file."""
        if os.path.exists(self.waypoint_file):
            try:
                with open(self.waypoint_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data is None:
                        return {}
                    return data
            except Exception as e:
                self.get_logger().error(f'Error loading waypoints: {e}')
                return {}
        else:
            self.get_logger().error(f'Waypoint file not found: {self.waypoint_file}')
            return {}

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


def main(args=None):
    rclpy.init(args=args)
    node = WaypointLabelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
