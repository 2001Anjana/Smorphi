#!/usr/bin/env python3
"""
Waypoint Commander Node
Navigates the robot to labeled waypoints using Nav2.
Reads waypoints from ~/smorphi_waypoints.yaml (persists across colcon build)
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


# Default path: ~/smorphi_waypoints.yaml (matches waypoint_manager save location)
DEFAULT_WAYPOINT_FILE = os.path.expanduser('~/smorphi_waypoints.yaml')


class WaypointCommander(Node):
    def __init__(self):
        super().__init__('waypoint_commander')

        # Declare parameters
        self.declare_parameter('waypoint_file', DEFAULT_WAYPOINT_FILE)
        self.declare_parameter('label', '')

        self.waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value
        self.label = self.get_parameter('label').get_parameter_value().string_value

        # Initialize Nav2 navigator
        self.navigator = BasicNavigator()

        self.get_logger().info(f'Waypoint Commander started.')
        self.get_logger().info(f'Loading waypoints from: {self.waypoint_file}')

        # Load waypoints
        self.waypoints = self.load_waypoints()

        if not self.waypoints:
            self.get_logger().error(
                'No waypoints found. Run the waypoint_manager first to save some locations.'
            )
            return

        # If label provided as parameter, navigate immediately; else go interactive
        if self.label:
            self.navigate_to_label(self.label)
        else:
            self.interactive_mode()

    def load_waypoints(self):
        """Load waypoints from YAML file."""
        if os.path.exists(self.waypoint_file):
            try:
                with open(self.waypoint_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data is None:
                        return {}
                    self.get_logger().info(
                        f'Loaded {len(data)} waypoints: {list(data.keys())}'
                    )
                    return data
            except Exception as e:
                self.get_logger().error(f'Error loading waypoints: {e}')
                return {}
        else:
            self.get_logger().error(f'Waypoint file not found: {self.waypoint_file}')
            return {}

    def navigate_to_label(self, label):
        """Navigate to the waypoint with the given label."""
        if label not in self.waypoints:
            self.get_logger().error(
                f'Waypoint "{label}" not found. '
                f'Available: {list(self.waypoints.keys())}'
            )
            return False

        waypoint = self.waypoints[label]

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        goal_pose.pose.position.x = waypoint['position']['x']
        goal_pose.pose.position.y = waypoint['position']['y']
        goal_pose.pose.position.z = waypoint['position']['z']

        goal_pose.pose.orientation.x = waypoint['orientation']['x']
        goal_pose.pose.orientation.y = waypoint['orientation']['y']
        goal_pose.pose.orientation.z = waypoint['orientation']['z']
        goal_pose.pose.orientation.w = waypoint['orientation']['w']

        self.get_logger().info(
            f'Navigating to "{label}" at '
            f'({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})'
        )

        # Wait for Nav2 to be active
        self.navigator.waitUntilNav2Active()

        # Send goal
        self.navigator.goToPose(goal_pose)

        # Wait for result
        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Successfully reached "{label}"!')
            return True
        else:
            self.get_logger().error(f'Failed to reach "{label}". Result: {result}')
            return False

    def interactive_mode(self):
        """Continuously accept waypoint labels from the terminal."""
        self.get_logger().info('Available waypoints:')
        for label in self.waypoints.keys():
            self.get_logger().info(f'  - {label}')

        while rclpy.ok():
            try:
                label = input(
                    '\nEnter waypoint label to navigate to (or "list" / "quit"): '
                ).strip()

                if label.lower() == 'quit':
                    break
                elif label.lower() == 'list':
                    print('Available waypoints:', list(self.waypoints.keys()))
                elif label:
                    self.navigate_to_label(label)

            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.lifecycleShutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
