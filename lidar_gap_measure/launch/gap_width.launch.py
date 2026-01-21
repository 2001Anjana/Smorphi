from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_gap_measure',
            executable='gap_width_node',
            name='gap_width_node',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                {'target_x': 0.60},
                {'x_tol': 0.05},
            ],
        )
    ])
