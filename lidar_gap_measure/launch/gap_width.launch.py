from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ── Front gap: slice at 20 cm AHEAD of LiDAR ──
        Node(
            package='lidar_gap_measure',
            executable='gap_width_node',
            name='gap_width_node',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                {'target_x': 0.20},
                {'x_tol': 0.05},
            ],
        ),

        # ── Back gap: slice at 60 cm BEHIND LiDAR ──
        Node(
            package='lidar_gap_measure',
            executable='gap_width_node',
            name='gap_width_back_node',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                {'target_x': -0.60},
                {'x_tol': 0.05},
            ],
            remappings=[
                ('gap_width', 'gap_width_back'),
                ('gap_marker', 'gap_marker_back'),
                ('gap_text', 'gap_text_back'),
            ],
        ),
    ])
