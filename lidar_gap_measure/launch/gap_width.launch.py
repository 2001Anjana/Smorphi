from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ── Continuous gap sweep: slices from +0.20 m to −0.60 m ──
        Node(
            package='lidar_gap_measure',
            executable='continuous_gap_node',
            name='continuous_gap_node',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                {'x_start': 0.20},
                {'x_end': -0.60},
                {'x_step': 0.05},
                {'x_tol': 0.05},
            ],
        ),
    ])
