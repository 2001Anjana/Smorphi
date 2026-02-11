from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_gap_measure',
            executable='shape_auto_node',
            name='shape_auto_node',
            output='screen',
            parameters=[
                {'gap_topic': '/gap_width'},
                {'shape_topic': 'shape_need'},
                {'threshold_i': 0.30},
                {'threshold_o': 0.35},
                {'stable_samples': 5},
                {'cooldown_s': 2.0},
            ],
        )
    ])
