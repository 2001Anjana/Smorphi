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
                {'back_gap_topic': '/gap_width_back'},
                {'shape_topic': 'shape_need'},
                {'threshold': 0.50},
                {'stable_samples': 5},
                {'cooldown_s': 2.0},
            ],
        )
    ])
