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
                {'front_gap_topic': 'front_gap_min'},
                {'back_gap_topic': 'back_gap_min'},     # NEW: gap width behind
                {'front_dist_topic': 'front_distance'}, # STOP guard only
                {'clearance_topic': 'left_back_clearance_min'},  # NEW: clearance
                {'shape_topic': 'shape_need'},
                {'cmd_vel_topic': 'cmd_vel'},
                {'blocked_topic': 'path_blocked'},

                {'threshold': 0.50},            # O->I : front gap < 0.50 m
                {'front_clear_gap': 0.50},      # I->O : front gap >= 0.50 m
                {'back_clear_gap': 0.60},       # I->O : back gap  >= 0.60 m (escaped)
                {'required_clearance': 0.40},   # both : left-back disc >= 0.40 m clear
                {'front_stop_distance': 0.15},  # STOP : wall within 0.15 m ahead
                {'stable_samples': 5},
                {'cooldown_s': 2.0},
                {'publish_stop_cmd_vel': True}, # set False if nav2 owns cmd_vel
            ],
        )
    ])