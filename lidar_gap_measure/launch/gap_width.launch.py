from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Continuous gap sweep (front-region gap + back-region gap + distances)
        Node(
            package='lidar_gap_measure',
            executable='continuous_gap_node',
            name='continuous_gap_node',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                # Sweep must reach 0.40 m ahead and 0.60 m behind.
                {'x_start': 0.40},
                {'x_end': -0.60},
                {'x_step': 0.05},
                {'x_tol': 0.05},
                # FRONT region that decides O->I (0 .. 40 cm ahead).
                {'front_x_min': 0.00},
                {'front_x_max': 0.40},
                # BACK region that decides the I->O escape (5 .. 60 cm behind).
                {'back_x_min': -0.60},
                {'back_x_max': -0.05},
                # LEFT->BACK clearance quarter-disc (gate for O<->I transforms).
                {'clearance_center_back': 0.10},   # center 10 cm behind lidar
                {'clearance_radius': 0.40},        # 40 cm quarter-disc radius
            ],
        ),
    ])