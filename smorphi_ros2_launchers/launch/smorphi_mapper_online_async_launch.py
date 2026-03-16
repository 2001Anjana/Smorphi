import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    waypoint_file = LaunchConfiguration('waypoint_file')

    pkg_share = get_package_share_directory('smorphi_ros2_launchers')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_share, 'config', 'smorphi_mapper_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_waypoint_file_cmd = DeclareLaunchArgument(
        'waypoint_file',
        default_value=os.path.join(pkg_share, 'params', 'smorphi_waypoints.yaml'),
        description='Full path to the waypoint YAML file to save labeled locations')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    # Waypoint Manager — click on the map in RViz and label locations during mapping
    start_waypoint_manager_node = Node(
        package='smorphi_ros2_launchers',
        executable='save_waypoint',
        name='waypoint_manager',
        parameters=[{'waypoint_file': waypoint_file}],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_waypoint_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(start_waypoint_manager_node)

    return ld
