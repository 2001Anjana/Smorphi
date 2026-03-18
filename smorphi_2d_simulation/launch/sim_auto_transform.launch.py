#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_smorphi_2d_simulation = get_package_share_directory('smorphi_2d_simulation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Paths
    map_yaml_file = os.path.join(pkg_smorphi_2d_simulation, 'map', 'sim_map.yaml')
    nav2_params_file = os.path.join(pkg_smorphi_2d_simulation, 'config', 'sim_nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_smorphi_2d_simulation, 'rviz', 'sim_auto_transform.rviz')
    bt_xml = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Simulation nodes
    sim_robot_cmd = Node(
        package='smorphi_2d_simulation',
        executable='sim_robot_node',
        name='sim_robot_node',
        output='screen',
        parameters=[
            {'init_x': 0.5},
            {'init_y': 0.5},
            {'init_yaw': 0.0}
        ]
    )

    sim_lidar_cmd = Node(
        package='smorphi_2d_simulation',
        executable='sim_lidar_node',
        name='sim_lidar_node',
        output='screen',
        parameters=[
            {'map_file': os.path.join(pkg_smorphi_2d_simulation, 'map', 'sim_map.pgm')},
            {'resolution': 0.05},
            {'publish_rate': 10.0}
        ]
    )

    shape_transform_cmd = Node(
        package='smorphi_2d_simulation',
        executable='shape_transform_manager',
        name='shape_transform_manager',
        output='screen'
    )

    # Nav2 Bringup
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart': 'True',
            'default_bt_xml_filename': bt_xml
        }.items()
    )

    # RViz2
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Static TF map->odom (If we rely on Nav2 AMCL, it should provide map->odom. 
    # But since sim_lidar sends scan from base_link and map_server needs origin... 
    # Actually wait, we should run AMCL. bringup_launch.py runs AMCL.)
    
    # We provide a fake map->odom transform initially so AMCL can localize
    initial_pose_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='initial_pose_tf',
        arguments=['0.5', '0.5', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )

    ld = LaunchDescription()

    ld.add_action(sim_robot_cmd)
    ld.add_action(sim_lidar_cmd)
    ld.add_action(shape_transform_cmd)
    ld.add_action(initial_pose_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(rviz_cmd)

    return ld
