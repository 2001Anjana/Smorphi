#!/usr/bin/env python3
#
# Waypoint Navigation Launch File
# Launches Nav2 + RViz for navigation (waypoints are loaded by go_to_waypoint separately)
#

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    map_yaml_file = LaunchConfiguration(
        'map_yaml_file',
        default=PathJoinSubstitution(
            [FindPackageShare('smorphi_ros2_launchers'), 'map', 'big_map1.yaml']
        )
    )

    params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution(
            [FindPackageShare('smorphi_ros2_launchers'), 'params', 'smorphi.yaml']
        )
    )

    default_bt_xml_filename = PathJoinSubstitution(
        [
            FindPackageShare('nav2_bt_navigator'),
            'behavior_trees',
            'navigate_w_replanning_and_recovery.xml'
        ]
    )

    nav2_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch']
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('smorphi_ros2_launchers'), 'rviz', 'navigation2.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz', default_value='true',
            description='Whether to launch RViz2'),

        DeclareLaunchArgument(
            'use_sim', default_value='false',
            description='Use simulation time'),

        DeclareLaunchArgument(
            'map_yaml_file', default_value=map_yaml_file,
            description='Full path to map YAML file'),

        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='Full path to Nav2 params file'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically start Nav2 stack'),

        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Use composed bringup'),

        DeclareLaunchArgument(
            'use_respawn', default_value='false',
            description='Respawn nodes on crash'),

        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': LaunchConfiguration('use_sim'),
                'params_file': params_file,
                'default_bt_xml_filename': default_bt_xml_filename,
                'autostart': LaunchConfiguration('autostart'),
                'use_composition': LaunchConfiguration('use_composition'),
                'use_respawn': LaunchConfiguration('use_respawn'),
            }.items(),
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_rviz'))),
    ])

