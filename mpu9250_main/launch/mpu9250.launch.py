import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mpu9250",
            executable="mpu9250",
            name="mpu9250",
            output="screen",

            # ✅ remap so controller_node + ekf_filter_node receive IMU on /imu_data
            remappings=[
                ("/imu", "/imu_data"),
            ],

           parameters=[
                {"acceleration_scale": [1.000000, 1.000000, 1.000000], 
                "acceleration_bias": [0.102924, 0.005753, 0.094218], 
                "gyro_bias": [-0.453430, 3.143407, -1.127055], 
                "magnetometer_bias": [28.998047, 113.625000, 21.671875], 
                "magnetometer_transform": [
                            1.199457, 0.000000, 0.000000,
                            0.000000, 0.803208, 0.000000,
                            0.000000, 0.000000, 1.085444
                ]}  
                ],
        )
    ])
