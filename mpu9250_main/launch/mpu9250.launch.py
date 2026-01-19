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
                {"acceleration_scale": [1.0072387165748442, 1.0081436035838134, 0.9932769089604535], 
                "acceleration_bias": [0.17038044467587418, 0.20464685207217453, -0.12461014438322202], 
                "gyro_bias": [0.0069376404996494, -0.0619247665634732, 0.05717760948453845], 
                "magnetometer_bias": [-32.68, 63.79, -5.984038606178013], 
                "magnetometer_transform": [   0.9983016121720226, 0.044890057238382707, 0.007231924972024632, 
                                    0.044890057238382707, 1.2981683205953654, -0.1173361838042438, 
                                    0.007231924972024633, -0.11733618380424381, 0.7835617468652673]}  
                ],
        )
    ])
