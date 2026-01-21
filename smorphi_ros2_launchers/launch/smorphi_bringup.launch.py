from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # --- LiDAR launch arguments (so bringup uses the correct port) ---
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    lidar_scan_mode = LaunchConfiguration('lidar_scan_mode')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0',
        description='RPLidar serial port'
    )

    declare_lidar_baud = DeclareLaunchArgument(
        'lidar_baud',
        default_value='115200',
        description='RPLidar serial baudrate'
    )

    declare_lidar_scan_mode = DeclareLaunchArgument(
        'lidar_scan_mode',
        default_value='Sensitivity',   # change to 'Standard' if needed
        description='RPLidar scan mode (e.g., Standard, Sensitivity)'
    )

    declare_lidar_frame_id = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='laser_link',
        description='RPLidar frame id'
    )

    # --- Include paths ---
    smorphi_controller_launch = os.path.join(
        get_package_share_directory('smorphi_ros2_controller'),
        'launch',
        'smorphi_controller.launch.py'
    )

    mpu_launch = os.path.join(
        get_package_share_directory('mpu9250'),
        'launch',
        'mpu9250.launch.py'
    )

    ekf_launch = os.path.join(
        get_package_share_directory('robot_localization'),
        'launch',
        'ekf.launch.py'
    )

    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a1_launch.py'
    )

    gap_launch = os.path.join(
        get_package_share_directory('lidar_gap_measure'),
        'launch',
        'gap_width.launch.py'
    )

    return LaunchDescription([
        # Declare LiDAR args
        declare_lidar_port,
        declare_lidar_baud,
        declare_lidar_scan_mode,
        declare_lidar_frame_id,

        # Controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(smorphi_controller_launch)
        ),

        # IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mpu_launch)
        ),

        # EKF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch)
        ),

        # LiDAR (pass args into rplidar_a1_launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            launch_arguments={
                'serial_port': lidar_port,
                'serial_baudrate': lidar_baud,
                'scan_mode': lidar_scan_mode,
                'frame_id': lidar_frame_id,
            }.items()
        ),

	IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gap_launch)
        ),
    ])


