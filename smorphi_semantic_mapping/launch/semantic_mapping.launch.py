import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    image_topic = LaunchConfiguration('image_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    start_camera = LaunchConfiguration('start_camera')
    video_device = LaunchConfiguration('video_device')

    # Weights are bundled in the package and installed to its share dir, so the
    # launch works straight after `colcon build` with no manual file copying.
    default_model = os.path.join(
        get_package_share_directory('smorphi_semantic_mapping'), 'models', 'best.pt')

    declare_model = DeclareLaunchArgument(
        'model_path',
        default_value=default_model,
        description='Path to your trained YOLOv8 .pt weights')

    declare_image = DeclareLaunchArgument(
        'image_topic', default_value='/image_raw',
        description='Camera image topic the detector subscribes to')

    declare_scan = DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='LiDAR scan topic used to range detections')

    declare_start_camera = DeclareLaunchArgument(
        'start_camera', default_value='true',
        description='Also launch a v4l2 camera node (set false if camera already running)')

    declare_video_device = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='V4L2 device for the camera node')

    # Optional camera driver. Requires: sudo apt install ros-humble-v4l2-camera
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        condition=IfCondition(start_camera),
        parameters=[{'video_device': video_device}],
        remappings=[('/image_raw', image_topic)],
        output='screen',
    )

    semantic_mapper = Node(
        package='smorphi_semantic_mapping',
        executable='semantic_mapper',
        name='semantic_mapper',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'image_topic': image_topic,
            'scan_topic': scan_topic,
            'map_frame': 'map',
            'horizontal_fov_deg': 70.0,
            'camera_yaw_offset': 0.0,
            'conf_threshold': 0.6,     # model precision is high; 0.5-0.6 is safe
            # Model classes are Machine_01 and Machine_02. Empty list maps both.
            # To map only one, e.g. ['Machine_01'].
            'class_allowlist': [''],
            'fusion_method': 'lidar',  # 'lidar' (recommended) or 'pose'
            # Machines are large; widen the merge radius so views of one machine
            # from different angles collapse to a single landmark. Lower it if you
            # have several units of the same class placed close together.
            'merge_radius': 1.2,
            'max_fusion_range': 8.0,
        }],
    )

    return LaunchDescription([
        declare_model,
        declare_image,
        declare_scan,
        declare_start_camera,
        declare_video_device,
        camera_node,
        semantic_mapper,
    ])
