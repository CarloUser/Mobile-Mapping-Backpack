from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='/data/bags/mmb_recording',
        description='Full output path for the rosbag2 file (no extension).'
                    ' Use format: /data/bags/mmb_YYYYMMDD_HHMM_<location>_<run_type>_<run_number>',
    )

    # Load topic list from config — passed directly as CLI args
    # Topics are kept in config/topics.yaml for easy editing; we inline them
    # here because ros2 bag record does not accept a topics yaml directly.
    topics = [
        '/lidar/points',
        '/oak4d/rgb/image_raw',
        '/oak4d/depth/image_raw',
        '/oak4d/rgb/camera_info',
        '/oak4d/imu/data',
        '/oakd_lite/rgb/image_raw',
        '/oakd_lite/depth/image_raw',
        '/oakd_lite/rgb/camera_info',
        '/realsense/color/image_raw',
        '/realsense/depth/image_rect_raw',
        '/realsense/color/camera_info',
        '/imu/data',
        '/imu/mag',
        '/gnss/fix',
        '/gnss/fix_velocity',
        '/tf',
        '/tf_static',
    ]

    record_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'mcap',
            '--output', LaunchConfiguration('bag_name'),
            '--compression-mode', 'none',   # change to 'file' to enable LZ4 compression
        ] + topics,
        output='screen',
        shell=False,
    )

    return LaunchDescription([bag_name_arg, record_process])
