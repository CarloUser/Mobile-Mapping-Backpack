from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import time

import yaml


def generate_launch_description():
    default_name = time.strftime('/data/bags/mmb_%Y%m%d_%H%M')
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value=default_name,
        description='Output path for the rosbag2 (no extension). Convention: '
                    '/data/bags/mmb_YYYYMMDD_HHMM_<location>_<run_type>_<run_number>',
    )

    # Single source of truth for the topic list: config/topics.yaml.
    topics_yaml = os.path.join(
        get_package_share_directory('mmb_bringup'), 'config', 'topics.yaml')
    with open(topics_yaml) as f:
        topics = yaml.safe_load(f)['topics']

    record_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'mcap',
            '--output', LaunchConfiguration('bag_name'),
            '--compression-mode', 'none',   # 'file' enables LZ4 if storage-bound
        ] + topics,
        output='screen',
        shell=False,
    )

    return LaunchDescription([bag_name_arg, record_process])
