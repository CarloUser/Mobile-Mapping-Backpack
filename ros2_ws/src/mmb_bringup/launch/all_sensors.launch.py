from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('mmb_bringup')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'lidar.launch.py'))
    )
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'cameras.launch.py'))
    )
    imu_gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'imu_gnss.launch.py'))
    )
    static_extrinsics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'static_extrinsics.launch.py'))
    )

    return LaunchDescription([
        lidar_launch,
        cameras_launch,
        imu_gnss_launch,
        static_extrinsics_launch,
    ])
