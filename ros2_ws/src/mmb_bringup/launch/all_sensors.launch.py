from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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

    # Static TF tree — placeholder transforms from base_link to each sensor.
    # Values below are identity (0 translation, no rotation) until Student 5
    # provides CAD-derived poses and Student 4 delivers calibrated extrinsics.
    # Format: x y z qx qy qz qw  parent  child  (all in metres)
    static_tfs = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_lidar',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'lidar_hesai'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_oak4d',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'camera_oak4d'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_oakd_lite',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'camera_oakd_lite'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_realsense',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'camera_realsense'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_xsens'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_gnss',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'gnss_antenna'],
        ),
    ]

    return LaunchDescription([
        lidar_launch,
        cameras_launch,
        imu_gnss_launch,
        *static_tfs,
    ])
