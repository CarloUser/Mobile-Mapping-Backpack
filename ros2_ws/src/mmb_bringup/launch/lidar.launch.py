from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mmb_bringup'),
        'config', 'hesai_jt128.yaml'
    )

    hesai_node = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        output='screen',
        parameters=[config],
        remappings=[
            ('lidar_points', '/lidar/points'),
        ],
    )

    return LaunchDescription([hesai_node])
