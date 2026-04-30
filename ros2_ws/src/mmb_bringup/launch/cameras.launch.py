from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer
from launch.actions import GroupAction


def generate_launch_description():
    # OAK-4D Luxonis — uses depthai-ros v3 API
    # Run under /oak4d namespace so topics don't collide with OAK-D Lite
    oak4d_group = GroupAction([
        PushRosNamespace('oak4d'),
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_model': 'OAK-4',
                'i_pipeline_type': 'RGBD',
                'i_nn_type': 'none',
            }],
        ),
    ])

    # OAK-D Lite — standard depthai-ros
    oakd_lite_group = GroupAction([
        PushRosNamespace('oakd_lite'),
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_model': 'OAK-D-LITE',
                'i_pipeline_type': 'RGBD',
                'i_nn_type': 'none',
            }],
        ),
    ])

    # Intel RealSense — realsense2_camera
    # Adjust serial_no if you have multiple RealSense units
    realsense_group = GroupAction([
        PushRosNamespace('realsense'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            output='screen',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'color_fps': 30,
                'depth_fps': 30,
                'align_depth.enable': False,
                'serial_no': '',   # leave empty to use first detected unit
            }],
        ),
    ])

    return LaunchDescription([
        oak4d_group,
        oakd_lite_group,
        realsense_group,
    ])
