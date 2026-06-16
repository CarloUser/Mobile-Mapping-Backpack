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

    # OAK-1 (rear, single 4K RGB — no stereo/depth) — depthai-ros driver.
    # NOTE: topic names (/oak1/rgb/image_raw, /oak1/rgb/camera_info) follow the
    # same convention as oakd_lite but were NOT yet confirmed against a live
    # device (none connected at config time). Confirm with `ros2 topic list`.
    # MULTI-OAK GOTCHA: with oak4d + oakd_lite + oak1 all on one host the driver
    # may grab the wrong device — set 'i_mx_id' (or 'i_usb_port_id') per node to
    # pin each to its serial if they race.
    oak1_group = GroupAction([
        PushRosNamespace('oak1'),
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_model': 'OAK-1',
                'i_pipeline_type': 'RGB',
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

    # Insta360 (omni hub camera) — driver node only: it publishes the
    # H.264-compressed dual-fisheye stream (/dual_fisheye/image/compressed)
    # + raw IMU, which is exactly what we record. The decoder /
    # equirectangular / Madgwick nodes from the driver's own
    # bringup.launch.xml are offline-processing tools; keeping them out of
    # the recording session saves Jetson CPU.
    insta360_node = Node(
        package='insta360_ros_driver',
        executable='insta360_ros_driver',
        name='insta360_ros_driver',
        output='log',
    )

    return LaunchDescription([
        oak4d_group,
        oakd_lite_group,
        oak1_group,
        realsense_group,
        insta360_node,
    ])
