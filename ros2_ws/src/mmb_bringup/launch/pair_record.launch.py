"""Bring up ONE camera<->LiDAR pair's drivers and record its bag, in one shot.

Used by bash/record_pair.sh (and the per-pair wrappers record_<pair>.sh) so the
whole recording step is a single command: connect the pair, run the script,
drivers start, and after a short warm-up the bag records the pair's three
topics. Ctrl-C tears down the recorder AND the drivers together (ros2 launch
owns all child processes).

Args:
  pair          one of oak4d, oak1, oakd_lite, realsense, insta360
  topics        space-separated topics to record (passed by record_pair.sh,
                read from extrinsic_calibration/camera_lidar/config.yaml so the
                recorded set never drifts from what the calibrator expects)
  bag_name      output path (no extension)
  record_delay  seconds to let drivers come up before recording starts

This file holds only the DRIVER mapping (which is launch's job); the topic list
stays in config.yaml. Driver definitions mirror cameras.launch.py /
lidar.launch.py one-for-one — keep them in sync.
"""
import os
import time

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            TimerAction, GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def _depthai(ns, model, pipeline):
    return GroupAction([
        PushRosNamespace(ns),
        Node(package='depthai_ros_driver', executable='camera_node',
             name='camera_node', output='screen',
             parameters=[{'camera_model': model,
                          'i_pipeline_type': pipeline,
                          'i_nn_type': 'none'}]),
    ])


def _oak4d_v3():
    # OAK-4D is RVC4/PoE -> the v2 depthai_ros_driver can't drive it. Use the
    # official depthai_ros_driver_v3 (>= 3.3.0; 3.2.1 SIGSEGVs vs core 3.7.1).
    # Run driver_node directly as a plain Node (the composable-container
    # driver.launch.py does not come up when nested in this launch). Node name
    # 'oak4d' (no namespace) -> the driver publishes /oak4d/rgb/image_raw +
    # /oak4d/rgb/camera_info, as config.yaml expects. Params (device id + RGB
    # pipeline) in config/oak4d_v3.yaml. The missing oak4d_state_publisher only
    # costs the camera's internal URDF TF, which calibration does not use.
    params = os.path.join(get_package_share_directory('mmb_bringup'),
                          'config', 'oak4d_v3.yaml')
    return Node(package='depthai_ros_driver_v3', executable='driver_node',
                name='oak4d', output='screen', parameters=[params])


def _realsense():
    return GroupAction([
        PushRosNamespace('realsense'),
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             name='realsense2_camera_node', output='screen',
             parameters=[{'enable_color': True, 'enable_depth': True,
                          'enable_infra1': False, 'enable_infra2': False,
                          'color_fps': 30, 'depth_fps': 30,
                          'align_depth.enable': False, 'serial_no': ''}]),
    ])


def _insta360():
    return Node(package='insta360_ros_driver', executable='insta360_ros_driver',
                name='insta360_ros_driver', output='log')


def _hesai():
    config = os.path.join(get_package_share_directory('mmb_bringup'),
                          'config', 'hesai_jt128.yaml')
    return Node(package='hesai_ros_driver', executable='hesai_ros_driver_node',
                name='hesai_ros_driver_node', output='screen',
                parameters=[config],
                remappings=[('lidar_points', '/lidar_points')])


def _livox():
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('livox_ros2_avia'),
                     'launch', 'livox_lidar_launch.py')))


# pair -> (camera factory, lidar name). MULTI-OAK note: only one camera is
# launched here, so depthai device contention does not apply to this flow.
CAMERAS = {
    'oak4d': _oak4d_v3,   # RVC4/PoE -> v3 driver (not the v2 _depthai path)
    'oak1': lambda: _depthai('oak1', 'OAK-1', 'RGB'),
    'oakd_lite': lambda: _depthai('oakd_lite', 'OAK-D-LITE', 'RGBD'),
    'realsense': _realsense,
    'insta360': _insta360,
}
LIDAR_OF = {'oak4d': 'hesai', 'insta360': 'hesai',
            'oak1': 'livox', 'oakd_lite': 'livox', 'realsense': 'livox'}


def _setup(context, *_a, **_k):
    pair = LaunchConfiguration('pair').perform(context)
    topics = LaunchConfiguration('topics').perform(context).split()
    bag = LaunchConfiguration('bag_name').perform(context)
    delay = float(LaunchConfiguration('record_delay').perform(context))

    if pair not in CAMERAS:
        raise RuntimeError(f"unknown pair '{pair}'. choices: {sorted(CAMERAS)}")
    if not topics:
        raise RuntimeError("no topics passed (topics:=\"...\")")

    camera = CAMERAS[pair]()
    lidar = _hesai() if LIDAR_OF[pair] == 'hesai' else _livox()
    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--storage', 'mcap',
             '--output', bag] + topics,
        output='screen', shell=False)

    # Drivers start now; recording starts after the warm-up delay so it does not
    # subscribe before the publishers exist.
    return [camera, lidar, TimerAction(period=delay, actions=[record])]


def generate_launch_description():
    default_bag = time.strftime(
        os.path.expanduser('~/recordings/camlidar/pair_%Y%m%d_%H%M%S'))
    return LaunchDescription([
        DeclareLaunchArgument('pair'),
        DeclareLaunchArgument('topics',
                              description='space-separated topics to record'),
        DeclareLaunchArgument('bag_name', default_value=default_bag),
        DeclareLaunchArgument('record_delay', default_value='6.0',
                              description='driver warm-up seconds before record'),
        OpaqueFunction(function=_setup),
    ])
