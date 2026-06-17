import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # The v2 OAK driver (camera_node, used by oakd_lite + oak1) links
    # libdepthai-core.so transitively through libdepthai_ros_driver.so. The
    # binary's DT_RUNPATH does NOT cover that sub-dependency, so the v2
    # depthai-core install lib has to be on LD_LIBRARY_PATH at runtime — but
    # ONLY for these v2 nodes. The oak4d driver (depthai_ros_driver_v3) ships
    # its own incompatible core; putting v2's core on a global path would break
    # it, so we scope it per-node via additional_env.
    home = os.path.expanduser('~')
    v2_core_lib = os.path.join(
        home, 'ros2_ws', 'depthai_install', home.lstrip('/'),
        'ros2_ws', 'src', 'depthai-core', 'build', 'install', 'lib')
    v2_env = {'LD_LIBRARY_PATH': v2_core_lib + ':' + os.environ.get('LD_LIBRARY_PATH', '')}

    # OAK-4D (OAK-4-PRO, RVC4 PoE) — MUST use the official depthai_ros_driver_v3
    # (the v2 depthai_ros_driver cannot drive RVC4). Params (device id + 1080p
    # @10fps RGB) in config/oak4d_v3.yaml. Plain Node name='oak4d' -> publishes
    # /oak4d/rgb/image_raw + /oak4d/rgb/camera_info.
    oak4d_group = Node(
        package='depthai_ros_driver_v3',
        executable='driver_node',
        name='oak4d',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('mmb_bringup'),
            'config', 'oak4d_v3.yaml')],
    )

    # OAK-D Lite — v2 depthai-ros. The v2 driver publishes under the NODE NAME
    # (like oak4d above -> /oak4d/...), so name it 'oakd_lite' directly instead
    # of PushRosNamespace+camera_node (which would give /oakd_lite/camera_node/..).
    # Params live under the 'camera.' namespace; default i_nn_type is 'spatial'
    # which loads the mobilenet NN blob and aborts (PackageNotFoundError) — set
    # camera.i_nn_type=none. RGB-only pipeline (we don't record depth), which
    # also skips the stereo setup.
    oakd_lite_node = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oakd_lite',
        output='screen',
        additional_env=v2_env,
        parameters=[{
            # Pin to THIS rig's OAK-D-Lite by serial — with oak1 also on USB the
            # driver otherwise races and grabs whichever it finds first. MXIDs
            # from `dai.Device.getAllAvailableDevices()`.
            'camera.i_mx_id': '19443010A1063C1200',
            'camera.i_pipeline_type': 'RGB',
            'camera.i_nn_type': 'none',
        }],
    )

    # OAK-1 (rear, single 4K RGB — no stereo/depth) — v2 depthai-ros.
    # MULTI-OAK GOTCHA: with oak4d + oakd_lite + oak1 all on one host the driver
    # may grab the wrong device — set 'camera.i_mx_id' (or 'camera.i_usb_port_id')
    # per node to pin each to its serial if they race.
    oak1_node = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oak1',
        output='screen',
        additional_env=v2_env,
        parameters=[{
            'camera.i_mx_id': '19443010F16F9B4800',   # this rig's OAK-1
            'camera.i_pipeline_type': 'RGB',
            'camera.i_nn_type': 'none',
        }],
    )

    # Intel RealSense — realsense2_camera. The driver publishes under its full
    # node name, so name='realsense' (no PushRosNamespace) gives
    # /realsense/color/image_raw — matching topics.yaml. (Namespace + a separate
    # node name would nest as /realsense/<node>/color/..., which the recorder
    # wouldn't match.) Adjust serial_no if you have multiple RealSense units.
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,    # RGB (color) only — depth not recorded
            'enable_infra1': False,
            'enable_infra2': False,
            'color_fps': 30,
            'align_depth.enable': False,
            'serial_no': '',   # leave empty to use first detected unit
        }],
    )

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
        oakd_lite_node,
        oak1_node,
        realsense_node,
        insta360_node,
    ])
