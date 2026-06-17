from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Xsens MTi-610R IMU
    # Adjust port to match what shows up on the Jetson (/dev/ttyUSB0 or /dev/ttyACM0)
    xsens_node = Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baud_rate': 0,           # 0 = auto-detect
            'frame_id': 'imu_xsens',
            # The all-sensors bag got /imu/mag but NO /imu/data: with
            # enable_deviceConfig=false the driver used the MTi's stored output
            # config, which streamed MagneticField but not accel/gyro, so the
            # node had nothing to publish on /imu/data. Set enable_deviceConfig
            # true so the driver programs the device output (accel + gyro +
            # quaternion at output_data_rate) on startup. It writes the config to
            # the device, so you may flip this back to False once a recording
            # confirms /imu/data streams (leaving it True just reconfigures each
            # launch — idempotent, a few seconds slower).
            'enable_deviceConfig': True,
            'output_data_rate': 400,  # MTi-610R supports up to 400 Hz
            'pub_imu': True,
            'pub_acceleration': True,
            'pub_angular_velocity': True,
            'pub_quaternion': True,
            'pub_mag': True,
            'pub_gps': False,
            'pub_gnss': False,
            'pub_temp': False,
        }],
        remappings=[
            ('imu/data', '/imu/data'),
            ('imu/mag', '/imu/mag'),
        ],
    )

    # u-blox GNSS receiver (paired with ANN-MB-00 antenna)
    # Adjust port to match your u-blox receiver's USB/serial port
    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[{
            'device': '/dev/ttyACM0',
            'frame_id': 'gnss_antenna',
            'baud': 115200,
            'config_on_startup': False,   # set True once you have a custom config file
            'rate': 5.0,
        }],
        remappings=[
            ('fix', '/gnss/fix'),
            ('fix_velocity', '/gnss/fix_velocity'),
        ],
    )

    return LaunchDescription([xsens_node, ublox_node])
