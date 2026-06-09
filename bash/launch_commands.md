# Launch Commands

## Xsens

*open two terminals:*

```shell
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch
```

*or with the 3D display rviz:*

```shell
ros2 launch xsens_mti_driver display.launch
```

*and then:*

```shell
ros2 launch ntrip ntrip.launch
```

## Hesai

For ROS2-Dashing

```shell
  ros2 launch hesai_ros_driver dashing_start.py
```

For other ROS2 version

```shell
  ros2 launch hesai_ros_driver start.py
```

## Livox

Publish Livox customized point cloud data

```shell
ros2 launch livox_ros2_avia livox_lidar_msg_launch.py
```

Publish pointcloud2 format data

```shell
ros2 launch livox_ros2_avia livox_lidar_launch.py
```

Publish pointcloud2 format data and visualize the point cloud in RViz

```shell
ros2 launch livox_ros2_avia livox_lidar_rviz_launch.py
```

## OAK 4D

Run the driver with:
Command Line

```shell
ros2 launch depthai_ros_driver_v3 driver.launch.py
```

To visualize data in RViz:
Command Line

```shell
ros2 launch depthai_ros_driver_v3 driver.launch.py use_rviz:=true
```

For running the composable driver package as a separate node:
Command Line

```shell
ros2 run depthai_ros_driver_v3 driver_node
```

## OAK D Lite

Run the driver with:
Command Line

```shell
ros2 launch depthai_ros_driver_v3 driver.launch.py
```

To visualize data in RViz:
Command Line

```shell
ros2 launch depthai_ros_driver_v3 driver.launch.py use_rviz:=true
```

For running the composable driver package as a separate node:
Command Line

```shell
ros2 run depthai_ros_driver_v3 driver_node

```

## OAK D 1

```shell
ros2 launch depthai_ros_driver camera.launch.py
```

## Realsense

```shell
ros2 run realsense_ros2_camera realsense_ros2_camera
```

```shell
ros2 launch realsense2_camera rs_launch.py
```

## Insta360

```shell
ros2 launch insta360_ros_driver bringup.launch.xml
```

## Xsens IMU

open first terminal:

```shell
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
```

or with the 3D display rviz:

```shell
ros2 launch xsens_mti_ros2_driver display.launch.py
```

and then open another terminal

```shell
ros2 launch ntrip ntrip_launch.py
```

## GNSS

```shell
ros2 launch ublox_gps ublox_gps_node-launch.py
```
