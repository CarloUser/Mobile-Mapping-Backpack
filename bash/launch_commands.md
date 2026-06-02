# Xsens

*open two terminals:*

```
roslaunch xsens_mti_driver xsens_mti_node.launch
```

*or with the 3D display rviz:*

```
roslaunch xsens_mti_driver display.launch
```

*and then:*

```
roslaunch ntrip ntrip.launch
```

# Hesai

For ROS2-Dashing

```
  ros2 launch hesai_ros_driver dashing_start.py
```

For other ROS2 version

```
  ros2 launch hesai_ros_driver start.py
```

# Livox

Publish Livox customized point cloud data

```
ros2 launch livox_ros2_avia livox_lidar_msg_launch.py
```

Publish pointcloud2 format data

```
ros2 launch livox_ros2_avia livox_lidar_launch.py
```

Publish pointcloud2 format data and visualize the point cloud in RViz

```
ros2 launch livox_ros2_avia livox_lidar_rviz_launch.py
```

# Realsense

```
ros2 run realsense_ros2_camera realsense_ros2_camera
```

# Xsens IMU

open first terminal:

```
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
```

or with the 3D display rviz:

```
ros2 launch xsens_mti_ros2_driver display.launch.py
```

and then open another terminal

```
ros2 launch ntrip ntrip_launch.py
```

# 



