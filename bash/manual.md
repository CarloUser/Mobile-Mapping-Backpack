# Manual of the ROS Drivers

## Prerequisites and prepeation

<div style="padding: 15px; border-left: 5px solid #007acc; background-color: #2e4250da; border-radius: 0 15px 0px 0">
    <strong>Ubuntu 22.04:</strong> Most of the ROS drivers are based on Ubuntu 22, thus it is mandatory
</div>

<br>

+ If you want you can try installing everything on a higher version but we don't recommend that as it is not tested.

<br>

<div style="padding: 15px; border-left: 5px solid #cc4b00; background-color: #503d2eda;">

<strong>Using Blickfeld:</strong> We didn't use the Blickfeld LiDAR and thus it has not being tested on any machine. If you want to use it make sure you try to install the driver byitself first.

</div>

<br>

+ If you want to use the `blickfled` drviers, make sure to add the corresponding sdk and workspace setup files to the `run_all_jetson.sh` or run them by yourself.

<br>

<div style="padding: 15px; border-left: 5px solid #00cca0; background-color: #194546; border-radius: 0 0px 0px 0;">

**1. Insta360 x5 SDK:** You have to download the newest insta360 SDK and put the zip file into the `/Downloads` directory if not already there.

**2. Naming of the zip file:** The current name that will be recognized is `Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip`. If needed you can either change the file's name or the name in the `setup_insta360_sdk.sh` and `setup_insta360_sdk_jetson.sh`.

</div>

<br>

+ There are more files that can be used after installing the drivers like:
    + **launch files**: allow you to launch sensors in groups or all at once. You can read more on that in the [launch section](#launching-the-drivers).
    + **Record files**: are used to record specific topics in ROS2.
    + **Wipe files**: can be used to remove all the installation. Be really cautious as they **wipe** out everything that the driver is dependant on.

<br>

<div style="padding: 15px; border-left: 5px solid #007acc; background-color: #2e4250da; border-radius: 0 0px 15px 0;">

**Repository:** Every needed file is in the repository. You can clone the repository from Github:

`git clone https://github.com/CarloUser/Mobile-Mapping-Backpack.git`

</div>

## Installing on jetson

Make sure you have met all the requirements in [Prerequisites](#prerequisites-and-prepeation) before trying to install the packages.

There are three ways to install all the drivers. You can either install it using our [one_command](#run_all_jetson-method) file or running each driver's installation file [seperately](#running-files-one-by-one). The last way is by installing each from [the official docs](#installation-form-the-docs).

### Run_all_jetson Method

The easiest way to install all the drivers is to use the `run_all_jetson.sh` in the `/bash` directory in the repository.

### Running files one by one

Another way is by running each file seperately. In that case you have to maek sure you are using the right files and the right order.

This is the order for the jetson, the most important one is to run `jetson_libopencv.sh` before installing the ros2.

    "jetson_libopencv.sh"

    "setup_ros2_humble.sh"

    "setup_depthai_v2.sh"
    "setup_depthai_v3_auto.sh"

    "setup_realsense_sdk_jetson.sh"
    "setup_realsense_binary.sh"

    "setup_insta360_sdk_jetson.sh"
    "setup_insta360.sh"
    "config_insta360_jetson.sh"

    "setup_livox.sh"
    "setup_livox_config.sh"

    "setup_hesai_sdk.sh"
    "setup_hesai.sh"
    "config_hesai.sh"

    "setup_xsens.sh"

    "setup_gnss.sh"
    "config_gnss.sh"

    "build_sensors.sh"

### Installation form the docs

Alternatively you can install each driver by yourself. You may follow the instructions here:

<div style="padding: 15px; border-left: 5px solid #cc4b00; background-color: #503d2eda;">

**Installing both depthai versions:** Normally you can't install both. The idea we came up with was to install the v2 in the workspace and build the packages, as depthai will automatically look for the latest packages available. Then the idea is to install the v3 through apt istallation globally.

Remember **not to rebuild** the v2 packages after installing the v3 as it will break.

That is why we recommend to use a **seperate workspace** for **depthai_v2** so you don't accidently rebuild it's packages. 

If you really want to have it with other ones in one workspace though, you have to allways build with `--symlink-install` and `--packages-select` flags.

</div>

<br>

`Jetsonlibopencv`:

[Forum for installing the jetson specific opencv](https://forums.developer.nvidia.com/t/opencv-libraries-missing-or-links-are-broken-on-a-fresh-install/335400?utm_source=chatgpt.com)

`Ros2`:

[ROS2 humble setup for Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

`Depthai v2`:

<div style="padding: 15px; border-left: 5px solid #cc4b00; background-color: #503d2eda;">

make sure as you clone the `depthai_ros` use the `humble` branch and for `depthai_core` use the `v2_stable`. you can either `git checkout` to those branches or clone with those branches directly wiht `git clone -b <branch_name> <link>`.

</div>

Here is also the [link](https://docs.luxonis.com/software/ros/depthai-ros/build/) to the installation from the source.

Note that it is made mainly for the `ROS1`. If you need help you can always look into the `setup_depthai_v2.sh`.

`Depthai v3`:

[Depthai v3 humble setup](https://docs.luxonis.com/software-v3/depthai/ros/driver/)

`Realsense`:

[realsense sdk for jetson](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)

[realsense driver setup](https://github.com/realsenseai/realsense-ros)

`Insta360`:

[Insta360 full setup](https://github.com/ai4ce/insta360_ros_driver/tree/ros2)

`Livox LiDAR`:

[Livox ROS2 humble setup](https://github.com/ASIG-X/livox_ros2_avia)

`Hesai JT128`:

[Hesai ROS2 setup](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0)

`Xsens`:

[Xsens ROS2 setup](https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/ros2)

`GNSS`:

[GNSS ROS2 setup](https://www.ardusimple.com/how-to-integrate-u-blox-zed-f9p-gnss-rtk-receiver-into-ros-2-jazzy/)

Note that you might need to twick some commands as you follow through this one.

Congratulation! You now installed everything manually you nerd.

## Launching the drivers

Always source ROS 2 and the driver workspace first (do this in every terminal):

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Tip: launch over SSH inside `tmux` so the drivers survive a dropped connection.

### Using Launch bash files

The `mmb_bringup` package groups the drivers into launch files so you can start
everything, or one group, with a single command:

```bash
# all sensors at once (LiDARs + cameras + IMU/GNSS + static TF tree)
ros2 launch mmb_bringup all_sensors.launch.py

# one group at a time
ros2 launch mmb_bringup lidar.launch.py        # Hesai + Livox
ros2 launch mmb_bringup cameras.launch.py      # OAK-4D, OAK-D Lite, OAK-1, RealSense, Insta360
ros2 launch mmb_bringup imu_gnss.launch.py     # Xsens + u-blox

# publish only the static base_link -> sensor transforms from a YAML
ros2 launch mmb_bringup static_extrinsics.launch.py \
  extrinsics_file:=ros2_ws/src/mmb_bringup/config/extrinsics_initial.yaml
```

`all_sensors.launch.py` simply includes the three group launches plus
`static_extrinsics.launch.py`, so bringing the whole rig up is one command.

### Using Launch commands manually

To bring up a single driver directly (useful for debugging one sensor), use the
per-driver commands below.

```bash
# Hesai JT128  -> /lidar_points (+ /lidar_imu)
ros2 launch hesai_ros_driver start.py

# Livox Avia: CustomMsg with per-point time (use THIS for calibration / FAST-LIO)
ros2 launch livox_ros2_avia livox_lidar_msg_launch.py
#   PointCloud2 format instead (no per-point time):
ros2 launch livox_ros2_avia livox_lidar_launch.py

# OAK-4D (DepthAI v3); add use_rviz:=true to visualize
ros2 launch depthai_ros_driver_v3 driver.launch.py

# OAK-D Lite / OAK-1 (DepthAI v2)
ros2 launch depthai_ros_driver camera.launch.py

# Intel RealSense
ros2 launch realsense2_camera rs_launch.py

# Insta360 (publishes the H.264 dual-fisheye stream)
ros2 launch insta360_ros_driver bringup.launch.xml

# Xsens MTi-610R  -> /imu/data, /imu/mag   (display.launch.py adds RViz)
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py

# u-blox GNSS  -> /gnss/fix, /gnss/fix_velocity
ros2 launch ublox_gps ublox_gps_node-launch.py

# Optional: RTK correction client (NTRIP), run alongside the GNSS
ros2 launch ntrip ntrip_launch.py
```

## Recording topics

### Topics

The canonical list of topics to record is
`ros2_ws/src/mmb_bringup/config/topics.yaml` — the **single source of truth**, so
do not duplicate the list elsewhere. It covers both LiDAR clouds and their IMUs,
the four pinhole cameras' RGB + `camera_info`, the Insta360's compressed
dual-fisheye stream, the Xsens IMU + magnetometer, the GNSS fix + velocity, and
the TF tree (`/tf`, `/tf_static`). For mapping runs the cameras are recorded
**RGB-only** (depth omitted — geometry comes from the LiDARs); `/compressed`
variants are available (commented out) if storage is the bottleneck. Aggregate
bandwidth at full rate is ~1.5–2 GB/min, so make sure you have disk headroom.

### Recording with the bash files

`bash/record_all.sh` launches every driver (`all_sensors.launch.py`) **and**
records every topic in `topics.yaml` to one MCAP bag. It runs a pre-flight check
(clock sync, devices reachable — Hesai at `192.168.1.201`, OAK-4D PoE at
`192.168.1.97` — and ≥20 GB free) and a ~60 s warm-up before subscribing.

```bash
# Detached — survives SSH / works from separate prompts:
bash bash/record_all.sh start <location>_<run_type>_<run_number> [out_dir]
bash bash/record_all.sh status
bash bash/record_all.sh stop

# Interactive — a real terminal with a live status line; stop with Ctrl+C or 'stop':
bash bash/record_all.sh <location>_<run_type>_<run_number> [out_dir]
```

Example run name: `campus_loop_mapping_01`. The rig records whatever drivers
actually come up — a missing sensor just won't appear; check `<bag>.drivers.log`
(driver output) and `<bag>.record.log` (recorder output) and the pre-flight
`[ MISSING ]` lines if a sensor is absent. To split the rig across two Jetsons
for throughput, see `docs/dual_jetson_recording.md` (`record_dual.sh`).

### Recording manually with ROS2

To record from an already-running driver set, call rosbag2 directly with MCAP
storage:

```bash
ros2 bag record --storage mcap -o <bag_name> /lidar_points /livox/lidar ...
```

The launch file `recording.launch.py` does exactly this from the topic list, so
the one-command equivalent is:

```bash
ros2 launch mmb_bringup recording.launch.py \
  bag_name:=/data/bags/mmb_$(date +%Y%m%d_%H%M)_<location>_<run_type>_<run_number>
```

Naming convention: `mmb_YYYYMMDD_HHMM_<location>_<run_type>_<run_number>`.

## Trouble shooting

| Symptom | Cause / fix |
| --- | --- |
| `/livox/lidar` records **0 messages** | The recorder needs the `livox_interfaces` typesupport — `source ~/ros2_ws/install/setup.bash` before recording (`record_all.sh` does this automatically). |
| LiDAR header stamps drift / don't match | PTP not locked. Run `sudo bash bash/setup_time_sync.sh <iface>`, enable PTP slave in the Hesai web UI (`http://192.168.1.201` → Network → PTP), and confirm `chronyc tracking` shows < 1 ms. Without PTP the Livox free-runs (~10⁹ s offset). |
| Hesai won't stop on **Ctrl-C** | It ignores SIGINT — kill its process with **SIGKILL** between recordings. |
| OAK build breaks after installing v3 | Do **not** rebuild the DepthAI **v2** packages after the global **v3** apt install; keep v2 in its own workspace. |
| OAK camera not detected | Install the Movidius `udev` rule and use a **USB3** port (not USB2); each OAK needs its own USB3 port or a powered hub. |
| OAK-4D crashes (SIGSEGV) | Use DepthAI **v3 ≥ 3.3.0** (3.2.1 has an ABI mismatch); pass parameters via a **file**, not `-p` on the CLI. |
| Xsens / GNSS "permission denied" on serial | Add your user to `dialout` (`sudo usermod -aG dialout $USER`, then re-login); find the port with `ls /dev/ttyUSB* /dev/ttyACM*`. |
| Insta360 driver fails to start | Its bundled `libCameraSDK.so` must be on `LD_LIBRARY_PATH` (`record_all.sh` adds `~/ros2_ws/src/insta360_ros_driver/lib`); the SDK zip must be named `Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip` in `~/Downloads` (or update the name in `setup_insta360_sdk*.sh`). |
| Hesai unreachable | Set the Jetson's LiDAR NIC to a static IP in `192.168.1.0/24` (e.g. `192.168.1.100`); the Hesai is at `192.168.1.201`. |
| A sensor missing from the bag | `record_all.sh` records whatever comes up — inspect `<bag>.drivers.log` and the pre-flight output for the failing device. |
