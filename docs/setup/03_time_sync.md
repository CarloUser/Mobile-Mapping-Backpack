# Time Synchronization

Misaligned timestamps corrupt calibration and make trajectory evaluation
unreliable. Target: all sensor timestamps agree to within ~1 ms for
indoor tests, <0.1 ms for outdoor RTK evaluation.

Two layers apply here:

1. **System clock sync** (chrony/NTP) — keeps the Jetson's Linux clock aligned
   to UTC. Affects all sensors that stamp using `ros::Time::now()`.
2. **LiDAR hardware sync** (PTP/GPS/PPS where supported) — the HESAI JT128 and
   Livox Avia must be aligned closely enough for LiDAR-LiDAR calibration and
   mapping. Use PTP where possible, and verify both point-cloud header stamps.

---

## 1. Install and Configure Chrony

```bash
sudo apt install -y chrony
```

Edit `/etc/chrony/chrony.conf`. Replace the default pool lines with:

```
# Use pool.ntp.org when internet is available
pool pool.ntp.org iburst maxsources 4

# Fall back to the Jetson's own clock when offline (important for field use)
local stratum 10

# Allow ROS nodes on the same LAN to use this Jetson as NTP server
allow 192.168.0.0/16
```

```bash
sudo systemctl restart chrony
# Check sync status (wait ~30 s after restart)
chronyc tracking
# "System time" should drop to <1 ms within a minute on a good internet connection
```

---

## 2. LiDAR Time Synchronization

The JT128 can act as a **PTP slave**, disciplining its internal clock to a PTP
master on the network. The Jetson can serve as the master using `linuxptp`.
Configure the Livox Avia with the best supported sync mode available in the
driver/hardware setup, then verify its PointCloud2 header stamps against the
same system time.

```bash
sudo apt install -y linuxptp
```

Identify the Ethernet interface connected to the LiDAR network:
```bash
ip link   # e.g., eth0 or enp3s0
```

Start `ptp4l` as master on that interface:
```bash
sudo ptp4l -i eth0 -m -s    # -m = master, -s = software timestamping
# For hardware timestamping (preferred if NIC supports it):
sudo ptp4l -i eth0 -m -H
```

Sync the system clock to PTP time:
```bash
sudo phc2sys -a -rr -s CLOCK_REALTIME -m
```

**Configure the HESAI**: Open the JT128 web interface at `http://192.168.1.201`,
navigate to **Network → PTP** and enable PTP slave mode. After ~30 seconds,
the LiDAR's internal clock should track the Jetson's PTP master.

**Verify** in the JT128 web UI: PTP status should show "Locked". Then check both
LiDAR PointCloud2 header timestamps:

```bash
ros2 topic echo /lidar_points --field header.stamp --once
ros2 topic echo /livox/lidar --field header.stamp --once
ros2 topic echo /livox/imu --field header.stamp --once
```

---

## 3. IMU and Camera Timestamps

The Xsens MTi-610R, OAK cameras, and RealSense all stamp using the host system
clock via their USB drivers. With chrony keeping the Jetson clock accurate,
these are automatically aligned. No extra configuration needed.

**Exception**: The OAK-4D has an onboard IMU that can optionally use hardware
timestamps. Check the depthai-ros `imu_from_descr` parameter — leaving it
at the default (software timestamps) is acceptable for our purposes.

---

## 4. Verify End-to-End

With all sensors running:

```bash
# Check timestamp spread across all sensor topics
ros2 topic echo /lidar_points --field header.stamp --once
ros2 topic echo /livox/lidar --field header.stamp --once
ros2 topic echo /livox/imu --field header.stamp --once
ros2 topic echo /imu/data --field header.stamp --once
ros2 topic echo /oak4d/rgb/image_raw --field header.stamp --once
```

All stamps should be within a few milliseconds of each other. Large
discrepancies (>50 ms) indicate a driver not using the system clock or a
chrony sync issue.

```bash
# Quick sanity check
chronyc tracking | grep "System time"
```

---

## Field Operation Note

When operating outdoors with GNSS, the u-blox receiver outputs a **1 PPS signal**
on its timepulse pin. For maximum accuracy, this PPS can be fed into the
Jetson's GPIO and used to discipline chrony at the hardware level
(`refclock SHM 0 refid GPS`). This provides UTC-aligned accuracy < 1 μs.
This is optional for the current project scope but worth noting for future
improvement.
