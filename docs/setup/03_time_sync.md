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

**Both LiDARs support IEEE 1588v2 PTP slave mode**, so one `ptp4l` master on
the Jetson's LiDAR Ethernet interface disciplines both:

- **HESAI JT128**: PTP slave, enabled in its web UI (below).
- **Livox Avia**: supports PTP / GPS / PPS with priority **PTP > GPS > PPS**
  (per the Livox device time-synchronization manual). With a PTP master on the
  network it synchronizes automatically — no driver configuration needed.
  Without it, the Avia free-runs on its power-on clock (we measured the
  resulting offset at ~1.78e9 s with +12 ppm drift on the 2026-06-09 bag).

Everything in this section is automated by **`bash/setup_time_sync.sh`** (run
once with sudo; installs chrony + linuxptp, deploys the config below, and
installs systemd units `mmb-ptp4l` / `mmb-phc2sys` so the stack survives
reboots):

```bash
sudo bash bash/setup_time_sync.sh eth0    # use your LiDAR interface (ip link)
```

Manual equivalent, for reference:

```bash
sudo apt install -y linuxptp ethtool
ethtool -T eth0                      # check for hardware-transmit timestamping
sudo ptp4l -i eth0 --masterOnly 1 -m       # hardware timestamping
sudo ptp4l -i eth0 --masterOnly 1 -S -m    # software timestamping fallback
# Hardware mode only: keep the NIC PHC following the system clock so PTP time
# on the wire equals system/UTC time:
sudo phc2sys -c eth0 -s CLOCK_REALTIME -O 0 -m
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

**Recorded-bag audit (the authoritative check):** every bag can be verified
offline, per topic, with

```bash
python3 evaluation/check_time_sync.py --bag <bag_dir>
```

It reports each topic's rate, median header-vs-receive offset, jitter, and
clock drift (ppm), with PASS/FAIL verdicts. A large but stable offset means a
sensor is on its own clock epoch (fix = PTP, or carry the printed median as a
post-hoc correction); nonzero drift means a free-running oscillator — sync it,
don't just subtract a constant on long recordings.

---

## Field Operation Note

When operating outdoors with GNSS, the u-blox receiver outputs a **1 PPS signal**
on its timepulse pin. For maximum accuracy, this PPS can be fed into the
Jetson's GPIO and used to discipline chrony at the hardware level
(`refclock SHM 0 refid GPS`). This provides UTC-aligned accuracy < 1 μs.
This is optional for the current project scope but worth noting for future
improvement.
