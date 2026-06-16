# Dual-Jetson all-sensor recording

Splitting the rig across two Jetsons to handle the throughput, while keeping the
two recordings **fuseable** (one shared clock). No sensor data crosses between
the Jetsons — only PTP time and ROS discovery.

## Roles & sensor split

| | **Camera Jetson (B) — time MASTER** | **LiDAR Jetson (A) — time SLAVE** |
|---|---|---|
| LiDAR | — | Hesai JT128 + Livox Avia |
| Cameras | OAK-1, OAK-D Lite, RealSense, Insta360 | OAK-4D |
| Other | Xsens IMU, u-blox GNSS | — |
| Clock | chrony server + ptp4l **grandmaster** | ptp4l **slave** + phc2sys |
| Topics | `config/topics_camera_jetson.yaml` | `config/topics_lidar_jetson.yaml` |

The GNSS lives on B and B is the grandmaster, so the future **GNSS/PPS upgrade**
(outdoors, with reception) is just uncommenting the `refclock` lines in B's
`chrony.conf` — no re-architecture. OAK-4D rides with the LiDARs to keep it off
B's busy USB3 bus (LiDARs are Ethernet).

**Mapping profile:** cameras record RGB only (no depth) — geometry comes from
the LiDAR. If `check_dual_sync` / `check_time_sync` show dropped frames, switch
the camera topics to their `/compressed` variants (commented in the topic
files).

## Hardware

- One PTP-capable Gigabit switch. On it: both Jetsons, Hesai, Livox.
- Each Jetson writes to **local NVMe** (~0.5–1 GB/min per side after the
  RGB-only trim; confirm with `df` before long runs).

## 1. Time sync (once per power-up, until made persistent)

```bash
# On the CAMERA Jetson (B) first — note its switch IP (e.g. 192.168.1.50):
sudo bash bash/setup_time_sync.sh master eth0

# On the LiDAR Jetson (A), pointing at B's IP:
sudo bash bash/setup_time_sync.sh slave  eth0 192.168.1.50
```
Then enable PTP slave mode in the Hesai web UI (`http://192.168.1.201` → Network
→ PTP); Livox locks automatically. Watch A lock: `journalctl -fu mmb-ptp4l`
(offset should settle to < ~1 µs with hardware timestamping, ms with software).

The systemd units (`mmb-ptp4l`, `mmb-phc2sys`) are enabled, so this survives
reboot once run.

## 2. Bring up drivers

```bash
# LiDAR Jetson (A):
ros2 launch mmb_bringup lidar.launch.py
ros2 launch mmb_bringup cameras.launch.py   # (OAK-4D only on this Jetson)

# Camera Jetson (B):
ros2 launch mmb_bringup cameras.launch.py
ros2 launch mmb_bringup imu_gnss.launch.py
```
(Trim `cameras.launch.py` per Jetson, or run only the nodes that pair has.)

## 3. Record (one terminal on each Jetson)

```bash
# LiDAR Jetson (A):
bash bash/record_split.sh lidar  <run_name>
# Camera Jetson (B):
bash bash/record_split.sh camera <run_name>
```
Clocks are synced, so the two need **not** start at the same instant — just
overlap. Each script pre-flights PTP health, topic liveness and disk, then
records its half to local MCAP. `record_split.sh` puts the two graphs on
separate `ROS_DOMAIN_ID`s (41 / 42) so they don't cross-discover over the
switch.

Optionally capture the PTP servo log during the run for the QA step:
```bash
journalctl -fu mmb-ptp4l > ptp_<role>.log    # on each Jetson, Ctrl-C after
```

## 4. Assess "how good" the data is

```bash
# per bag: rate / receive-vs-header offset / jitter / drift, PASS-FAIL per topic
python3 evaluation/check_time_sync.py --bag <bag>

# cross-Jetson: overlap, shared epoch, and (with the logs) real PTP offset
python3 evaluation/check_dual_sync.py \
    --bag-a <lidar_bag> --bag-b <camera_bag> \
    --ptp-log-a ptp_lidar.log --ptp-log-b ptp_cam.log
```
What to look for:
- **LiDARs** PTP-locked → `recv-hdr` stable (constant offset is fine; drift is
  not). Livox may still stamp on its own epoch — the printed median is the
  correction (see check_time_sync notes).
- **Cross-Jetson**: positive overlap, both bags on UNIX epoch, max |PTP offset|
  ≪ the inter-sensor timing you care about (sub-ms is ample at walking speed).
- **Dropped frames**: actual message rate well below nominal ⇒ USB3/disk
  saturation ⇒ switch that camera to `/compressed` or move it to the other
  Jetson.

## Merging for mapping

With synced clocks the two bags share a timeline; process them together by
timestamp (e.g. feed the LiDAR bag to the LIO/mapping node and sample the
camera bag at each keyframe time for colorization). No re-stamping needed beyond
any per-sensor median offset that `check_time_sync.py` reports.
