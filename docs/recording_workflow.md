# All-sensor recording workflow (mobile mapping)

End-to-end procedure for recording a mapping run with **every** sensor on the
rig, time-synchronized, producing a bag that downstream calibration/mapping can
consume. Complements `data_collection_protocol.md` (naming, frame IDs);
supersedes nothing.

## What "synchronized" means on this rig

| Sensor group | Clock | Sync mechanism |
| --- | --- | --- |
| Jetson host (recorder) | system clock | **chrony/NTP** (internet) or free-running fallback in the field |
| OAK-4D, OAK-D Lite, RealSense, Xsens, u-blox, Insta360 | host-stamped via USB drivers | inherit the Jetson clock automatically |
| HESAI JT128 | internal | **PTP slave** of the Jetson (enable in web UI) |
| Livox Avia | internal | **PTP slave** of the Jetson (automatic when a master exists; priority PTP > GPS > PPS) |

Without PTP, both LiDARs free-run (measured on the 2026-06-09 bag: offsets of
~10^9 s with ~+10 ppm drift). With the stack below, all header stamps land on
one UTC-aligned timeline at the ms level — which is what multi-sensor mapping
needs.

## One-time setup

```bash
sudo bash bash/setup_time_sync.sh <lidar_iface>   # chrony + ptp4l master (systemd)
# Then once: Hesai web UI (http://192.168.1.201) -> Network -> PTP -> slave.
```

Details and verification: `docs/setup/03_time_sync.md`.

## Per-session procedure

1. **Power + network up**, then check sync (takes <1 min after boot):
   ```bash
   chronyc tracking | grep "System time"     # < a few ms
   systemctl status mmb-ptp4l --no-pager | head -3
   ```
2. **Start all drivers** (tmux strongly recommended over SSH):
   ```bash
   tmux new -s rig
   source ~/ros2_ws/install/setup.bash
   ros2 launch mmb_bringup all_sensors.launch.py
   ```
3. **Record** (second tmux pane). `record_all.sh` runs the pre-flight checks
   (clock sync, all topics live, disk space) and then records every topic in
   `mmb_bringup/config/topics.yaml` to MCAP:
   ```bash
   bash bash/record_all.sh <location>_<run_type>_<run_number>
   # e.g.  bash bash/record_all.sh campus_loop_mapping_01
   ```
   Stop with a single Ctrl-C in the recording pane.
   (`ros2 launch mmb_bringup recording.launch.py bag_name:=...` is the
   launch-file equivalent; it reads the same topics.yaml.)
4. **Verify before leaving the site** — a bag that fails here is a re-record,
   not a post-processing problem:
   ```bash
   ros2 bag info <bag>                                   # all topics, sane counts
   python3 evaluation/check_time_sync.py --bag <bag>     # per-topic PASS/FAIL
   ```

## Motion profile for mapping runs

- Start and end with ~10 s static (IMU bias estimation downstream).
- Close at least one loop (revisit the start area) per run.
- Smooth walking; avoid fast spins (the Avia's narrow FOV loses tracking —
  see the FAST-LIO tuning notes in `extrinsic_calibration/lidar_lidar/`).
- Outdoors: wait for GNSS fix before starting (`ros2 topic echo /gnss/fix
  --once`), and prefer runs that begin/end in open sky for RTK evaluation.

## Topic list / bandwidth

The recorded set is defined **only** in `ros2_ws/src/mmb_bringup/config/topics.yaml`
(recording.launch.py and record_all.sh both read it). Notable choices:

- `/livox/lidar` is recorded as `livox_interfaces/CustomMsg` (per-point time,
  required by FAST-LIO). Convert to PointCloud2 offline if a consumer needs it.
- The Insta360 is recorded as the H.264-compressed dual-fisheye stream
  (`/dual_fisheye/image/compressed`); decode offline with the driver's
  `decoder` / `equirectangular_cpp` nodes.
- `/lidar_imu` (Hesai IMU) is recorded — the LiDAR↔LiDAR calibration uses it.
- Budget ~1.5–2 GB/min all-in. `record_all.sh` warns under 20 GB free.

## Known open items

- `cameras.launch.py` device assignment (OAK-4D vs OAK-D Lite by `camera_model`,
  RealSense by serial) has not been validated with all cameras attached
  simultaneously — verify namespaces/topics on first full bring-up against
  `topics.yaml`, and pin `serial_no`/`mx_id` per device if discovery is flaky.
- u-blox/Xsens serial ports are configured as `/dev/ttyACM0`/`/dev/ttyUSB0` in
  `imu_gnss.launch.py`; add udev rules (see `docs/setup/02_sensor_drivers.md`)
  for stable naming before relying on them.
- GNSS PPS → chrony hardware discipline (sub-µs UTC) is documented as a future
  improvement in `03_time_sync.md`; not needed for current scope.
