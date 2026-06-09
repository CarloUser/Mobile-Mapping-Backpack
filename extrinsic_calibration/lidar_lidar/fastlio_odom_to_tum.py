#!/usr/bin/env python3
"""Convert a recorded FAST-LIO `/Odometry` (nav_msgs/Odometry) stream into a TUM
trajectory for solve_extrinsic.py.

FAST-LIO publishes the IMU-body pose in the map/init frame. Our calibration needs
the Livox *LiDAR-frame* trajectory (that is what the CAD `lidar_livox_avia` frame
is), so we compose the LiDAR->IMU extrinsic that FAST-LIO uses:

    T_map_lidar = T_map_imu @ T_imu_lidar

where T_imu_lidar = (extrinsic_R, extrinsic_T) -- the "LiDAR pose in IMU frame"
from the FAST-LIO config. IMPORTANT: pass the SAME extrinsic you set in the
FAST-LIO yaml so the two stay consistent. Defaults are the standard Livox Avia
values.

Workflow:
  1. Run FAST-LIO on the (re-recorded, CustomMsg) bag and record its output:
       ros2 launch fast_lio mapping.launch.py config_file:=fast_lio_avia.yaml &
       ros2 bag record -o livox_odom /Odometry
       ros2 bag play <your_customsg_bag>
  2. Convert:
       python fastlio_odom_to_tum.py --bag livox_odom --topic /Odometry \
           --out out/livox_tum.txt
"""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

AVIA_EXT_T = [0.04165, 0.02326, -0.0284]       # Livox Avia LiDAR position in IMU frame
AVIA_EXT_R = [1, 0, 0, 0, 1, 0, 0, 0, 1]


def se3(Rm, t):
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = t; return T


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="Bag containing the FAST-LIO odometry topic")
    ap.add_argument("--topic", default="/Odometry")
    ap.add_argument("--out", default="out/livox_tum.txt")
    ap.add_argument("--ros-distro", default="humble")
    ap.add_argument("--ext-t", type=float, nargs=3, default=AVIA_EXT_T,
                    help="LiDAR->IMU translation (must match FAST-LIO config)")
    ap.add_argument("--ext-r", type=float, nargs=9, default=AVIA_EXT_R,
                    help="LiDAR->IMU rotation, row-major 3x3 (must match FAST-LIO config)")
    ap.add_argument("--no-extrinsic", action="store_true",
                    help="Do NOT compose the extrinsic (output the IMU-frame trajectory)")
    args = ap.parse_args()

    T_imu_lidar = np.eye(4) if args.no_extrinsic else \
        se3(np.array(args.ext_r, float).reshape(3, 3), np.array(args.ext_t, float))

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(getattr(Stores, f"ROS2_{args.ros_distro.upper()}"))

    out = Path(args.out); out.parent.mkdir(parents=True, exist_ok=True)
    n = 0
    with AnyReader([Path(args.bag)], default_typestore=ts) as reader, open(out, "w") as f:
        conns = [c for c in reader.connections if c.topic == args.topic]
        if not conns:
            avail = sorted({c.topic for c in reader.connections})
            raise SystemExit(f"Topic '{args.topic}' not in bag. Available: {avail}")
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            st = m.header.stamp
            t = float(st.sec) + float(st.nanosec) * 1e-9
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            T_map_imu = se3(R.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [p.x, p.y, p.z])
            T = T_map_imu @ T_imu_lidar
            pos = T[:3, 3]; quat = R.from_matrix(T[:3, :3]).as_quat()
            f.write(f"{t:.6f} {pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} "
                    f"{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}\n")
            n += 1
    if n == 0:
        raise SystemExit(f"No messages on '{args.topic}'.")
    print(f"[fastlio_odom_to_tum] wrote {n} poses to {out} "
          f"({'IMU frame' if args.no_extrinsic else 'LiDAR frame (extrinsic applied)'})")


if __name__ == "__main__":
    main()
