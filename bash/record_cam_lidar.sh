#!/usr/bin/env bash
# Record one camera<->LiDAR board-calibration bag for a single pair.
#
# Unlike record_all.sh (full mission, all topics from mmb_bringup/topics.yaml),
# this records ONLY the three topics that pair needs — image + camera_info +
# LiDAR — read straight out of extrinsic_calibration/camera_lidar/config.yaml so
# the record command can never drift from what the calibrator expects.
#
# Usage:  bash record_cam_lidar.sh <pair> [out_dir]
#         bash record_cam_lidar.sh oak4d
#         bash record_cam_lidar.sh realsense /data/bags
#   <pair> is a key under `pairs:` in config.yaml (oak4d, oak1, oakd_lite,
#   realsense, insta360).
#
# Recording protocol (see calibrate_cam_lidar.py): hold the ChArUco board STATIC
# 3-5 s per pose, board fully visible to the camera AND hit by the LiDAR, 15-25
# well-spread poses (vary distance, angle, and board-normal direction).
PAIR="${1:?usage: record_cam_lidar.sh <pair> [out_dir]}"
OUT_DIR="${2:-$HOME/recordings/camlidar}"

# ROS setup scripts reference unbound variables — source before `set -u`.
# ~/ros2_ws gives the livox_interfaces typesupport; without it the Livox
# CustomMsg topic records as zero messages (silently).
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
CONFIG="$HERE/../extrinsic_calibration/camera_lidar/config.yaml"
[ -f "$CONFIG" ] || { echo "config not found: $CONFIG" >&2; exit 1; }

# Pull this pair's three topics from config.yaml (single source of truth).
read -r IMG INFO LID < <(python3 - "$CONFIG" "$PAIR" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
pairs = cfg.get("pairs", {})
name = sys.argv[2]
if name not in pairs:
    sys.exit(f"unknown pair '{name}'. available: {sorted(pairs)}")
p = pairs[name]
img, info, lid = p.get("image_topic"), p.get("camera_info_topic"), p.get("lidar_topic")
if not img or not lid:
    sys.exit(f"pair '{name}' missing image_topic/lidar_topic in config.yaml")
# camera_info may be null (e.g. insta360) — omit it then.
print(img, info if info and info != "None" else "", lid)
PY
) || exit 1

TOPICS=("$IMG" "$LID")
[ -n "$INFO" ] && TOPICS+=("$INFO")
echo "[record_cam_lidar] pair=$PAIR topics: ${TOPICS[*]}"
[ -z "$INFO" ] && echo "  NOTE: no camera_info topic for this pair — you must" \
                       "supply intrinsics to the calibrator separately."

echo "[1/2] topic liveness (10 s discovery window)"
AVAILABLE=""
for _ in 1 2 3 4 5; do
    AVAILABLE=$(ros2 topic list 2>/dev/null)
    miss=0
    for t in "${TOPICS[@]}"; do grep -qx "$t" <<<"$AVAILABLE" || miss=$((miss+1)); done
    [ "$miss" -eq 0 ] && break
    sleep 2
done
MISSING=0
for t in "${TOPICS[@]}"; do
    if ! grep -qx "$t" <<<"$AVAILABLE"; then
        echo "  MISSING: $t"; MISSING=$((MISSING+1))
    fi
done
if [ "$MISSING" -gt 0 ]; then
    echo "  $MISSING topic(s) absent. Start the driver(s), e.g.:"
    echo "    ros2 launch mmb_bringup cameras.launch.py"
    read -r -p "  Record anyway? [y/N] " yn
    [[ "$yn" =~ ^[Yy] ]] || exit 1
else
    echo "  all topics present"
fi

echo "[2/2] output dir"
mkdir -p "$OUT_DIR"
BAG="$OUT_DIR/${PAIR}_$(date +%Y%m%d_%H%M%S)"

echo
echo "[record_cam_lidar] recording to $BAG  (Ctrl-C to stop)"
echo "  hold the board static 3-5 s per pose, 15-25 well-spread poses"
ros2 bag record --storage mcap --output "$BAG" "${TOPICS[@]}"

echo
echo "[record_cam_lidar] done. Calibrate with:"
echo "  cd $HERE/../extrinsic_calibration/camera_lidar"
echo "  python3 calibrate_cam_lidar.py --bag $BAG --pair $PAIR --dry-run --save-debug dbg/"
