#!/usr/bin/env bash
# One-shot camera<->LiDAR pair recording: start the pair's drivers, warm up,
# then record its bag. Ctrl-C stops recording AND the drivers together.
#
# This is the shared engine; the per-pair wrappers (record_oak4d.sh, ...) just
# call it with their pair name. You can also call it directly:
#     bash record_pair.sh <pair> [out_dir]
#     bash record_pair.sh oak4d
#     bash record_pair.sh realsense /data/bags
#   <pair>: oak4d, oak1, oakd_lite, realsense, insta360
#
# Topics are read from extrinsic_calibration/camera_lidar/config.yaml (single
# source of truth) and passed to launch/pair_record.launch.py, which owns the
# driver mapping. Recording protocol: hold the ChArUco board STATIC 3-5 s per
# pose, 15-25 well-spread poses (vary distance, angle, board-normal direction).
PAIR="${1:?usage: record_pair.sh <pair> [out_dir]}"
OUT_DIR="${2:-$HOME/recordings/camlidar}"

# ROS setup scripts use unbound vars — source before `set -u`. ~/ros2_ws gives
# the livox_interfaces typesupport (else /livox/lidar records zero messages).
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
CONFIG="$HERE/../extrinsic_calibration/camera_lidar/config.yaml"
[ -f "$CONFIG" ] || { echo "config not found: $CONFIG" >&2; exit 1; }

# Pull this pair's topics from config.yaml; null camera_info (insta360) omitted.
read -r IMG INFO LID < <(python3 - "$CONFIG" "$PAIR" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1])); pairs = cfg.get("pairs", {})
name = sys.argv[2]
if name not in pairs:
    sys.exit(f"unknown pair '{name}'. available: {sorted(pairs)}")
p = pairs[name]
img, info, lid = p.get("image_topic"), p.get("camera_info_topic"), p.get("lidar_topic")
if not img or not lid:
    sys.exit(f"pair '{name}' missing image_topic/lidar_topic in config.yaml")
print(img, info if info and info != "None" else "", lid)
PY
) || exit 1

TOPICS="$IMG $LID"
[ -n "$INFO" ] && TOPICS="$TOPICS $INFO"
[ -z "$INFO" ] && echo "[record_pair] NOTE: $PAIR has no camera_info topic —" \
                       "supply intrinsics to the calibrator separately."

mkdir -p "$OUT_DIR"
BAG="$OUT_DIR/${PAIR}_$(date +%Y%m%d_%H%M%S)"

echo "[record_pair] pair=$PAIR"
echo "  topics : $TOPICS"
echo "  bag    : $BAG"
echo "  starting drivers + recorder (Ctrl-C to stop both)..."
echo "  >>> hold the board static 3-5 s per pose, 15-25 well-spread poses <<<"

# ros2 launch owns the drivers + recorder; Ctrl-C tears them all down cleanly.
exec ros2 launch mmb_bringup pair_record.launch.py \
    pair:="$PAIR" topics:="$TOPICS" bag_name:="$BAG"
