#!/usr/bin/env bash
# Interactive camera<->LiDAR pair recording: starts the pair's drivers + the
# recorder, shows a live status line, and stops cleanly on EITHER Ctrl+C OR
# typing `stop` <Enter>. On stop it finalizes the bag (SIGINT to the recorder so
# the MCAP closes), then SIGKILLs the drivers (the Hesai ignores SIGINT), and
# prints the final per-topic frame counts.
#
# Stopping the drivers between recordings is intentional/good: it frees the
# cameras + LiDARs so the next pair's drivers can grab them without contention.
#
# Shared engine; the per-pair wrappers (record_oak4d.sh, ...) just call it.
#     bash record_pair.sh <pair> [out_dir]      <pair>: oak4d oak1 oakd_lite realsense insta360
#
# Topics come from extrinsic_calibration/camera_lidar/config.yaml. Protocol:
# hold the ChArUco board STATIC 3-5 s per pose, 15-25 well-spread poses.
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
NTOP=$(wc -w <<<"$TOPICS")
[ -z "$INFO" ] && echo "[record_pair] NOTE: $PAIR has no camera_info topic —" \
                       "supply intrinsics to the calibrator separately."

mkdir -p "$OUT_DIR"
BAG="$OUT_DIR/${PAIR}_$(date +%Y%m%d_%H%M%S)"
LOG="$BAG.launch.log"

echo "================================================================"
echo " RECORDING pair: $PAIR"
echo "   topics : $TOPICS"
echo "   bag    : $BAG"
echo "   driver log: $LOG"
echo "================================================================"

# Drivers + recorder in their own session so we can tear the whole tree down.
setsid ros2 launch mmb_bringup pair_record.launch.py \
    pair:="$PAIR" topics:="$TOPICS" bag_name:="$BAG" </dev/null >"$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 1
LAUNCH_PGID=$(ps -o pgid= "$LAUNCH_PID" 2>/dev/null | tr -d ' ')

stop_all() {
    trap - INT TERM
    echo; echo "[record_pair] stopping — finalizing bag (SIGINT to recorder)..."
    pkill -INT -f "ros2 bag record" 2>/dev/null
    for _ in $(seq 1 20); do pgrep -f "ros2 bag record" >/dev/null || break; sleep 0.5; done
    echo "[record_pair] stopping drivers (SIGKILL)..."
    [ -n "${LAUNCH_PGID:-}" ] && kill -9 -"$LAUNCH_PGID" 2>/dev/null
    pkill -9 -f hesai_ros_driver_node 2>/dev/null
    pkill -9 -f "depthai_ros_driver_v3/lib" 2>/dev/null
    pkill -9 -f "depthai_ros_driver/lib" 2>/dev/null
    pkill -9 -f realsense2_camera_node 2>/dev/null
    pkill -9 -f livox 2>/dev/null
    pkill -9 -f pair_record.launch 2>/dev/null
    sleep 1
    echo; echo "[record_pair] final bag contents:"
    ros2 bag info "$BAG" 2>/dev/null \
        | grep -iE "Duration|^Messages|Topic:|Count" | sed 's/^/  /' \
        || echo "  (ros2 bag info failed — check $BAG)"
    echo "[record_pair] done: $BAG"
    exit 0
}
trap stop_all INT TERM

# Wait for the recorder to subscribe (drivers warm up ~6 s).
echo "[record_pair] starting drivers + recorder (warming up)..."
for _ in $(seq 1 20); do
    [ "$(grep -c 'Subscribed to topic' "$LOG" 2>/dev/null || echo 0)" -gt 0 ] && break
    sleep 2
done
NSUB=$(grep -c 'Subscribed to topic' "$LOG" 2>/dev/null || echo 0)
if [ "$NSUB" -eq 0 ]; then
    echo "[record_pair] WARNING: recorder not subscribed yet — a driver may have"
    echo "  failed. Last log lines:"; tail -6 "$LOG" 2>/dev/null | sed 's/^/    /'
fi
echo
echo ">>> RECORDING ($NSUB/$NTOP topics live). Hold the board: 15-25 static"
echo ">>> poses, 3-5 s each, visible to BOTH camera and LiDAR."
echo ">>> Stop with Ctrl+C  OR  type 'stop' then Enter."
echo

# Live status; `read -t 3` doubles as the tick and the 'stop' listener.
START=$(date +%s)
while true; do
    if IFS= read -r -t 3 line; then
        case "$line" in stop|STOP|s|q|quit) stop_all ;; esac
    fi
    EL=$(( $(date +%s) - START ))
    SIZE=$(du -sh "$BAG" 2>/dev/null | cut -f1); SIZE=${SIZE:-0}
    NSUB=$(grep -c 'Subscribed to topic' "$LOG" 2>/dev/null || echo 0)
    printf "\r[REC %4ds]  %s/%s topics recording  |  bag %-6s  |  Ctrl+C or 'stop'+Enter " \
        "$EL" "$NSUB" "$NTOP" "$SIZE"
done
