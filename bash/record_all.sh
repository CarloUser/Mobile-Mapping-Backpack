#!/usr/bin/env bash
# Record ALL sensors on ONE Jetson, in one command: launches every sensor driver
# (all_sensors.launch.py) AND records every topic in topics.yaml to a single MCAP
# bag. Shows a live status line; stop with Ctrl+C OR by typing `stop` <Enter>.
# On stop it finalizes the bag (SIGINT to the recorder so the MCAP closes), then
# SIGKILLs the drivers.
#
#   bash record_all.sh <run_name> [out_dir]
#   bash record_all.sh campus_lab_01
#   bash record_all.sh hall_02 /data/bags
#
# It records whatever drivers actually come up — any sensor whose driver is
# missing/failing simply won't appear (the status line shows N/total recording,
# and the driver log lists failures). Driver log: <bag>.drivers.log.
RUN="${1:?usage: record_all.sh <run_name> [out_dir]}"
OUT_DIR="${2:-$HOME/recordings/mapping}"
MIN_FREE_GB=20
WARMUP_S=12

# ROS setup scripts use unbound vars — source before `set -u`. ~/ros2_ws gives
# the livox_interfaces typesupport (else /livox/lidar records zero messages).
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
set -uo pipefail

# Insta360 driver needs its bundled libCameraSDK.so on the lib path.
export LD_LIBRARY_PATH="$HOME/ros2_ws/src/insta360_ros_driver/lib:${LD_LIBRARY_PATH:-}"

TOPICS_YAML="$(ros2 pkg prefix mmb_bringup)/share/mmb_bringup/config/topics.yaml"
[ -f "$TOPICS_YAML" ] || { echo "topics.yaml not found: $TOPICS_YAML (build mmb_bringup)"; exit 1; }
mapfile -t TOPICS < <(grep -oE '^\s+-\s+(/\S+)' "$TOPICS_YAML" | awk '{print $2}')
NTOP=${#TOPICS[@]}

mkdir -p "$OUT_DIR"
BAG="$OUT_DIR/mmb_all_$(date +%Y%m%d_%H%M%S)_${RUN}"
DLOG="$BAG.drivers.log"
RLOG="$BAG.record.log"

echo "================================================================"
echo " RECORD ALL SENSORS  (one Jetson)"
echo "   topics : $NTOP  (from topics.yaml)"
echo "   bag    : $BAG"
echo "   logs   : $DLOG  (drivers) / $RLOG (recorder)"
echo "================================================================"

FREE_GB=$(df -BG --output=avail "$OUT_DIR" | tail -1 | tr -dc '0-9')
echo "[disk] ${FREE_GB} GB free in $OUT_DIR  (~2 GB/min all-sensors)"
if [ "$FREE_GB" -lt "$MIN_FREE_GB" ]; then
    read -r -p "  Less than ${MIN_FREE_GB} GB free. Continue? [y/N] " yn
    [[ "$yn" =~ ^[Yy] ]] || exit 1
fi

# 1) all sensor drivers, in their own session so we can tear the tree down.
echo "[1/2] launching all sensor drivers (all_sensors.launch.py)..."
setsid ros2 launch mmb_bringup all_sensors.launch.py </dev/null >"$DLOG" 2>&1 &
DRV_PID=$!; sleep 1
DRV_PGID=$(ps -o pgid= "$DRV_PID" 2>/dev/null | tr -d ' ')

# 2) after warm-up, the recorder (also its own session).
echo "[2/2] warming up ${WARMUP_S}s, then recording all topics..."
sleep "$WARMUP_S"
setsid ros2 bag record --storage mcap --output "$BAG" "${TOPICS[@]}" </dev/null >"$RLOG" 2>&1 &
REC_PID=$!; sleep 1
REC_PGID=$(ps -o pgid= "$REC_PID" 2>/dev/null | tr -d ' ')

stop_all() {
    trap - INT TERM
    echo; echo "[record_all] stopping — finalizing bag (SIGINT recorder)..."
    pkill -INT -f "ros2 bag record" 2>/dev/null
    for _ in $(seq 1 30); do pgrep -f "ros2 bag record" >/dev/null || break; sleep 0.5; done
    echo "[record_all] stopping drivers (SIGKILL)..."
    [ -n "${DRV_PGID:-}" ] && kill -9 -"$DRV_PGID" 2>/dev/null
    for pat in all_sensors.launch hesai_ros_driver_node "depthai_ros_driver_v3/lib" \
               "depthai_ros_driver/lib" realsense2_camera_node insta360_ros_driver \
               livox xsens ublox robot_state_publisher; do
        pkill -9 -f "$pat" 2>/dev/null
    done
    sleep 1
    echo; echo "[record_all] final bag contents:"
    ros2 bag info "$BAG" 2>/dev/null | grep -iE "Duration|^Messages|Topic:|Count" | sed 's/^/  /' \
        || echo "  (ros2 bag info failed — check $BAG)"
    echo "[record_all] done: $BAG"
    echo "  QA: python3 $(cd "$(dirname "$0")" && pwd)/../evaluation/check_time_sync.py --bag $BAG"
    exit 0
}
trap stop_all INT TERM

# wait until the recorder subscribes to at least one topic
for _ in $(seq 1 15); do
    [ "$(grep -c 'Subscribed to topic' "$RLOG" 2>/dev/null || echo 0)" -gt 0 ] && break
    sleep 1
done
NSUB=$(grep -c 'Subscribed to topic' "$RLOG" 2>/dev/null || echo 0)
echo
echo ">>> RECORDING ($NSUB/$NTOP topics live). Walk/collect your data."
echo ">>> Stop with Ctrl+C  OR  type 'stop' then Enter."
[ "$NSUB" -lt "$NTOP" ] && echo ">>> ($((NTOP-NSUB)) topic(s) not up yet — see $DLOG for any driver that failed)"
echo

# Live status; `read -t 3` doubles as the tick and the 'stop' listener.
START=$(date +%s)
while true; do
    if IFS= read -r -t 3 line; then
        case "$line" in stop|STOP|s|q|quit) stop_all ;; esac
    fi
    EL=$(( $(date +%s) - START ))
    NSUB=$(grep -c 'Subscribed to topic' "$RLOG" 2>/dev/null || echo 0)
    SIZE=$(du -sh "$BAG" 2>/dev/null | cut -f1); SIZE=${SIZE:-0}
    printf "\r[REC %5ds]  %s/%s topics recording  |  bag %-7s  |  Ctrl+C or 'stop'+Enter " \
        "$EL" "$NSUB" "$NTOP" "$SIZE"
done
