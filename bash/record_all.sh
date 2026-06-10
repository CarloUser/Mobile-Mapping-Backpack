#!/usr/bin/env bash
# All-sensor recording with pre-flight checks. Run AFTER the drivers are up
# (ros2 launch mmb_bringup all_sensors.launch.py, ideally in tmux).
#
# Usage:  bash record_all.sh <location>_<run_type>_<run_number> [out_dir]
#         bash record_all.sh campus_mapping_01
#         bash record_all.sh hall_calib_02 /data/bags
#
# Checks before recording starts:
#   1. system clock NTP-synchronized (chrony/timedatectl)
#   2. every topic in mmb_bringup/config/topics.yaml has a live publisher
#   3. enough free disk space for the expected duration
# Then records all topics (MCAP) and prints the post-run verification command.
RUN_NAME="${1:?usage: record_all.sh <location>_<runtype>_<nn> [out_dir]}"
OUT_DIR="${2:-$HOME/recordings/mapping}"
MIN_FREE_GB=20

# ROS setup scripts reference unbound variables — source before `set -u`.
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
set -uo pipefail

TOPICS_YAML="$(ros2 pkg prefix mmb_bringup)/share/mmb_bringup/config/topics.yaml"
mapfile -t TOPICS < <(grep -oE '^\s+-\s+(/\S+)' "$TOPICS_YAML" | awk '{print $2}')
echo "[record_all] ${#TOPICS[@]} topics from $TOPICS_YAML"

echo "[1/3] clock sync"
if command -v chronyc >/dev/null && chronyc tracking >/dev/null 2>&1; then
    OFF=$(chronyc tracking | awk '/System time/ {print $4}')
    echo "  chrony system-time offset: ${OFF}s"
elif timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -q yes; then
    echo "  WARNING: chrony not running, but systemd NTP reports synchronized."
    echo "  PTP for the LiDARs needs the chrony/linuxptp stack:"
    echo "  sudo bash bash/setup_time_sync.sh <lidar_iface>"
else
    echo "  WARNING: system clock NOT NTP-synchronized. Timestamps will be"
    echo "  internally consistent but not UTC-aligned. Continuing in 5 s..."
    sleep 5
fi

echo "[2/3] topic liveness (10 s discovery window)"
MISSING=0
AVAILABLE=$(ros2 topic list 2>/dev/null)
for i in 1 2 3 4 5; do
    AVAILABLE=$(ros2 topic list 2>/dev/null)
    n_missing=0
    for t in "${TOPICS[@]}"; do
        grep -qx "$t" <<<"$AVAILABLE" || n_missing=$((n_missing+1))
    done
    [ "$n_missing" -eq 0 ] && break
    sleep 2
done
for t in "${TOPICS[@]}"; do
    if ! grep -qx "$t" <<<"$AVAILABLE"; then
        echo "  MISSING: $t"
        MISSING=$((MISSING+1))
    fi
done
if [ "$MISSING" -gt 0 ]; then
    echo "  $MISSING topic(s) absent. Start the drivers first:"
    echo "    ros2 launch mmb_bringup all_sensors.launch.py"
    read -r -p "  Record anyway, without them? [y/N] " yn
    [[ "$yn" =~ ^[Yy] ]] || exit 1
else
    echo "  all topics present"
fi

echo "[3/3] disk space"
mkdir -p "$OUT_DIR"
FREE_GB=$(df -BG --output=avail "$OUT_DIR" | tail -1 | tr -dc '0-9')
echo "  ${FREE_GB} GB free in $OUT_DIR (rule of thumb ~2 GB/min all-sensors)"
if [ "$FREE_GB" -lt "$MIN_FREE_GB" ]; then
    read -r -p "  Less than ${MIN_FREE_GB} GB free. Continue? [y/N] " yn
    [[ "$yn" =~ ^[Yy] ]] || exit 1
fi

BAG="$OUT_DIR/mmb_$(date +%Y%m%d_%H%M)_${RUN_NAME}"
echo
echo "[record_all] recording to $BAG  (Ctrl-C to stop)"
ros2 bag record --storage mcap --output "$BAG" "${TOPICS[@]}"

echo
echo "[record_all] done. Verify:"
echo "  ros2 bag info $BAG"
echo "  python3 $(dirname "$0")/../evaluation/check_time_sync.py --bag $BAG"
