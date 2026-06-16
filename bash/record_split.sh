#!/usr/bin/env bash
# Record THIS Jetson's half of the split rig. Run on EACH Jetson (one terminal
# each); with PTP-synced clocks they need not start at the same instant — just
# overlap, then align the two bags by timestamp in post.
#
# Usage:  bash record_split.sh <lidar|camera> <run_name> [out_dir]
#         bash record_split.sh lidar  campus_01
#         bash record_split.sh camera campus_01 /data/bags
#
# Pre-flight: PTP clock health, topic liveness, free disk. Records the role's
# topics_<role>_jetson.yaml (MCAP). Drivers must already be up, e.g.:
#   LiDAR Jetson : ros2 launch mmb_bringup lidar.launch.py  + the one camera
#   Camera Jetson: ros2 launch mmb_bringup cameras.launch.py + imu_gnss.launch.py
ROLE="${1:?usage: record_split.sh <lidar|camera> <run_name> [out_dir]}"
RUN_NAME="${2:?usage: record_split.sh <lidar|camera> <run_name> [out_dir]}"
OUT_DIR="${3:-$HOME/recordings/mapping}"
MIN_FREE_GB=20
case "$ROLE" in lidar|camera) ;; *) echo "role must be lidar|camera"; exit 1;; esac

# ROS setup scripts use unbound vars — source before `set -u`. ~/ros2_ws gives
# the livox_interfaces typesupport (else /livox/lidar records zero messages).
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
set -uo pipefail

# Isolate each Jetson's ROS graph so the two don't cross-discover over the
# switch. Keep these two values consistent across runs.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$([ "$ROLE" = lidar ] && echo 41 || echo 42)}"
echo "[record_split] role=$ROLE  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

TOPICS_YAML="$(ros2 pkg prefix mmb_bringup)/share/mmb_bringup/config/topics_${ROLE}_jetson.yaml"
[ -f "$TOPICS_YAML" ] || { echo "topic file not found: $TOPICS_YAML (rebuild mmb_bringup?)"; exit 1; }
mapfile -t TOPICS < <(grep -oE '^\s+-\s+(/\S+)' "$TOPICS_YAML" | awk '{print $2}')
echo "[record_split] ${#TOPICS[@]} topics from $(basename "$TOPICS_YAML")"

# Non-interactive mode (set MMB_YES=1, e.g. when driven over SSH by
# record_dual.sh): proceed past warnings instead of prompting.
confirm_or_exit() {   # $1 = prompt text
    if [ "${MMB_YES:-0}" = 1 ]; then
        echo "  [MMB_YES] proceeding despite the warning above"; return 0
    fi
    read -r -p "$1" yn; [[ "$yn" =~ ^[Yy] ]] || exit 1
}

echo "[1/3] PTP clock health"
EXPECT_ROLE=$([ "$ROLE" = lidar ] && echo slave || echo master)
if systemctl is-active --quiet mmb-ptp4l; then
    echo "  mmb-ptp4l active (expected $EXPECT_ROLE on this Jetson)"
    journalctl -u mmb-ptp4l -n 3 --no-pager 2>/dev/null \
        | grep -iE "offset|rms|master" | tail -1 | sed 's/^/  last: /' || true
    [ "$ROLE" = lidar ] && journalctl -u mmb-ptp4l -n 20 --no-pager 2>/dev/null \
        | grep -qi "selected" && echo "  slave has selected a grandmaster"
else
    echo "  WARNING: mmb-ptp4l not active. Clocks across the two Jetsons may not"
    echo "  agree -> the two bags won't fuse. Set up sync first:"
    echo "    sudo bash bash/setup_time_sync.sh $EXPECT_ROLE <iface> [master_ip]"
    confirm_or_exit "  Record anyway (un-synced)? [y/N] "
fi

echo "[2/3] topic liveness (10 s discovery window)"
AVAILABLE=""; MISSING=0
for _ in 1 2 3 4 5; do
    AVAILABLE=$(ros2 topic list 2>/dev/null)
    n=0; for t in "${TOPICS[@]}"; do grep -qx "$t" <<<"$AVAILABLE" || n=$((n+1)); done
    [ "$n" -eq 0 ] && break; sleep 2
done
for t in "${TOPICS[@]}"; do
    grep -qx "$t" <<<"$AVAILABLE" || { echo "  MISSING: $t"; MISSING=$((MISSING+1)); }
done
if [ "$MISSING" -gt 0 ]; then
    echo "  $MISSING topic(s) absent — start this Jetson's drivers first."
    confirm_or_exit "  Record anyway? [y/N] "
else
    echo "  all topics present"
fi

echo "[3/3] disk space"
mkdir -p "$OUT_DIR"
FREE_GB=$(df -BG --output=avail "$OUT_DIR" | tail -1 | tr -dc '0-9')
echo "  ${FREE_GB} GB free in $OUT_DIR"
if [ "$FREE_GB" -lt "$MIN_FREE_GB" ]; then
    confirm_or_exit "  Less than ${MIN_FREE_GB} GB free. Continue? [y/N] "
fi

BAG="$OUT_DIR/mmb_${ROLE}_$(date +%Y%m%d_%H%M)_${RUN_NAME}"
echo
echo "[record_split] recording to $BAG  (Ctrl-C to stop)"
ros2 bag record --storage mcap --output "$BAG" "${TOPICS[@]}"

echo
echo "[record_split] done. Verify:"
echo "  ros2 bag info $BAG"
echo "  python3 $(dirname "$0")/../evaluation/check_time_sync.py --bag $BAG"
echo "  # then cross-check against the OTHER Jetson's bag:"
echo "  python3 $(dirname "$0")/../evaluation/check_dual_sync.py --bag-a <lidar_bag> --bag-b <camera_bag>"
