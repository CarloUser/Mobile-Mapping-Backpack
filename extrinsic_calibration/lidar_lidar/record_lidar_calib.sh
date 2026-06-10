#!/usr/bin/env bash
# record_lidar_calib.sh -- one-command recording for the Hesai<->Livox calibration.
#
# Launches both LiDAR drivers, waits for all four topics, verifies the Livox is in
# CustomMsg mode (FAST-LIO needs per-point time), records for a fixed duration,
# then closes the bag cleanly and shuts the drivers down.
#
# Run it INSIDE tmux so it survives an SSH/Wi-Fi drop while you walk:
#     tmux new -s rec
#     ./record_lidar_calib.sh            # default 240 s (4 min)
#     ./record_lidar_calib.sh 300        # custom duration in seconds
#     Ctrl-b d   (detach, walk the rig), then `tmux attach -t rec` afterwards
#
# Why a fixed duration and not "auto-stop when enough data": judging motion quality
# live needs odometry running during the walk (fragile on a backpack). 2-5 min of
# good 3-axis motion is the sweet spot; longer adds FAST-LIO drift, shorter risks
# too few hand-eye pairs. The real sufficiency check is AFTER: solve_extrinsic.py's
# rotation-observability gate tells you if the walk was rich enough -- if it warns,
# just re-run this script and walk with more pitch/roll.

# NOTE: no `set -u` -- ROS 2 setup.bash references unset vars when sourced.
set -o pipefail

# ===================== EDIT IF YOUR PATHS DIFFER =====================
# This rig builds all drivers into one workspace (~/ros2_ws, per bash/build_sensors.sh).
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"          # combined driver workspace
HESAI_LAUNCH="ros2 launch hesai_ros_driver start.py"
LIVOX_LAUNCH="ros2 launch livox_ros2_avia livox_lidar_msg_launch.py"   # CustomMsg launch!
# ====================================================================

DURATION="${1:-240}"                       # seconds (2-5 min recommended)
OUTDIR="${OUTDIR:-$HOME/recordings/lidar}"
TOPICS=(/lidar_points /lidar_imu /livox/lidar /livox/imu)
LIVOX_CLOUD="/livox/lidar"
PRE_ROLL=8                                  # countdown before recording starts
LOGDIR="$(mktemp -d)"
HESAI_PID=""; LIVOX_PID=""

cleanup() {
  echo; echo "[record] shutting down drivers..."
  [[ -n "$LIVOX_PID" ]] && kill -INT "$LIVOX_PID" 2>/dev/null
  [[ -n "$HESAI_PID" ]] && kill -INT "$HESAI_PID" 2>/dev/null
  sleep 3
  [[ -n "$LIVOX_PID" ]] && kill -9 "$LIVOX_PID" 2>/dev/null
  [[ -n "$HESAI_PID" ]] && kill -9 "$HESAI_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

# --- source ROS + driver workspace ---
source "$ROS_SETUP" || { echo "[record] cannot source $ROS_SETUP"; exit 1; }
if [[ -f "$WS_SETUP" ]]; then
  source "$WS_SETUP"
else
  echo "[record] driver workspace not found at: $WS_SETUP"
  echo "         find it with:  find \$HOME -maxdepth 3 -name setup.bash -path '*install*'"
  exit 1
fi
# Sanity: both driver packages must resolve.
for pkg in hesai_ros_driver livox_ros2_avia; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || { echo "[record] package '$pkg' not found after sourcing $WS_SETUP"; exit 1; }
done

# Refresh the ROS 2 daemon: a stale daemon's discovery cache has caused the topic
# checks below to miss live publishers (seen as !rclpy.ok() errors).
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

# --- launch drivers ---
echo "[record] launching Hesai driver (log: $LOGDIR/hesai.log)"
$HESAI_LAUNCH >"$LOGDIR/hesai.log" 2>&1 & HESAI_PID=$!
echo "[record] launching Livox driver in CustomMsg mode (log: $LOGDIR/livox.log)"
$LIVOX_LAUNCH >"$LOGDIR/livox.log" 2>&1 & LIVOX_PID=$!

# --- wait for every topic to actually publish ---
wait_topic() {
  local t="$1" to="${2:-40}" start=$SECONDS
  echo -n "[record] waiting for $t ..."
  while (( SECONDS - start < to )); do
    # best_effort subscriber is compatible with both best_effort and reliable
    # publishers (LiDAR/IMU sensor data is published best-effort).
    if timeout 5 ros2 topic echo "$t" --once --qos-reliability best_effort >/dev/null 2>&1; then echo " ok"; return 0; fi
  done
  echo " TIMEOUT"; return 1
}
echo "[record] giving drivers a few seconds to initialise..."
sleep 8
for t in "${TOPICS[@]}"; do
  wait_topic "$t" 60 || { echo "[record] ERROR: $t not publishing. Check $LOGDIR/*.log"; exit 1; }
done

# --- hard-check: Livox must be CustomMsg, not PointCloud2 ---
LTYPE="$(ros2 topic type "$LIVOX_CLOUD" 2>/dev/null)"
echo "[record] $LIVOX_CLOUD type: ${LTYPE:-<unknown>}"
if [[ "$LTYPE" != *CustomMsg* ]]; then
  echo "[record] ERROR: $LIVOX_CLOUD is '$LTYPE', not a CustomMsg type."
  echo "         You launched the PointCloud2 driver. Use livox_lidar_msg_launch.py."
  exit 1
fi

# --- record ---
mkdir -p "$OUTDIR"
BAG="$OUTDIR/lidar_calib_imu_$(date +%Y%m%d_%H%M%S)"
echo
echo "=================================================================="
echo " All four topics live, Livox is CustomMsg. Recording ${DURATION}s."
echo " WALK: figure-eights + a ramp/stairs (pitch) + side-to-side tilt"
echo " (roll) + up/down nod. Smooth motions. Feature-rich room."
echo "=================================================================="
for ((i=PRE_ROLL; i>0; i--)); do printf "\r  starting in %2ds " "$i"; sleep 1; done
printf "\r  RECORDING NOW -- WALK!        \n"; printf '\a'

timeout --signal=INT "$DURATION" \
  ros2 bag record -o "$BAG" "${TOPICS[@]}" >"$LOGDIR/record.log" 2>&1
RC=$?
printf '\a'
if [[ $RC -ne 0 && $RC -ne 124 ]]; then
  echo "[record] WARNING: recorder exited with code $RC. Check $LOGDIR/record.log"
fi

echo
echo "[record] DONE. Bag saved to:"
echo "         $BAG"
echo "[record] verify:   ros2 bag info \"$BAG\""
echo "[record] timing:   python3 inspect_bag_timing.py --bag \"$BAG\" --config config.yaml"
echo "[record] (drivers will now shut down)"
