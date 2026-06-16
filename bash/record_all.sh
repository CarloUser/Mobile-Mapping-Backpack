#!/usr/bin/env bash
# Record ALL sensors on ONE Jetson: launches every sensor driver
# (all_sensors.launch.py) AND records every topic in topics.yaml to a single
# MCAP bag.
#
# Two ways to use it:
#
#   DETACHED (works under Claude Code `!` and over plain ssh — each command
#   returns immediately; you start, walk, then stop from a later prompt):
#     bash record_all.sh start <run_name> [out_dir]
#     bash record_all.sh status
#     bash record_all.sh stop
#
#   INTERACTIVE (a real terminal; live status line, stop with Ctrl+C or 'stop'):
#     bash record_all.sh <run_name> [out_dir]
#
# It records whatever drivers actually come up — a sensor whose driver is
# missing/failing just won't appear (status shows N/total; the driver log lists
# failures). Driver log: <bag>.drivers.log   Recorder log: <bag>.record.log

set -uo pipefail
MIN_FREE_GB=20
WARMUP_S=12
STATE_DIR="$HOME/recordings/mapping/.record_all_state"
STATE="$STATE_DIR/run.env"

# ROS setup scripts reference unbound vars, so relax `set -u` while sourcing.
# ~/ros2_ws gives the livox_interfaces typesupport (else /livox/lidar records
# zero messages). Insta360 needs its bundled libCameraSDK.so on the lib path.
ros_env() {
    set +u
    source /opt/ros/humble/setup.bash
    source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
    set -u
    export LD_LIBRARY_PATH="$HOME/ros2_ws/src/insta360_ros_driver/lib:${LD_LIBRARY_PATH:-}"
}

load_topics() {
    TOPICS_YAML="$(ros2 pkg prefix mmb_bringup)/share/mmb_bringup/config/topics.yaml"
    [ -f "$TOPICS_YAML" ] || { echo "topics.yaml not found: $TOPICS_YAML (build mmb_bringup)"; exit 1; }
    mapfile -t TOPICS < <(grep -oE '^\s+-\s+(/\S+)' "$TOPICS_YAML" | awk '{print $2}')
    NTOP=${#TOPICS[@]}
}

# grep -c prints "0" AND exits non-zero on no match, so a `|| echo 0` would
# emit a second line. Capture once and default if the file is missing entirely.
nsub() { local n; n=$(grep -c 'Subscribed to topic' "$1" 2>/dev/null); echo "${n:-0}"; }

# Bring up drivers (own session) then, after warm-up, the recorder (own
# session). setsid detaches both into their own sessions so they survive this
# script exiting — that is what makes the detached start/stop model work.
# Sets: BAG DLOG RLOG DRV_PGID REC_PGID START
launch_stack() {
    local RUN="$1" OUT_DIR="$2"
    mkdir -p "$OUT_DIR"
    BAG="$OUT_DIR/mmb_all_$(date +%Y%m%d_%H%M%S)_${RUN}"
    DLOG="$BAG.drivers.log"; RLOG="$BAG.record.log"

    echo "================================================================"
    echo " RECORD ALL SENSORS  (one Jetson)"
    echo "   topics : $NTOP  (from topics.yaml)"
    echo "   bag    : $BAG"
    echo "   logs   : $DLOG  (drivers) / $RLOG (recorder)"
    echo "================================================================"

    local FREE_GB
    FREE_GB=$(df -BG --output=avail "$OUT_DIR" | tail -1 | tr -dc '0-9')
    echo "[disk] ${FREE_GB} GB free in $OUT_DIR  (~2 GB/min all-sensors)"
    if [ "${FREE_GB:-0}" -lt "$MIN_FREE_GB" ] && [ "${FORCE:-0}" != "1" ]; then
        echo "  Less than ${MIN_FREE_GB} GB free — refusing. Free space or re-run with FORCE=1." >&2
        exit 1
    fi

    echo "[1/2] launching all sensor drivers (all_sensors.launch.py)..."
    setsid ros2 launch mmb_bringup all_sensors.launch.py </dev/null >"$DLOG" 2>&1 &
    local DRV_PID=$!; sleep 1
    DRV_PGID=$(ps -o pgid= "$DRV_PID" 2>/dev/null | tr -d ' ')

    echo "[2/2] warming up ${WARMUP_S}s, then recording all topics..."
    sleep "$WARMUP_S"
    setsid ros2 bag record --storage mcap --output "$BAG" "${TOPICS[@]}" </dev/null >"$RLOG" 2>&1 &
    local REC_PID=$!; sleep 1
    REC_PGID=$(ps -o pgid= "$REC_PID" 2>/dev/null | tr -d ' ')

    # wait until the recorder subscribes to at least one topic
    local _
    for _ in $(seq 1 15); do
        [ "$(nsub "$RLOG")" -gt 0 ] && break
        sleep 1
    done
    START=$(date +%s)
}

# Finalize the bag (SIGINT the recorder so the MCAP closes cleanly) then tear
# down the driver tree. Reads BAG / DRV_PGID from the caller's environment.
stop_stack() {
    echo; echo "[record_all] stopping — finalizing bag (SIGINT recorder)..."
    pkill -INT -f "ros2 bag record" 2>/dev/null
    local _
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
}

print_status() {  # args: START RLOG BAG NTOP
    local START="$1" RLOG="$2" BAG="$3" NTOP="$4" EL N SIZE
    EL=$(( $(date +%s) - START ))
    N=$(nsub "$RLOG")
    SIZE=$(du -sh "$BAG" 2>/dev/null | cut -f1); SIZE=${SIZE:-0}
    printf '[REC %5ds]  %s/%s topics recording  |  bag %-7s\n' "$EL" "$N" "$NTOP" "$SIZE"
}

cmd_start() {
    local RUN="${1:?usage: record_all.sh start <run_name> [out_dir]}"
    local OUT_DIR="${2:-$HOME/recordings/mapping}"
    if [ -f "$STATE" ]; then
        # shellcheck disable=SC1090
        source "$STATE"
        if kill -0 "-${DRV_PGID:-0}" 2>/dev/null || pgrep -f "ros2 bag record" >/dev/null; then
            echo "A recording is already active (bag: ${BAG:-?}). Run 'record_all.sh stop' first." >&2
            exit 1
        fi
    fi
    ros_env; load_topics
    launch_stack "$RUN" "$OUT_DIR"
    mkdir -p "$STATE_DIR"
    cat >"$STATE" <<EOF
BAG="$BAG"
DLOG="$DLOG"
RLOG="$RLOG"
DRV_PGID="$DRV_PGID"
REC_PGID="$REC_PGID"
START="$START"
NTOP="$NTOP"
EOF
    echo
    echo ">>> RECORDING STARTED (detached) — $(nsub "$RLOG")/$NTOP topics live."
    [ "$(nsub "$RLOG")" -lt "$NTOP" ] && echo ">>> (some topics not up yet — check: $DLOG)"
    echo ">>> Status :  bash $0 status"
    echo ">>> Stop   :  bash $0 stop"
}

cmd_status() {
    [ -f "$STATE" ] || { echo "No active recording (no state file)."; exit 1; }
    # shellcheck disable=SC1090
    source "$STATE"
    if ! pgrep -f "ros2 bag record" >/dev/null; then
        echo "Recorder is NOT running. Last bag: ${BAG:-?}  (run 'record_all.sh stop' to clean up)"
        exit 1
    fi
    print_status "$START" "$RLOG" "$BAG" "$NTOP"
}

cmd_stop() {
    [ -f "$STATE" ] || { echo "No active recording (no state file)."; exit 1; }
    ros_env
    # shellcheck disable=SC1090
    source "$STATE"
    stop_stack
    rm -f "$STATE"
}

# Legacy interactive mode: foreground, live status line, stop via Ctrl+C/'stop'.
cmd_interactive() {
    local RUN="$1" OUT_DIR="${2:-$HOME/recordings/mapping}"
    ros_env; load_topics
    launch_stack "$RUN" "$OUT_DIR"

    local on_stop=0
    interactive_stop() { [ "$on_stop" = 1 ] && return; on_stop=1; trap - INT TERM; stop_stack; exit 0; }
    trap interactive_stop INT TERM

    echo
    echo ">>> RECORDING ($(nsub "$RLOG")/$NTOP topics live). Walk/collect your data."
    echo ">>> Stop with Ctrl+C  OR  type 'stop' then Enter."
    [ "$(nsub "$RLOG")" -lt "$NTOP" ] && echo ">>> (some topics not up yet — see $DLOG)"
    echo

    local line
    while true; do
        if IFS= read -r -t 3 line; then
            case "$line" in stop|STOP|s|q|quit) interactive_stop ;; esac
        fi
        printf '\r%s ' "$(print_status "$START" "$RLOG" "$BAG" "$NTOP" | tr -d '\n')"
    done
}

case "${1:-}" in
    start)  shift; cmd_start "$@" ;;
    status) cmd_status ;;
    stop)   cmd_stop ;;
    "")     echo "usage: record_all.sh {start <run_name> [out_dir] | status | stop}" >&2
            echo "       record_all.sh <run_name> [out_dir]   # interactive" >&2
            exit 2 ;;
    *)      cmd_interactive "$@" ;;   # legacy: first arg is the run name
esac
