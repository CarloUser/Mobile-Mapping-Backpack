#!/usr/bin/env bash
# Orchestrate the two-Jetson split recording from ONE terminal: start the OTHER
# Jetson's half over SSH and this Jetson's half locally, then stop both together
# on Ctrl-C. With PTP-synced clocks an exact simultaneous start isn't required;
# this just saves running two terminals.
#
# Usage:  bash record_dual.sh <run_name>
#
# Env (override as needed):
#   MMB_REMOTE      ssh target of the OTHER Jetson
#                   (default bgd-lab-03@10.183.242.124)
#   MMB_REMOTE_DIR  repo path on the remote, relative to its $HOME
#                   (default Documents/Mobile-Mapping-Backpack)
#   MMB_LOCAL_ROLE  lidar|camera for THIS Jetson (default lidar — this host is
#                   on the 192.168.1.x LiDAR subnet)
#   MMB_OUT_DIR     recordings dir on both (passed through; default per-script)
#
# Prerequisites (NOT done by this script):
#   - passwordless SSH to MMB_REMOTE  ->  ssh-copy-id <MMB_REMOTE>
#   - time sync up on both            ->  setup_time_sync.sh master|slave ...
#   - drivers up on both              ->  ros2 launch mmb_bringup ...
#   See docs/dual_jetson_recording.md.
set -uo pipefail
RUN="${1:?usage: record_dual.sh <run_name>}"
REMOTE="${MMB_REMOTE:-bgd-lab-03@10.183.242.124}"
REMOTE_DIR="${MMB_REMOTE_DIR:-Documents/Mobile-Mapping-Backpack}"
LOCAL_ROLE="${MMB_LOCAL_ROLE:-lidar}"
case "$LOCAL_ROLE" in
    lidar)  REMOTE_ROLE=camera ;;
    camera) REMOTE_ROLE=lidar  ;;
    *) echo "MMB_LOCAL_ROLE must be lidar|camera"; exit 1 ;;
esac
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT_ARG="${MMB_OUT_DIR:-}"

echo "[record_dual] local=$LOCAL_ROLE   remote=$REMOTE ($REMOTE_ROLE)   run=$RUN"

# 1. passwordless SSH must work, or we can't drive the remote.
if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE" true 2>/dev/null; then
    echo "[record_dual] passwordless SSH to $REMOTE failed. One-time setup:"
    echo "    ssh-copy-id $REMOTE        # asks for the remote password once"
    exit 1
fi

# 2. start the remote half in the background (non-interactive via MMB_YES).
echo "[record_dual] starting remote recorder on $REMOTE ..."
ssh "$REMOTE" "MMB_YES=1 bash '$REMOTE_DIR/bash/record_split.sh' $REMOTE_ROLE '$RUN' $OUT_ARG" &
SSH_PID=$!

# 3. on exit/Ctrl-C, SIGINT the remote recorder so its MCAP closes cleanly.
stop_remote() {
    trap - EXIT INT TERM
    echo; echo "[record_dual] stopping remote recorder ..."
    ssh -o ConnectTimeout=5 "$REMOTE" "pkill -INT -f 'ros2 bag record'" 2>/dev/null || true
    wait "$SSH_PID" 2>/dev/null || true
    echo "[record_dual] both stopped. QA:"
    echo "  python3 $HERE/../evaluation/check_dual_sync.py --bag-a <lidar_bag> --bag-b <camera_bag>"
}
trap stop_remote EXIT INT TERM

sleep 3   # give the remote a moment to come up before we start locally

# 4. local half in the foreground; Ctrl-C here stops it, then the trap stops the
#    remote. (Local record_split also runs non-interactively for symmetry.)
MMB_YES=1 bash "$HERE/record_split.sh" "$LOCAL_ROLE" "$RUN" $OUT_ARG
