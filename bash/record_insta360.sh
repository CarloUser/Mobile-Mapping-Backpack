#!/usr/bin/env bash
# Record the Insta360 <-> Hesai JT128 pair. Connect both, run this, Ctrl-C to
# stop. NOTE: the Insta360 stream is H.264 dual-fisheye with NO camera_info, so
# this bag is for the Koide targetless flow (see camera_lidar/KOIDE_CROSSCHECK.md),
# not the direct ChArUco calibrator. Thin wrapper around record_pair.sh.
exec "$(cd "$(dirname "$0")" && pwd)/record_pair.sh" insta360 "$@"
