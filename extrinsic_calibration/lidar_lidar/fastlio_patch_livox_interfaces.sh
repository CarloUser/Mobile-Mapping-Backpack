#!/usr/bin/env bash
# fastlio_patch_livox_interfaces.sh
# ---------------------------------------------------------------------------
# Make Ericsii/FAST_LIO_ROS2 (ros2 branch) subscribe to the CustomMsg package
# THIS rig actually publishes. The Avia driver (livox_ros2_avia) publishes
# `livox_interfaces/msg/CustomMsg`, but stock FAST-LIO is hard-coded to
# `livox_ros_driver2/msg/CustomMsg`. The two messages are field-for-field
# identical (header, timebase, point_num, lidar_id, rsvd, points[] with
# offset_time,x,y,z,reflectivity,tag,line), so the ONLY change needed is the
# package name in the includes, the namespace, and the build dependency.
#
# This script does that rename and nothing else. Run it from inside the cloned
# FAST_LIO_ROS2 directory BEFORE colcon build.
#
#   cd ~/fastlio_ws/src/FAST_LIO_ROS2
#   bash /path/to/fastlio_patch_livox_interfaces.sh
#
# To target a different package name (e.g. if `ros2 interface show` reveals a
# different one), pass it as $1:  bash fastlio_patch_livox_interfaces.sh livox_ros_driver
# ---------------------------------------------------------------------------
set -euo pipefail

FROM="livox_ros_driver2"
TO="${1:-livox_interfaces}"

# Files that reference the CustomMsg package on the ros2 branch.
FILES=(package.xml CMakeLists.txt src/preprocess.h src/preprocess.cpp src/laserMapping.cpp)

# --- sanity: are we in the right place? ---
if [[ ! -f package.xml || ! -d src ]]; then
  echo "ERROR: run this from the FAST_LIO_ROS2 root (no package.xml/src here)." >&2
  echo "       cd ~/fastlio_ws/src/FAST_LIO_ROS2 && bash $(basename "$0")" >&2
  exit 1
fi
if ! grep -q "rclcpp" package.xml 2>/dev/null; then
  echo "ERROR: this package.xml is not the ROS 2 one. You are probably on the" >&2
  echo "       'main' (ROS 1) branch. Re-clone with:  git clone -b ros2 --recursive ..." >&2
  exit 1
fi

echo "[patch] renaming '$FROM' -> '$TO'"
changed=0
for f in "${FILES[@]}"; do
  if [[ -f "$f" ]] && grep -q "$FROM" "$f"; then
    cp -n "$f" "$f.orig"                       # one-time backup
    n=$(grep -c "$FROM" "$f" || true)
    sed -i "s/${FROM}/${TO}/g" "$f"
    echo "   $f: $n occurrence(s) replaced (backup: $f.orig)"
    changed=$((changed + 1))
  fi
done

if [[ "$changed" -eq 0 ]]; then
  echo "[patch] nothing to change. Either already patched, or '$FROM' not present."
  echo "        Remaining references to any 'livox_ros_driver' package:"
  grep -rn "livox_ros_driver" . --include=*.cpp --include=*.h --include=*.hpp \
      --include=package.xml --include=CMakeLists.txt 2>/dev/null || echo "        (none)"
  exit 0
fi

echo
echo "[patch] done. Verify no stray references remain:"
grep -rn "$FROM" . --include=*.cpp --include=*.h --include=*.hpp \
    --include=package.xml --include=CMakeLists.txt 2>/dev/null \
    && echo "   ^^ still referencing $FROM (investigate)" \
    || echo "   clean: no remaining '$FROM' references."

cat <<'EOF'

Next:
  1. Make sure your driver workspace (with livox_interfaces) is sourced so the
     build can find the package:
        source ~/ros2_ws/install/setup.bash
     Confirm the message is visible and field names match:
        ros2 interface show livox_interfaces/msg/CustomMsg
        ros2 interface show livox_interfaces/msg/CustomPoint
     (Expect CustomPoint fields: offset_time, x, y, z, reflectivity, tag, line.
      If a field name differs, the C++ field access in preprocess.cpp must be
      edited too -- tell me and I'll hand you the exact lines.)
  2. Build (rosdep can't resolve the local interface pkg, so skip it):
        cd ~/fastlio_ws
        rosdep install --from-paths src --ignore-src -y --skip-keys livox_interfaces
        colcon build --symlink-install
        source install/setup.bash
EOF
