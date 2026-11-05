#!/usr/bin/env bash
#
# Generate a single compile_commands.json at the repository root for clangd.
#
# CMake emits one database per build directory (libflyrat/build,
# flyrat_pilot/build, flyrat_play/ros2_ws/build/<pkg>), but clangd only searches
# upwards from the file being edited, so sources under libflyrat/,
# flyrat_pilot/ or flyrat_play/ros2_ws/src/ would never find them. This script
# (re)configures every build and merges the databases into
# <repo>/compile_commands.json, which clangd does find from anywhere in the tree.
#
# Usage:
#   .script/export_compile_commands.sh
#
# Re-run it after adding/removing source files or changing compile flags.
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

# 1. ROS 2 packages. Every package CMakeLists enables
#    CMAKE_EXPORT_COMPILE_COMMANDS itself.
if command -v colcon >/dev/null 2>&1 && [ -f "${ROS_SETUP}" ]; then
    echo "[compile_commands] configuring/building flyrat_play/ros2_ws"
    (
        set +u
        # shellcheck disable=SC1090
        source "${ROS_SETUP}"
        cd flyrat_play/ros2_ws
        colcon build --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release
    )
else
    echo "[compile_commands] warning: colcon or ${ROS_SETUP} not found," \
         "skipping flyrat_play/ros2_ws" >&2
fi

# 2. The ROS-free projects. Configuring is enough to emit the database.
echo "[compile_commands] configuring libflyrat and flyrat_pilot"
cmake -S libflyrat -B libflyrat/build -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake -S flyrat_pilot -B flyrat_pilot/build -DCMAKE_BUILD_TYPE=Release >/dev/null

# 3. Merge. Later entries win, so the standalone libflyrat database goes last:
#    its sources then get the ROS-free flags instead of the ament ones.
python3 - "${REPO_ROOT}" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
databases = [
    root / "flyrat_play/ros2_ws/build/flyrat_sensors_interfaces/compile_commands.json",
    root / "flyrat_play/ros2_ws/build/flyrat_sensors/compile_commands.json",
    root / "flyrat_play/ros2_ws/build/flyrat_msp/compile_commands.json",
    root / "flyrat_play/ros2_ws/build/flyrat_control/compile_commands.json",
    root / "flyrat_play/ros2_ws/build/flyrat_slam/compile_commands.json",
    root / "flyrat_pilot/build/compile_commands.json",
    root / "libflyrat/build/compile_commands.json",
]

merged = {}
for database in databases:
    if not database.is_file():
        print(f"[compile_commands] warning: {database} missing", file=sys.stderr)
        continue
    for entry in json.loads(database.read_text()):
        merged[entry["file"]] = entry

output = root / "compile_commands.json"
output.write_text(json.dumps(list(merged.values()), indent=2) + "\n")
orb_entries = sum(
    1 for entry in merged.values() if "/third_party/orb_slam3/" in entry["file"])
print(f"[compile_commands] wrote {output} ({len(merged)} entries, "
      f"{orb_entries} ORB-SLAM3)")
if orb_entries == 0:
    print("[compile_commands] warning: no ORB-SLAM3 entries; build "
          "flyrat_slam with FLYRAT_BUILD_SLAM=ON to index it", file=sys.stderr)

# The standalone libflyrat build is configured with FLYRAT_BUILD_SLAM=OFF, so
# its database lacks stereo_slam.cpp and every ORB-SLAM3 source. Tools that
# point directly at libflyrat/build would then report missing ORB headers.
# Augment that database with the SLAM entries from flyrat_slam.
standalone = root / "libflyrat/build/compile_commands.json"
slam_db = root / "flyrat_play/ros2_ws/build/flyrat_slam/compile_commands.json"
if standalone.is_file() and slam_db.is_file():
    entries = {entry["file"]: entry
               for entry in json.loads(standalone.read_text())}
    for entry in json.loads(slam_db.read_text()):
        entries.setdefault(entry["file"], entry)
    standalone.write_text(
        json.dumps(list(entries.values()), indent=2) + "\n")
    standalone_orb = sum(
        1 for entry in entries.values()
        if "/third_party/orb_slam3/" in entry["file"])
    print(f"[compile_commands] updated {standalone} "
          f"({len(entries)} entries, {standalone_orb} ORB-SLAM3)")
PY
