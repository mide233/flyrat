#!/usr/bin/env bash
#
# Generate a single compile_commands.json at the repository root for clangd.
#
# CMake emits one database per build directory (autopilot/build,
# ros2_ws/build/<pkg>), but clangd only searches upwards from the file being
# edited, so sources under autopilot/ or ros2_ws/src/ would never find them.
# This script (re)configures both builds and merges the per-project databases
# into <repo>/compile_commands.json, which clangd does find from anywhere in
# the tree (see also the repo-root .clangd).
#
# Usage:
#   scripts/export_compile_commands.sh
#
# Re-run it after adding/removing source files or changing compile flags.
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

# 1. ROS 2 packages. colcon forwards the flag to every package's configure step.
if command -v colcon >/dev/null 2>&1 && [ -f "${ROS_SETUP}" ]; then
    echo "[compile_commands] configuring/building ros2_ws"
    (
        set +u
        # shellcheck disable=SC1090
        source "${ROS_SETUP}"
        cd ros2_ws
        colcon build --symlink-install \
            --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    )
else
    echo "[compile_commands] warning: colcon or ${ROS_SETUP} not found," \
         "skipping ros2_ws" >&2
fi

# 2. The ROS-free autopilot library. Configuring is enough to emit the database.
echo "[compile_commands] configuring autopilot"
cmake -S autopilot -B autopilot/build -DCMAKE_BUILD_TYPE=Release >/dev/null

# 3. Merge. Later entries win, so the standalone autopilot database goes last:
#    its sources then get the ROS-free flags instead of the ament ones.
python3 - "${REPO_ROOT}" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
databases = [
    root / "ros2_ws/build/flyrat_msp/compile_commands.json",
    root / "ros2_ws/build/flyrat_control/compile_commands.json",
    root / "autopilot/build/compile_commands.json",
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
print(f"[compile_commands] wrote {output} ({len(merged)} entries)")
PY
