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

# 1. ROS 2 packages. colcon forwards the flag to every package's configure step.
if command -v colcon >/dev/null 2>&1 && [ -f "${ROS_SETUP}" ]; then
    echo "[compile_commands] configuring/building flyrat_play/ros2_ws"
    (
        set +u
        # shellcheck disable=SC1090
        source "${ROS_SETUP}"
        cd flyrat_play/ros2_ws
        colcon build --symlink-install \
            --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
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
    root / "flyrat_play/ros2_ws/build/flyrat_msp/compile_commands.json",
    root / "flyrat_play/ros2_ws/build/flyrat_control/compile_commands.json",
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
print(f"[compile_commands] wrote {output} ({len(merged)} entries)")
PY
