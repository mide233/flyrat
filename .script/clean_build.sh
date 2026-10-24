#!/usr/bin/env bash
#
# Remove every build artifact produced by this repository.
#
# Usage:
#   .script/clean_build.sh [--dry-run] [--keep-sim]
#
# Options:
#   --dry-run    list what would be removed, delete nothing
#   --keep-sim   keep the SITL ELF and the Gazebo bridge plugin build
#   -h, --help   show this help
#
# Removed by default:
#   libflyrat/build/                                       CMake build tree
#   flyrat_pilot/build/                                    CMake build tree
#   flyrat_play/ros2_ws/{build,install,log}/               colcon outputs
#   flyrat_play/gazebo/third_party/betaflight/obj/         SITL ELF + objects
#   flyrat_play/gazebo/third_party/aeroloop_gazebo/plugins/build/
#                                                          Gazebo bridge plugin
#   compile_commands.json                                  merged clangd database
#   **/__pycache__/                                        Python bytecode
#   **/.cache/                                             clangd index caches
#
# Source files, submodules and the committed flight-controller config
# (flyrat_play/gazebo/config/eeprom.bin) are never touched.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

dry_run=0
keep_sim=0

usage() {
    sed -n '3,26p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

for arg in "$@"; do
    case "${arg}" in
        --dry-run) dry_run=1 ;;
        --keep-sim) keep_sim=1 ;;
        -h|--help) usage; exit 0 ;;
        *)
            echo "error: unknown option: ${arg}" >&2
            echo "       try: .script/clean_build.sh --help" >&2
            exit 1
            ;;
    esac
done

targets=(
    "libflyrat/build"
    "flyrat_pilot/build"
    "flyrat_play/ros2_ws/build"
    "flyrat_play/ros2_ws/install"
    "flyrat_play/ros2_ws/log"
    "compile_commands.json"
)

if [ "${keep_sim}" -eq 0 ]; then
    targets+=(
        "flyrat_play/gazebo/third_party/betaflight/obj"
        "flyrat_play/gazebo/third_party/aeroloop_gazebo/plugins/build"
    )
fi

# Generated caches are scattered across the tree, so discover them instead of
# hard-coding every location.
while IFS= read -r -d '' path; do
    targets+=("${path#./}")
done < <(find . -path ./.git -prune -o -type d \( -name __pycache__ -o -name .cache \) -print0)

removed=0
for target in "${targets[@]}"; do
    [ -e "${target}" ] || continue
    size="$(du -sh "${target}" 2>/dev/null | cut -f1 || true)"
    if [ "${dry_run}" -eq 1 ]; then
        printf '  would remove %-62s %s\n' "${target}" "${size}"
    else
        rm -rf -- "${target}"
        printf '  removed %-68s %s\n' "${target}" "${size}"
    fi
    removed=$((removed + 1))
done

if [ "${removed}" -eq 0 ]; then
    echo "[clean_build] nothing to remove"
elif [ "${dry_run}" -eq 1 ]; then
    echo "[clean_build] ${removed} target(s) would be removed (dry run)"
else
    echo "[clean_build] removed ${removed} target(s)"
fi
