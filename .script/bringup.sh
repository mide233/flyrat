#!/usr/bin/env bash
#
# Unified entry point for the operational scripts. The individual scripts stay
# available for direct use, but new documentation and automation should prefer
# this command so there is one place to discover the supported modes.
#
# Usage:
#   .script/bringup.sh sensors [bringup_real.sh arguments...]
#   .script/bringup.sh slam    [bringup_pilot_slam.sh arguments...]
#   .script/bringup.sh raw     [bringup_raw_record.sh arguments...]
#   .script/bringup.sh build   [build_all.sh arguments...]
#   .script/bringup.sh clean   [clean_build.sh arguments...]
#   .script/bringup.sh foxglove [foxglove.sh arguments...]
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    sed -n '6,17p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

if [ "$#" -eq 0 ]; then
    usage
    exit 1
fi

mode="$1"
shift

case "${mode}" in
    sensors|real)
        exec "${SCRIPT_DIR}/bringup_real.sh" "$@"
        ;;
    slam|pilot)
        exec "${SCRIPT_DIR}/bringup_pilot_slam.sh" "$@"
        ;;
    raw|calib)
        exec "${SCRIPT_DIR}/bringup_raw_record.sh" "$@"
        ;;
    build)
        exec "${SCRIPT_DIR}/build_all.sh" "$@"
        ;;
    clean)
        exec "${SCRIPT_DIR}/clean_build.sh" "$@"
        ;;
    foxglove|fox)
        exec "${SCRIPT_DIR}/foxglove.sh" "$@"
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        echo "error: unknown mode '${mode}'" >&2
        usage >&2
        exit 1
        ;;
esac
