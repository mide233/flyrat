#!/usr/bin/env bash
#
# M2 asset download stub.
#
# M0/M1 use procedural primitive scenes only.  The next milestone adds real
# indoor meshes from Gazebo Fuel and Replica.  Assets are intentionally not
# committed; this script records the expected sources and licenses so a
# reproduction can fetch them explicitly.
#
# Usage:
#   bash scripts/download_assets.sh [--out DIR]
#
set -euo pipefail

OUT_DIR="assets"
while [ "$#" -gt 0 ]; do
    case "$1" in
        --out) OUT_DIR="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

cat <<EOF
Asset download is not implemented in M0/M1.

Planned sources (M2):
  Gazebo Fuel  https://fuel.gazebosim.org/
    - Indoor office / warehouse models
    - License: per-model, usually CC-BY 4.0; record each model's metadata.
  Replica      https://github.com/facebookresearch/Replica-Dataset
    - Apartment / office / room meshes
    - License: Replica license, research use only; do not redistribute raw assets.

Target directory: ${OUT_DIR}
Suggested layout:
  ${OUT_DIR}/fuel/<model>/
  ${OUT_DIR}/replica/<scene>/
  ${OUT_DIR}/LICENSES.md

The generator and config files reference assets by relative path; missing
assets fall back to the procedural scenes used by M1.
EOF
