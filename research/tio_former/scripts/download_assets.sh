#!/usr/bin/env bash
#
# Download the small set of Gazebo Fuel mesh props used by the M2 scenes.
#
# Assets are not committed.  Models are cached under assets/fuel and copied to
# assets/models/<Model> so that `model://<Model>` resolves from
# GZ_SIM_RESOURCE_PATH=assets/models.  Each model keeps its upstream license;
# a summary is written to assets/LICENSES.md.
#
# Replica meshes are optional and not downloaded automatically: the Replica
# license is research-only and the raw dataset is several GB.  See
# docs/agent/research/tio_former.md for the manual integration recipe.
#
# Usage:
#   bash scripts/download_assets.sh [--out DIR] [--force]
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUT_DIR="${ROOT}/assets"
FORCE=0
while [ "$#" -gt 0 ]; do
    case "$1" in
        --out) OUT_DIR="$2"; shift 2 ;;
        --force) FORCE=1; shift ;;
        -h|--help)
            sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

MODELS=(
    Desk
    OfficeChairGrey
    Cabinet
    Fridge
    Table
    SquareShelf
    MonitorAndKeyboard
    Sofa
)

mkdir -p "${OUT_DIR}/fuel" "${OUT_DIR}/models"
export GZ_FUEL_CACHE_PATH="${OUT_DIR}/fuel"

for model in "${MODELS[@]}"; do
    dest="${OUT_DIR}/models/${model}"
    if [ -d "${dest}" ] && [ "${FORCE}" -eq 0 ]; then
        echo "[assets] ${model}: already present"
        continue
    fi
    echo "[assets] downloading ${model}"
    env -u GZ_CONFIG_PATH PATH="/usr/bin:/bin:${PATH}" \
        gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/${model}" \
        >/dev/null
    # Fuel cache layout: <cache>/<server>/<owner>/models/<lowercase>/<version>/
    source_dir="$(find "${OUT_DIR}/fuel" -type f -path "*/models/${model,,}/*/model.config" \
        | sort -V | tail -n 1 | xargs -r dirname)"
    if [ -z "${source_dir}" ]; then
        echo "[assets] error: could not locate downloaded model ${model}" >&2
        exit 1
    fi
    rm -rf "${dest}"
    cp -a "${source_dir}" "${dest}"
    echo "[assets] ${model}: ${dest}"
done

python3 - "${OUT_DIR}" <<'PY'
import json
import sys
import urllib.request
from pathlib import Path

out = Path(sys.argv[1])
models = [
    "Desk", "OfficeChairGrey", "Cabinet", "Fridge",
    "Table", "SquareShelf", "MonitorAndKeyboard", "Sofa",
]
lines = [
    "# Third-party mesh assets",
    "",
    "Downloaded from Gazebo Fuel (OpenRobotics) by `scripts/download_assets.sh`.",
    "Assets are not committed to this repository.",
    "",
    "| Model | License | Source |",
    "|---|---|---|",
]
for model in models:
    try:
        with urllib.request.urlopen(
            f"https://fuel.gazebosim.org/1.0/OpenRobotics/models/{model}", timeout=20
        ) as response:
            meta = json.load(response)
    except Exception as exc:  # network failure should not invalidate local assets
        lines.append(f"| {model} | unknown ({exc}) | https://fuel.gazebosim.org/1.0/OpenRobotics/models/{model} |")
        continue
    lines.append(
        f"| {model} | {meta.get('license_name', 'unknown')} | "
        f"https://fuel.gazebosim.org/1.0/OpenRobotics/models/{model} |"
    )
lines += [
    "",
    "Replica (https://github.com/facebookresearch/Replica-Dataset) is optional.",
    "Its license is research-only; download it manually and do not redistribute",
    "the raw meshes.  The M2 dataset uses the Fuel props above plus procedural",
    "walls and large furniture.",
]
(out / "LICENSES.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"[assets] wrote {out / 'LICENSES.md'}")
PY

echo "[assets] set GZ_SIM_RESOURCE_PATH to include ${OUT_DIR}/models"
