#!/bin/bash
#SBATCH --job-name=tio4eval
#SBATCH --partition=g6
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --output=logs/%x_%j.out
#SBATCH --error=logs/%x_%j.err
#SBATCH --time=02:00:00

# Evaluate one V4 checkpoint three ways: normal, degraded ToF, camera zeroed.
#
# Usage:  sbatch eval_v4.sbatch <run-name>
set -euo pipefail

ROOT="${FLYRAT_V4_ROOT:-$HOME/flyrat_tio/v4}"
PYTHON="${FLYRAT_PYTHON:-$HOME/micromamba/envs/fdrone/bin/python}"
RUN="${1:?run name required}"

cd "$ROOT/research/tio_former"
mkdir -p "$ROOT/logs" "$ROOT/runs_v4"
export OMP_NUM_THREADS=6
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

CKPT="$ROOT/checkpoints_v4/$RUN/best.pt"
EXTRA=""
if "$PYTHON" -c "import sys,torch; s=torch.load(sys.argv[1],map_location='cpu',weights_only=False); sys.exit(0 if s['model_config']['model'].get('camera',{}).get('enabled') else 1)" "$CKPT"; then
    EXTRA="--zero-camera"
fi

"$PYTHON" scripts/evaluate.py --checkpoint "$CKPT" \
    --processed "$ROOT/data_v4/processed" --split test \
    --out "$ROOT/runs_v4/${RUN}_test"

"$PYTHON" scripts/evaluate.py --checkpoint "$CKPT" \
    --processed "$ROOT/data_v4/processed" --split test \
    --drop-views front left right --status-dropout 0.3 \
    --out "$ROOT/runs_v4/${RUN}_tof_degraded"

if [ -n "$EXTRA" ]; then
    "$PYTHON" scripts/evaluate.py --checkpoint "$CKPT" \
        --processed "$ROOT/data_v4/processed" --split test \
        $EXTRA --out "$ROOT/runs_v4/${RUN}_zerocam"
fi
