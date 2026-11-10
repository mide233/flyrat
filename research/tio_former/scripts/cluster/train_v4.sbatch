#!/bin/bash
#SBATCH --job-name=tio4
#SBATCH --partition=g6
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --output=logs/%x_%j.out
#SBATCH --error=logs/%x_%j.err
#SBATCH --time=06:00:00

# Train one TIO-Former run on the V4 randomized dataset.
#
# Usage:  sbatch train_v4.sbatch <seed> <run-name> <1|0>
#   <1|0> enables/disables the forward camera branch.
#
# Layout expected under $FLYRAT_V4_ROOT (default $HOME/flyrat_tio/v4):
#   research/tio_former/   code from this repository
#   data_v4/processed/     V4 dataset
#   checkpoints_v4/        output checkpoints
#   logs/                  Slurm logs
set -euo pipefail

ROOT="${FLYRAT_V4_ROOT:-$HOME/flyrat_tio/v4}"
PYTHON="${FLYRAT_PYTHON:-$HOME/micromamba/envs/fdrone/bin/python}"
SEED="${1:?seed required}"
RUN="${2:?run name required}"
CAMERA="${3:?1 for camera, 0 for baseline}"

# A plain string avoids `set -u` failing on an empty array.
if [ "$CAMERA" = "1" ]; then
    CAM_ARG="--camera"
else
    CAM_ARG=""
fi

cd "$ROOT/research/tio_former"
mkdir -p "$ROOT/logs" "$ROOT/checkpoints_v4"

export OMP_NUM_THREADS=6
export MKL_NUM_THREADS=6
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

"$PYTHON" scripts/train.py \
    --processed "$ROOT/data_v4/processed" \
    --out "$ROOT/checkpoints_v4" \
    --run-name "$RUN" \
    --seed "$SEED" \
    --epochs 60 \
    --batch-size 24 \
    --workers 6 \
    $CAM_ARG
