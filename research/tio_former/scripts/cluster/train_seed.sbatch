#!/bin/bash
#SBATCH --job-name=tio_train
#SBATCH --partition=g6
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --output=logs/%x_%j.out
#SBATCH --error=logs/%x_%j.err
#SBATCH --time=04:00:00

# Train one TIO-Former run on the cluster.
#
# Usage:  sbatch train_seed.sbatch <seed> <run-name> <1|0>
#   <1|0> enables/disables the forward camera branch.
#
# Layout expected under $FLYRAT_P1_ROOT (default $HOME/flyrat_tio/p1_camera):
#   research/tio_former/      code from this repository
#   data_cam_full/processed/  camera-enabled dataset
#   checkpoints_cam_full/     output checkpoints
#   logs/                     Slurm logs
set -euo pipefail

ROOT="${FLYRAT_P1_ROOT:-$HOME/flyrat_tio/p1_camera}"
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
mkdir -p "$ROOT/logs" "$ROOT/checkpoints_cam_full"

export OMP_NUM_THREADS=6
export MKL_NUM_THREADS=6
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

"$PYTHON" scripts/train.py \
    --processed "$ROOT/data_cam_full/processed" \
    --out "$ROOT/checkpoints_cam_full" \
    --run-name "$RUN" \
    --seed "$SEED" \
    --epochs 60 \
    --batch-size 24 \
    --workers 6 \
    $CAM_ARG
