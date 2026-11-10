#!/bin/bash
#SBATCH --job-name=tio_eval
#SBATCH --partition=g6
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --output=logs/%x_%j.out
#SBATCH --error=logs/%x_%j.err
#SBATCH --time=02:00:00

# Evaluate one trained run three ways: normal test split, degraded ToF, and the
# v3.1 test split with a zero camera (camera-enabled checkpoints only).
#
# Usage:  sbatch eval_run.sbatch <run-name>
set -euo pipefail

ROOT="${FLYRAT_P1_ROOT:-$HOME/flyrat_tio/p1_camera}"
PYTHON="${FLYRAT_PYTHON:-$HOME/micromamba/envs/fdrone/bin/python}"
RUN="${1:?run name required}"

cd "$ROOT/research/tio_former"
mkdir -p "$ROOT/logs" "$ROOT/runs_cam_full"

export OMP_NUM_THREADS=6
export MKL_NUM_THREADS=6

"$PYTHON" scripts/evaluate.py \
    --checkpoint "$ROOT/checkpoints_cam_full/$RUN/best.pt" \
    --processed "$ROOT/data_cam_full/processed" \
    --split test \
    --out "$ROOT/runs_cam_full/${RUN}_test"

"$PYTHON" scripts/evaluate.py \
    --checkpoint "$ROOT/checkpoints_cam_full/$RUN/best.pt" \
    --processed "$ROOT/data_cam_full/processed" \
    --split test \
    --drop-views front left right \
    --status-dropout 0.3 \
    --out "$ROOT/runs_cam_full/${RUN}_tof_degraded"

"$PYTHON" scripts/evaluate.py \
    --checkpoint "$ROOT/checkpoints_cam_full/$RUN/best.pt" \
    --processed "$ROOT/data_v31_processed" \
    --split test \
    --out "$ROOT/runs_cam_full/${RUN}_v31_test"
