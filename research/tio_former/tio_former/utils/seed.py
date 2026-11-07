"""Deterministic seed helpers."""

from __future__ import annotations

import os
import random

import numpy as np


def set_seed(seed: int) -> np.random.Generator:
    """Seed Python and NumPy global RNGs and return a NumPy Generator."""
    os.environ["PYTHONHASHSEED"] = str(int(seed))
    random.seed(int(seed))
    np.random.seed(int(seed))
    return np.random.default_rng(int(seed))
