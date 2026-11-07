#!/usr/bin/env python3
"""Print a summary of a processed TIO-Former sequence."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.data.dataset import inspect_sequence  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence")
    args = parser.parse_args()
    print(inspect_sequence(args.sequence))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
