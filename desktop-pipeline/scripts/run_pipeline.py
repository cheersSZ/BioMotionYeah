#!/usr/bin/env python3
"""Convenience wrapper: `python scripts/run_pipeline.py --config ... --output ...`

Equivalent to `python -m biomotion_desktop.cli run ...`.
"""
from __future__ import annotations

import sys

from biomotion_desktop.cli import main


if __name__ == "__main__":
    sys.exit(main(["run", *sys.argv[1:]]))
