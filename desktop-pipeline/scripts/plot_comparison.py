#!/usr/bin/env python3
"""Convenience wrapper: `python scripts/plot_comparison.py --desktop ... --ios ...`

Equivalent to `python -m biomotion_desktop.cli plot ...`.
"""
from __future__ import annotations

import sys

from biomotion_desktop.cli import main


if __name__ == "__main__":
    sys.exit(main(["plot", *sys.argv[1:]]))
