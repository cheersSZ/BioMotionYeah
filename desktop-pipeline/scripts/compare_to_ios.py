#!/usr/bin/env python3
"""Convenience wrapper: `python scripts/compare_to_ios.py --desktop ... --ios ...`

Equivalent to `python -m biomotion_desktop.cli compare ...`.
"""
from __future__ import annotations

import sys

from biomotion_desktop.cli import main


if __name__ == "__main__":
    sys.exit(main(["compare", *sys.argv[1:]]))
