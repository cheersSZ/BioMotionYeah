"""Tiny `.mot` / `.sto` reader and writer.

Doesn't depend on OpenSim — this is what the comparator uses on machines that
only need to look at results, plus what the iOS-export loader uses to read the
files the phone produced.

The format is the OpenSim Storage v1 text format:

    <description>
    version=1
    nRows=...
    nColumns=...
    inDegrees=yes|no
    endheader
    time<TAB>col1<TAB>col2<TAB>...
    <values...>
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


@dataclass(frozen=True)
class StorageFile:
    path: Path
    description: str
    in_degrees: bool
    columns: list[str]            # excludes the time column
    data: pd.DataFrame            # index = time (s), columns = `columns`
    extra_header: dict[str, str]  # any non-standard header fields preserved

    @property
    def fps(self) -> float:
        if len(self.data.index) < 2:
            return 0.0
        dt = float(np.median(np.diff(self.data.index.values)))
        return 1.0 / dt if dt > 0 else 0.0


def read_storage(path: Path | str) -> StorageFile:
    """Parse a `.mot` or `.sto` file. Returns a `StorageFile`."""
    path = Path(path)
    with path.open("r") as fh:
        lines = fh.read().splitlines()

    description = lines[0] if lines else ""
    header: dict[str, str] = {}

    end_idx = None
    for i, line in enumerate(lines[1:], start=1):
        stripped = line.strip()
        if stripped.lower() == "endheader":
            end_idx = i
            break
        if "=" in stripped:
            k, v = stripped.split("=", 1)
            header[k.strip()] = v.strip()

    if end_idx is None:
        raise ValueError(f"{path}: missing 'endheader' line")

    column_line = lines[end_idx + 1]
    cols_all = [c for c in column_line.split("\t") if c != ""]
    if not cols_all or cols_all[0].lower() != "time":
        raise ValueError(
            f"{path}: expected first column to be 'time', got {cols_all[:1]!r}"
        )
    columns = cols_all[1:]

    data_rows = []
    for line in lines[end_idx + 2:]:
        if not line.strip():
            continue
        parts = line.split("\t")
        if len(parts) != len(cols_all):
            # Some writers pad with spaces instead of tabs; try whitespace split.
            parts = line.split()
        if len(parts) != len(cols_all):
            raise ValueError(
                f"{path}: row width {len(parts)} != header width {len(cols_all)}"
            )
        data_rows.append([float(p) for p in parts])

    arr = np.asarray(data_rows, dtype=float)
    times = arr[:, 0]
    values = arr[:, 1:]
    df = pd.DataFrame(values, index=pd.Index(times, name="time"), columns=columns)

    in_degrees = header.get("inDegrees", "no").lower() == "yes"
    extra = {k: v for k, v in header.items() if k not in {"version", "nRows", "nColumns", "inDegrees"}}

    return StorageFile(
        path=path,
        description=description,
        in_degrees=in_degrees,
        columns=columns,
        data=df,
        extra_header=extra,
    )


def write_storage(
    path: Path | str,
    description: str,
    df: pd.DataFrame,
    *,
    in_degrees: bool,
) -> Path:
    """Write a DataFrame back to OpenSim Storage v1 format."""
    path = Path(path)
    n_rows, n_cols = df.shape
    lines: list[str] = []
    lines.append(description)
    lines.append("version=1")
    lines.append(f"nRows={n_rows}")
    lines.append(f"nColumns={n_cols + 1}")
    lines.append(f"inDegrees={'yes' if in_degrees else 'no'}")
    lines.append("endheader")
    lines.append("time\t" + "\t".join(df.columns.astype(str)))
    for t, row in zip(df.index.values, df.values):
        formatted = "\t".join(f"{v:.6f}" for v in row)
        lines.append(f"{float(t):.6f}\t{formatted}")
    path.write_text("\n".join(lines) + "\n")
    return path


def common_time_grid(
    series: Iterable[pd.DataFrame],
    *,
    sample_rate_hz: float | None = None,
) -> np.ndarray:
    """Build a uniform time grid that covers the overlap of every series."""
    starts, stops, dts = [], [], []
    for df in series:
        if len(df.index) < 2:
            continue
        starts.append(float(df.index.values[0]))
        stops.append(float(df.index.values[-1]))
        dts.append(float(np.median(np.diff(df.index.values))))
    if not starts:
        return np.empty(0)
    t0 = max(starts)
    t1 = min(stops)
    if t1 <= t0:
        return np.empty(0)
    if sample_rate_hz is None:
        dt = min(dts)
    else:
        dt = 1.0 / sample_rate_hz
    n = int(np.floor((t1 - t0) / dt)) + 1
    return t0 + dt * np.arange(n)


def resample(df: pd.DataFrame, new_index: np.ndarray) -> pd.DataFrame:
    """Linear interpolation onto a new monotonically-increasing time grid.
    Out-of-range samples are dropped (NOT extrapolated)."""
    if len(new_index) == 0 or df.empty:
        return pd.DataFrame(index=pd.Index(new_index, name="time"), columns=df.columns)
    old_index = df.index.values.astype(float)
    valid = (new_index >= old_index[0]) & (new_index <= old_index[-1])
    masked = new_index[valid]
    cols = {}
    for col in df.columns:
        cols[col] = np.interp(masked, old_index, df[col].values)
    out = pd.DataFrame(cols, index=pd.Index(masked, name="time"))
    return out
