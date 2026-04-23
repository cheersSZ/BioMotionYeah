"""Compare a desktop-pipeline output folder against an iOS export folder.

Three layered tiers — all fast, all on by default:

  Tier 1  (every file, every column)
    rmse, max_abs, p95_abs, r2, bias

  Tier 2  (kinematics.mot only — joint-level diagnostics)
    rom_error, phase_lag_ms

  Tier 3  (activations.sto only — muscle timing)
    onset_diff_ms, offset_diff_ms, peak_time_diff_ms

Time alignment: linear interpolation of the iOS series onto the desktop time
grid (desktop is treated as authoritative because it always uses the source
TRC framerate). Out-of-range samples are dropped, never extrapolated. Only
columns present in BOTH sides participate in metrics; the rest go into
`unmatched_columns` so naming drift between the Nimble model and OpenSim
model is visible at a glance rather than silently ignored.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
import pandas as pd

from .ios_export import ExportBundle, load_export
from .storage_io import StorageFile, common_time_grid, resample


ACTIVATION_THRESHOLD = 0.05  # threshold (0–1) used for muscle on/off detection


# ---------------------------------------------------------------------------
# Result dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ChannelMetrics:
    rmse: float
    max_abs: float
    p95_abs: float
    r2: float
    bias: float
    n_samples: int


@dataclass(frozen=True)
class FileComparison:
    file_key: str                                  # e.g. "kinematics.mot"
    n_frames: int
    time_window_s: tuple[float, float]
    overall: dict
    channels: dict[str, ChannelMetrics]
    tier2: Optional[dict] = None                   # kinematics-only
    tier3: Optional[dict] = None                   # activations-only
    unmatched: dict = field(default_factory=dict)  # {desktop_only:[], ios_only:[]}


@dataclass(frozen=True)
class FolderComparison:
    stem: str
    desktop_dir: Path
    ios_dir: Path
    files: dict[str, FileComparison]
    files_missing: dict[str, list[str]]            # which side was missing what

    def to_json(self) -> dict:
        return {
            "stem": self.stem,
            "desktop_dir": str(self.desktop_dir),
            "ios_dir": str(self.ios_dir),
            "files_missing": self.files_missing,
            "files": {
                key: _file_comparison_to_json(fc) for key, fc in self.files.items()
            },
        }


def _file_comparison_to_json(fc: FileComparison) -> dict:
    return {
        "n_frames": fc.n_frames,
        "time_window_s": list(fc.time_window_s),
        "overall": fc.overall,
        "channels": {name: asdict(m) for name, m in fc.channels.items()},
        "tier2": fc.tier2,
        "tier3": fc.tier3,
        "unmatched": fc.unmatched,
    }


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


_FILE_KEYS: dict[str, str] = {
    "kinematics.mot": "kinematics",
    "inverse_dynamics.sto": "inverse_dynamics",
    "activations.sto": "activations",
    "muscle_forces.sto": "muscle_forces",
}


# Per-file column normalisers. The iOS exporter writes joint moments without
# any suffix (e.g. ``ankle_angle_l``) but OpenSim's InverseDynamicsTool tags
# rotational DOFs with ``_moment`` and translational ones with ``_force``
# (e.g. ``ankle_angle_l_moment``, ``pelvis_tx_force``). Strip those on the
# desktop side so the two name spaces line up. Same trial, same DOF, same
# physical quantity — the suffix is a labelling artefact, not data.
_ID_DESKTOP_SUFFIXES = ("_moment", "_force")


def _normalize_columns(file_key: str, side: str, columns: list[str]) -> list[str]:
    """Return the canonical channel names for a (file_key, side) pair.

    Order is preserved so callers can still index a DataFrame with the
    original column list and rename via `dict(zip(original, normalized))`.
    """
    if file_key == "inverse_dynamics.sto" and side == "desktop":
        return [_strip_suffix(c, _ID_DESKTOP_SUFFIXES) for c in columns]
    return list(columns)


def _strip_suffix(name: str, suffixes: tuple[str, ...]) -> str:
    for s in suffixes:
        if name.endswith(s):
            return name[: -len(s)]
    return name


def compare_folders(
    desktop_dir: Path | str,
    ios_dir: Path | str,
    stem: str,
    *,
    ios_stem: Optional[str] = None,
    tiers: Iterable[int] = (1, 2, 3),
) -> FolderComparison:
    """Run the comparison and return a structured result. The CLI's
    `compare` subcommand wraps this and dumps `to_json()` to disk.

    `ios_stem` defaults to `stem`. The iOS app names its exports after a
    per-run UUID (e.g. ``46A017D9-..._kinematics.mot``) while the desktop
    pipeline uses the human-readable trial name; pass `ios_stem` when the two
    differ so the loader finds the right files on each side.
    """
    desktop_dir = Path(desktop_dir).expanduser().resolve()
    ios_dir = Path(ios_dir).expanduser().resolve()
    tiers_set = {int(t) for t in tiers}

    desk = load_export(desktop_dir, stem)
    ios = load_export(ios_dir, ios_stem or stem)

    files: dict[str, FileComparison] = {}
    missing: dict[str, list[str]] = {"desktop_only": [], "ios_only": []}

    for file_key, attr in _FILE_KEYS.items():
        d_storage: Optional[StorageFile] = getattr(desk, attr)
        i_storage: Optional[StorageFile] = getattr(ios, attr)

        if d_storage is None and i_storage is None:
            continue
        if d_storage is None:
            missing["desktop_only"].append(file_key)
            continue
        if i_storage is None:
            missing["ios_only"].append(file_key)
            continue

        files[file_key] = _compare_one_file(
            file_key=file_key,
            desktop=d_storage,
            ios=i_storage,
            tiers=tiers_set,
        )

    return FolderComparison(
        stem=stem,
        desktop_dir=desktop_dir,
        ios_dir=ios_dir,
        files=files,
        files_missing=missing,
    )


def write_report(report: FolderComparison, out_path: Path | str) -> Path:
    """Dump the comparison to a pretty-printed JSON file."""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report.to_json(), indent=2, sort_keys=True) + "\n")
    return out_path


# ---------------------------------------------------------------------------
# Per-file comparison
# ---------------------------------------------------------------------------


def _compare_one_file(
    *,
    file_key: str,
    desktop: StorageFile,
    ios: StorageFile,
    tiers: set[int],
) -> FileComparison:
    desk_df = desktop.data
    grid = common_time_grid([desk_df, ios.data])
    if len(grid) < 2:
        return FileComparison(
            file_key=file_key,
            n_frames=0,
            time_window_s=(0.0, 0.0),
            overall={"error": "no overlapping time window"},
            channels={},
            unmatched={
                "desktop_only": list(desktop.columns),
                "ios_only": list(ios.columns),
            },
        )

    desk_aligned = resample(desk_df, grid)
    ios_aligned = resample(ios.data, grid)

    desk_rename = dict(zip(
        list(desk_aligned.columns),
        _normalize_columns(file_key, "desktop", list(desk_aligned.columns)),
    ))
    ios_rename = dict(zip(
        list(ios_aligned.columns),
        _normalize_columns(file_key, "ios", list(ios_aligned.columns)),
    ))
    if any(k != v for k, v in desk_rename.items()):
        desk_aligned = desk_aligned.rename(columns=desk_rename)
    if any(k != v for k, v in ios_rename.items()):
        ios_aligned = ios_aligned.rename(columns=ios_rename)

    desk_cols = set(desk_aligned.columns)
    ios_cols = set(ios_aligned.columns)
    shared = sorted(desk_cols & ios_cols)
    unmatched = {
        "desktop_only": sorted(desk_cols - ios_cols),
        "ios_only": sorted(ios_cols - desk_cols),
    }

    channels: dict[str, ChannelMetrics] = {}
    if 1 in tiers:
        for col in shared:
            channels[col] = _channel_metrics(
                desk_aligned[col].to_numpy(),
                ios_aligned[col].to_numpy(),
            )

    overall = _aggregate_overall(channels) if 1 in tiers else {}

    tier2 = None
    if 2 in tiers and file_key == "kinematics.mot":
        tier2 = _tier2_kinematics(desk_aligned, ios_aligned, shared, fps=desktop.fps)

    tier3 = None
    if 3 in tiers and file_key == "activations.sto":
        tier3 = _tier3_activations(desk_aligned, ios_aligned, shared, fps=desktop.fps)

    return FileComparison(
        file_key=file_key,
        n_frames=len(grid),
        time_window_s=(float(grid[0]), float(grid[-1])),
        overall=overall,
        channels=channels,
        tier2=tier2,
        tier3=tier3,
        unmatched=unmatched,
    )


# ---------------------------------------------------------------------------
# Tier 1 — per channel
# ---------------------------------------------------------------------------


def _channel_metrics(desk: np.ndarray, ios: np.ndarray) -> ChannelMetrics:
    n = min(len(desk), len(ios))
    if n == 0:
        return ChannelMetrics(
            rmse=float("nan"),
            max_abs=float("nan"),
            p95_abs=float("nan"),
            r2=float("nan"),
            bias=float("nan"),
            n_samples=0,
        )
    desk = desk[:n]
    ios = ios[:n]
    diff = ios - desk
    abs_diff = np.abs(diff)
    rmse = float(np.sqrt(np.mean(diff * diff)))
    return ChannelMetrics(
        rmse=rmse,
        max_abs=float(np.max(abs_diff)),
        p95_abs=float(np.percentile(abs_diff, 95)),
        r2=_pearson_r2(desk, ios),
        bias=float(np.mean(diff)),
        n_samples=int(n),
    )


def _pearson_r2(a: np.ndarray, b: np.ndarray) -> float:
    """Pearson R squared. Defined as 0 if either side is constant."""
    if a.size < 2:
        return 0.0
    sa = float(np.std(a))
    sb = float(np.std(b))
    if sa == 0.0 or sb == 0.0:
        return 0.0
    r = float(np.corrcoef(a, b)[0, 1])
    if math.isnan(r):
        return 0.0
    return r * r


def _aggregate_overall(channels: dict[str, ChannelMetrics]) -> dict:
    if not channels:
        return {"channel_count": 0}
    rmses = np.array([c.rmse for c in channels.values()])
    max_abs = np.array([c.max_abs for c in channels.values()])
    r2s = np.array([c.r2 for c in channels.values()])
    biases = np.array([c.bias for c in channels.values()])
    return {
        "channel_count": len(channels),
        "rmse_mean": float(np.nanmean(rmses)),
        "rmse_max":  float(np.nanmax(rmses)),
        "max_abs_max": float(np.nanmax(max_abs)),
        "r2_mean":   float(np.nanmean(r2s)),
        "r2_min":    float(np.nanmin(r2s)),
        "bias_mean": float(np.nanmean(biases)),
        "abs_bias_max": float(np.nanmax(np.abs(biases))),
    }


# ---------------------------------------------------------------------------
# Tier 2 — kinematics-only
# ---------------------------------------------------------------------------


def _tier2_kinematics(
    desk: pd.DataFrame, ios: pd.DataFrame, shared: list[str], *, fps: float
) -> dict:
    rom_error: dict[str, float] = {}
    phase_lag_ms: dict[str, float] = {}
    dt_ms = 1000.0 / fps if fps > 0 else 0.0

    for col in shared:
        d = desk[col].to_numpy()
        i = ios[col].to_numpy()
        if len(d) < 4 or len(i) < 4:
            continue
        rom_d = float(np.nanmax(d) - np.nanmin(d))
        rom_i = float(np.nanmax(i) - np.nanmin(i))
        rom_error[col] = rom_i - rom_d  # iOS minus desktop, signed

        if dt_ms > 0:
            lag_samples = _xcorr_peak_offset(d, i)
            phase_lag_ms[col] = lag_samples * dt_ms

    return {"rom_error": rom_error, "phase_lag_ms": phase_lag_ms}


def _xcorr_peak_offset(a: np.ndarray, b: np.ndarray) -> int:
    """Sample offset (positive = b lags a) at the peak of normalized
    cross-correlation. Uses mean-centered signals so a constant bias
    doesn't dominate."""
    a = a - np.mean(a)
    b = b - np.mean(b)
    if np.std(a) == 0 or np.std(b) == 0:
        return 0
    n = len(a)
    full = np.correlate(a, b, mode="full")
    lags = np.arange(-n + 1, n)
    peak = int(lags[int(np.argmax(full))])
    # `b lags a` means we need to shift b backwards (negative lag) to align,
    # which numpy's `correlate(a, b)` returns as positive lag at the peak.
    return peak


# ---------------------------------------------------------------------------
# Tier 3 — activations-only
# ---------------------------------------------------------------------------


def _tier3_activations(
    desk: pd.DataFrame, ios: pd.DataFrame, shared: list[str], *, fps: float
) -> dict:
    onset_ms: dict[str, float] = {}
    offset_ms: dict[str, float] = {}
    peak_ms: dict[str, float] = {}
    dt_ms = 1000.0 / fps if fps > 0 else 0.0
    if dt_ms == 0:
        return {"onset_diff_ms": {}, "offset_diff_ms": {}, "peak_time_diff_ms": {}}

    for col in shared:
        d = desk[col].to_numpy()
        i = ios[col].to_numpy()
        d_on, d_off = _on_off_indices(d, ACTIVATION_THRESHOLD)
        i_on, i_off = _on_off_indices(i, ACTIVATION_THRESHOLD)
        if d_on is not None and i_on is not None:
            onset_ms[col] = (i_on - d_on) * dt_ms
        if d_off is not None and i_off is not None:
            offset_ms[col] = (i_off - d_off) * dt_ms
        if len(d) and len(i):
            peak_ms[col] = (int(np.argmax(i)) - int(np.argmax(d))) * dt_ms

    return {
        "onset_diff_ms": onset_ms,
        "offset_diff_ms": offset_ms,
        "peak_time_diff_ms": peak_ms,
        "threshold": ACTIVATION_THRESHOLD,
    }


def _on_off_indices(signal: np.ndarray, threshold: float):
    """Return (first_above_idx, last_above_idx) or (None, None) if the signal
    never crosses the threshold."""
    above = signal >= threshold
    if not np.any(above):
        return None, None
    idx = np.where(above)[0]
    return int(idx[0]), int(idx[-1])
