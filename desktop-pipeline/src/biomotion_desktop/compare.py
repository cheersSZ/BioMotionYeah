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

# A channel is considered "locked" (a DOF the model holds at a fixed value, e.g.
# OpenSim's metatarsophalangeal joint) when its standard deviation falls below
# this absolute threshold in the channel's own unit. Locked channels are still
# reported per-channel — but they are excluded from the file-level aggregates
# (rmse_mean, r2_mean, …) because including them poisons the averages with
# meaningless ratios (R² = 0 by construction when std = 0).
LOCKED_STD_EPS = 1e-6

# Sign-flip diagnostic threshold (kinematics only). A kinematics channel is
# flagged as a suspected sign-convention mismatch when (a) flipping the
# desktop signal cuts RMSE by at least this factor AND (b) the original
# Pearson R² is already high — meaning shape agrees but amplitude doesn't.
SIGN_FLIP_RMSE_RATIO = 0.5
SIGN_FLIP_R2_FLOOR = 0.5

# Phase-compensation diagnostic (kinematics only). For each channel we search
# integer-sample lags up to ±PHASE_SEARCH_MAX_MS for the shift that minimises
# RMSE between the two signals. A channel is reported as benefiting from
# compensation when the optimal shift cuts RMSE by at least
# PHASE_COMPENSATION_RMSE_RATIO and is at least PHASE_COMPENSATION_MIN_LAG_MS
# away from zero (smaller shifts are within sample-quantisation noise).
#
# The motivating finding (run2_short, 2026-04-23): every kinematic DOF lagged
# the desktop side by +25–33 ms. Compensating that single uniform shift took
# `knee_angle_r` from RMSE 12.75°/R² 0.83 to RMSE 3.21°/R² 0.99. Root cause is
# almost certainly the iOS 1-Euro filter's group delay (causal) vs OpenSim's
# zero-phase Butterworth `filtfilt` (non-causal). See OPEN_ISSUES §4.2.
PHASE_SEARCH_MAX_MS = 100.0
PHASE_COMPENSATION_RMSE_RATIO = 0.7
PHASE_COMPENSATION_MIN_LAG_MS = 12.0


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
    status: str = "ok"     # "ok" | "locked_desktop" | "locked_ios" | "locked_both"


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
        tier2 = _tier2_kinematics(
            desk_aligned, ios_aligned, shared, fps=desktop.fps, channels=channels,
        )

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
            status="ok",
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
        status=_classify_lock(desk, ios),
    )


def _classify_lock(desk: np.ndarray, ios: np.ndarray) -> str:
    """Detect channels where one or both sides hold a fixed value (locked DOF).

    A locked channel cannot be meaningfully scored: R² is undefined when one
    side is constant, and RMSE collapses to |bias|. Tag them so the aggregator
    can skip them while still surfacing them per-channel.
    """
    desk_locked = float(np.std(desk)) < LOCKED_STD_EPS
    ios_locked = float(np.std(ios)) < LOCKED_STD_EPS
    if desk_locked and ios_locked:
        return "locked_both"
    if desk_locked:
        return "locked_desktop"
    if ios_locked:
        return "locked_ios"
    return "ok"


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
    locked = sorted(name for name, c in channels.items() if c.status != "ok")
    scored = {name: c for name, c in channels.items() if c.status == "ok"}
    if not scored:
        return {
            "channel_count": len(channels),
            "scored_count": 0,
            "locked_channels": locked,
        }
    rmses = np.array([c.rmse for c in scored.values()])
    max_abs = np.array([c.max_abs for c in scored.values()])
    r2s = np.array([c.r2 for c in scored.values()])
    biases = np.array([c.bias for c in scored.values()])
    return {
        "channel_count": len(channels),
        "scored_count": len(scored),
        "locked_channels": locked,
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
    desk: pd.DataFrame,
    ios: pd.DataFrame,
    shared: list[str],
    *,
    fps: float,
    channels: dict[str, ChannelMetrics],
) -> dict:
    rom_error: dict[str, float] = {}
    phase_lag_ms: dict[str, float] = {}
    sign_flip_suspects: dict[str, dict] = {}
    phase_compensated: dict[str, dict] = {}
    dt_ms = 1000.0 / fps if fps > 0 else 0.0
    max_lag_samples = int(PHASE_SEARCH_MAX_MS / dt_ms) if dt_ms > 0 else 0

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

        suspect = _sign_flip_suspect(d, i, channels.get(col))
        if suspect is not None:
            sign_flip_suspects[col] = suspect

        if max_lag_samples > 0:
            comp = _phase_compensated_metrics(
                d, i,
                original=channels.get(col),
                max_lag_samples=max_lag_samples,
                dt_ms=dt_ms,
            )
            if comp is not None:
                phase_compensated[col] = comp

    aggregate = _aggregate_phase_compensation(phase_compensated, dt_ms)

    return {
        "rom_error": rom_error,
        "phase_lag_ms": phase_lag_ms,
        "sign_flip_suspects": sign_flip_suspects,
        "phase_compensated": phase_compensated,
        "phase_compensation_summary": aggregate,
    }


def _phase_compensated_metrics(
    desk: np.ndarray,
    ios: np.ndarray,
    *,
    original: ChannelMetrics | None,
    max_lag_samples: int,
    dt_ms: float,
) -> Optional[dict]:
    """Search ±max_lag_samples for the shift that minimises RMSE between the
    two signals; return the best lag and what the metrics would look like
    after compensation. Returns None when compensation does not materially
    improve RMSE (the channel is either already aligned or genuinely
    different in shape, not just shifted).

    Positive lag = iOS lags desktop (we shift iOS *forward* to align).
    """
    if original is None or original.status != "ok":
        return None
    if original.rmse <= 0 or not math.isfinite(original.rmse):
        return None

    n = min(len(desk), len(ios))
    if n < 2 * max_lag_samples + 4:
        return None
    desk = desk[:n]
    ios = ios[:n]

    best_lag = 0
    best_rmse = original.rmse
    for lag in range(-max_lag_samples, max_lag_samples + 1):
        if lag == 0:
            continue
        if lag > 0:
            d_seg = desk[lag:]
            i_seg = ios[:-lag]
        else:
            d_seg = desk[:lag]
            i_seg = ios[-lag:]
        diff = i_seg - d_seg
        rmse = float(np.sqrt(np.mean(diff * diff)))
        if rmse < best_rmse:
            best_rmse = rmse
            best_lag = lag

    if best_lag == 0:
        return None
    best_lag_ms = best_lag * dt_ms
    if abs(best_lag_ms) < PHASE_COMPENSATION_MIN_LAG_MS:
        return None
    rmse_ratio = best_rmse / original.rmse
    if rmse_ratio > PHASE_COMPENSATION_RMSE_RATIO:
        return None

    if best_lag > 0:
        d_seg = desk[best_lag:]
        i_seg = ios[:-best_lag]
    else:
        d_seg = desk[:best_lag]
        i_seg = ios[-best_lag:]
    r2 = _pearson_r2(d_seg, i_seg)
    bias = float(np.mean(i_seg - d_seg))
    return {
        "best_lag_samples": int(best_lag),
        "best_lag_ms": best_lag_ms,
        "rmse_at_best_lag": best_rmse,
        "r2_at_best_lag": r2,
        "bias_at_best_lag": bias,
        "rmse_ratio": rmse_ratio,
        "original_rmse": original.rmse,
        "original_r2": original.r2,
    }


def _aggregate_phase_compensation(
    per_channel: dict[str, dict], dt_ms: float
) -> dict:
    """Roll the per-channel phase compensations up so a uniform whole-body
    time shift is obvious at a glance. A consistent sign across most channels
    is the fingerprint of a filter-topology mismatch (e.g., iOS 1-Euro causal
    delay vs desktop zero-phase filtfilt) rather than per-channel noise."""
    if not per_channel:
        return {"channel_count": 0}
    lags_ms = np.array([v["best_lag_ms"] for v in per_channel.values()])
    return {
        "channel_count": len(per_channel),
        "median_lag_ms": float(np.median(lags_ms)),
        "mean_lag_ms": float(np.mean(lags_ms)),
        "min_lag_ms": float(np.min(lags_ms)),
        "max_lag_ms": float(np.max(lags_ms)),
        "uniform_sign": bool(np.all(lags_ms > 0) or np.all(lags_ms < 0)),
        "dt_ms": dt_ms,
    }


def _sign_flip_suspect(
    desk: np.ndarray, ios: np.ndarray, original: ChannelMetrics | None
) -> Optional[dict]:
    """Flag a channel as a probable sign-convention mismatch.

    Pearson R² is invariant under sign flip of either input, but RMSE is not.
    A channel where shape agrees (R² high) yet RMSE is also high is the
    fingerprint of an inverted axis convention. We do *not* silently flip
    anything — the comparator only surfaces the hypothesis with the numbers
    that would result, so the operator can confirm before fixing the model
    or adding a CHANNEL_MAP transform.
    """
    if original is None or original.status != "ok":
        return None
    if not math.isfinite(original.r2) or original.r2 < SIGN_FLIP_R2_FLOOR:
        return None
    if original.rmse <= 0:
        return None

    n = min(len(desk), len(ios))
    if n < 4:
        return None
    diff = ios[:n] + desk[:n]  # equivalent to ios - (-desk)
    flipped_rmse = float(np.sqrt(np.mean(diff * diff)))
    if flipped_rmse >= SIGN_FLIP_RMSE_RATIO * original.rmse:
        return None
    flipped_bias = float(np.mean(diff))
    return {
        "original_rmse": original.rmse,
        "flipped_rmse": flipped_rmse,
        "rmse_ratio": flipped_rmse / original.rmse,
        "original_bias": original.bias,
        "flipped_bias": flipped_bias,
        "r2": original.r2,  # unchanged by sign flip
        "verdict": "desktop sign convention probably inverted relative to iOS",
    }


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
