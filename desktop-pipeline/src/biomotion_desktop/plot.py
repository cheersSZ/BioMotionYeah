"""Optional matplotlib comparison plots: one PNG per shared channel.

Saved as `<out>/<file_key>_<sanitized_column>.png`. Skipped silently when
matplotlib is not installed so the comparator stays usable in headless
environments.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Optional

from .compare import _normalize_columns  # column-name normaliser shared with metrics
from .ios_export import load_export
from .storage_io import StorageFile, common_time_grid, resample


_FILE_ATTRS = {
    "kinematics.mot": ("kinematics", "joint angle (deg)"),
    "inverse_dynamics.sto": ("inverse_dynamics", "generalized force (Nm or N)"),
    "activations.sto": ("activations", "activation"),
    "muscle_forces.sto": ("muscle_forces", "force (N)"),
}


def plot_comparison(
    desktop_dir: Path | str,
    ios_dir: Path | str,
    stem: str,
    out_dir: Path | str,
    *,
    ios_stem: Optional[str] = None,
    max_per_file: Optional[int] = None,
) -> list[Path]:
    """Write per-channel comparison PNGs. Returns list of generated paths.

    `ios_stem` defaults to `stem`; pass it when the iOS export uses a
    different filename stem (e.g. a per-run UUID instead of the trial name).
    """
    try:
        import matplotlib  # type: ignore

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # type: ignore
    except ImportError:
        raise ImportError(
            "matplotlib is required for plotting. Install via "
            "`pip install matplotlib` or use the `plot` extra."
        )

    out_dir = Path(out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    desk = load_export(desktop_dir, stem)
    ios = load_export(ios_dir, ios_stem or stem)

    written: list[Path] = []
    for file_key, (attr, ylabel) in _FILE_ATTRS.items():
        d_storage: Optional[StorageFile] = getattr(desk, attr)
        i_storage: Optional[StorageFile] = getattr(ios, attr)
        if d_storage is None or i_storage is None:
            continue
        grid = common_time_grid([d_storage.data, i_storage.data])
        if len(grid) < 2:
            continue
        d_aligned = resample(d_storage.data, grid)
        i_aligned = resample(i_storage.data, grid)

        d_aligned = d_aligned.rename(columns=dict(zip(
            list(d_aligned.columns),
            _normalize_columns(file_key, "desktop", list(d_aligned.columns)),
        )))
        i_aligned = i_aligned.rename(columns=dict(zip(
            list(i_aligned.columns),
            _normalize_columns(file_key, "ios", list(i_aligned.columns)),
        )))

        shared = sorted(set(d_aligned.columns) & set(i_aligned.columns))
        if max_per_file is not None:
            shared = shared[:max_per_file]

        for col in shared:
            fig, ax = plt.subplots(figsize=(8, 3.2))
            ax.plot(grid, d_aligned[col].to_numpy(), label="desktop (OpenSim)", linewidth=1.4)
            ax.plot(grid, i_aligned[col].to_numpy(), label="iOS (Nimble/OSQP)", linewidth=1.0, alpha=0.85)
            ax.set_title(f"{file_key} — {col}")
            ax.set_xlabel("time (s)")
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.25)
            ax.legend(loc="best", fontsize=8)
            fig.tight_layout()

            safe_col = _sanitize(col)
            out_path = out_dir / f"{file_key.replace('.', '_')}__{safe_col}.png"
            fig.savefig(out_path, dpi=120)
            plt.close(fig)
            written.append(out_path)

    return written


def _sanitize(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name)
