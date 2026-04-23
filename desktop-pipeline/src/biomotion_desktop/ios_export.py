"""Loader for an iOS Offline-Export folder.

The iOS app's `OfflineAnalysisExporter.export(...)` writes:

    OfflineExport-<stem>-<uuid>/
    ├── <stem>_kinematics.mot
    ├── <stem>_inverse_dynamics.sto
    ├── <stem>_activations.sto      (optional — only if model has muscles)
    ├── <stem>_muscle_forces.sto    (optional)
    └── <stem>_summary.json

This module locates and parses those files. Any missing file is `None` so
the comparator can degrade gracefully (e.g. a model with no muscles still
produces a meaningful kinematics + ID diff).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .storage_io import StorageFile, read_storage


@dataclass(frozen=True)
class ExportBundle:
    folder: Path
    stem: str
    kinematics: Optional[StorageFile]
    inverse_dynamics: Optional[StorageFile]
    activations: Optional[StorageFile]
    muscle_forces: Optional[StorageFile]
    summary: Optional[dict]

    def has_muscles(self) -> bool:
        return self.activations is not None and self.muscle_forces is not None

    def files_present(self) -> list[str]:
        present = []
        if self.kinematics is not None:
            present.append("kinematics.mot")
        if self.inverse_dynamics is not None:
            present.append("inverse_dynamics.sto")
        if self.activations is not None:
            present.append("activations.sto")
        if self.muscle_forces is not None:
            present.append("muscle_forces.sto")
        return present


def load_export(folder: Path | str, stem: str) -> ExportBundle:
    """Load a (desktop or iOS) export folder. The same loader works for both
    sides because the file layout is identical by design."""
    folder = Path(folder).expanduser().resolve()
    if not folder.exists() or not folder.is_dir():
        raise FileNotFoundError(f"Export folder not found: {folder}")

    def _load(name: str) -> Optional[StorageFile]:
        p = folder / name
        return read_storage(p) if p.exists() else None

    kinematics = _load(f"{stem}_kinematics.mot")
    inverse_dynamics = _load(f"{stem}_inverse_dynamics.sto")
    activations = _load(f"{stem}_activations.sto")
    muscle_forces = _load(f"{stem}_muscle_forces.sto")

    summary_path = folder / f"{stem}_summary.json"
    summary: Optional[dict] = None
    if summary_path.exists():
        try:
            summary = json.loads(summary_path.read_text())
        except json.JSONDecodeError:
            summary = None

    if all(
        x is None
        for x in (kinematics, inverse_dynamics, activations, muscle_forces, summary)
    ):
        raise FileNotFoundError(
            f"No export files with stem '{stem}' found under {folder}. "
            f"Expected names like '{stem}_kinematics.mot', '{stem}_summary.json'."
        )

    return ExportBundle(
        folder=folder,
        stem=stem,
        kinematics=kinematics,
        inverse_dynamics=inverse_dynamics,
        activations=activations,
        muscle_forces=muscle_forces,
        summary=summary,
    )


def autodetect_stem(folder: Path | str) -> str:
    """If the user doesn't tell us the stem, infer it from `*_summary.json`
    or `*_kinematics.mot` in the folder."""
    folder = Path(folder)
    for pattern in ("*_summary.json", "*_kinematics.mot"):
        matches = sorted(folder.glob(pattern))
        if matches:
            name = matches[0].name
            for suffix in ("_summary.json", "_kinematics.mot"):
                if name.endswith(suffix):
                    return name[: -len(suffix)]
    raise FileNotFoundError(
        f"Could not autodetect stem in {folder}. Pass --stem explicitly."
    )
