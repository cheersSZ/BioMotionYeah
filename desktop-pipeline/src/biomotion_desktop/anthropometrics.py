"""Subject anthropometric override for the OpenSim model.

When a `Trial` carries `subject_mass_kg`, this module writes a lightly modified
copy of the model with every body's mass uniformly rescaled so the total
matches the measured value. It returns the new `Trial` (with `osim` repointed)
so downstream IK/ID/SO drivers consume the rescaled model transparently.

Why "uniform rescale" instead of de Leva regression:
- The model is already segment-scaled (LaiUhlrich2022_*scaled*.osim) — body
  geometry is per-subject. Only the *mass* fields are still at default.
- Uniform scaling preserves the relative mass distribution the modeller chose
  and is exactly what OpenSim's ScaleTool does in its mass-preservation step.
- de Leva-style per-segment recomputation only pays off when we also rescale
  geometry, which is out of scope until the GRF estimator lands.

If `subject_mass_kg` is None, this is a no-op and the original trial is
returned unchanged. The function never mutates the source `.osim` on disk —
the rescaled model is written into `output_dir` as `<stem>_subject_scaled.osim`.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Optional

from .trial import Trial


def apply_subject_overrides(
    trial: Trial,
    output_dir: Path,
    *,
    stem: Optional[str] = None,
) -> tuple[Trial, dict]:
    """Return a `(trial, info)` pair.

    `info` documents what was applied so the caller can record it in the
    summary JSON. When no overrides are present, info["mass_override"] is
    None and the original trial is returned untouched.
    """
    info: dict = {"mass_override": None, "height_recorded_m": trial.subject_height_m}

    if trial.subject_mass_kg is None:
        return trial, info

    try:
        import opensim  # type: ignore
    except ImportError as exc:  # pragma: no cover — same guard as other drivers
        raise ImportError(
            "OpenSim Python bindings are required for anthropometric overrides."
        ) from exc

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = stem or trial.name

    model = opensim.Model(str(trial.osim))
    model.initSystem()

    body_set = model.getBodySet()
    original_total = 0.0
    bodies = []
    for i in range(body_set.getSize()):
        body = body_set.get(i)
        m = float(body.getMass())
        original_total += m
        bodies.append((body, m))

    if original_total <= 0.0:
        # Defensive: a model with zero total mass cannot be uniformly rescaled.
        # This should never happen with a real .osim file; if it does, we want
        # to fail loudly rather than silently produce a NaN-mass model.
        raise ValueError(
            f"Model {trial.osim} reports total body mass {original_total} kg; "
            "cannot apply subject mass override."
        )

    target = float(trial.subject_mass_kg)
    scale = target / original_total

    for body, original_mass in bodies:
        body.setMass(original_mass * scale)

    rescaled_path = output_dir / f"{stem}_subject_scaled.osim"
    model.printToXML(str(rescaled_path))

    info["mass_override"] = {
        "source_model": str(trial.osim),
        "rescaled_model": str(rescaled_path),
        "original_total_kg": original_total,
        "target_total_kg": target,
        "scale_factor": scale,
    }

    return dataclasses.replace(trial, osim=rescaled_path), info
