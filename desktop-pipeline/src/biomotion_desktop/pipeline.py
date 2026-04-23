"""End-to-end pipeline orchestration: IK -> ID -> SO -> summary.

Produces exactly the same five filenames the iOS app's
`OfflineAnalysisExporter` writes, so the comparator can pair them by name:

    <stem>_kinematics.mot
    <stem>_inverse_dynamics.sto
    <stem>_activations.sto
    <stem>_muscle_forces.sto
    <stem>_summary.json
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from .anthropometrics import apply_subject_overrides
from .trial import Trial
from .opensim_ik import run_ik
from .opensim_id import run_id
from .opensim_so import run_so
from .storage_io import read_storage


@dataclass(frozen=True)
class PipelineResult:
    output_dir: Path
    stem: str
    kinematics_mot: Path
    inverse_dynamics_sto: Path
    activations_sto: Optional[Path]
    muscle_forces_sto: Optional[Path]
    summary_json: Path
    timings_s: dict


def run_pipeline(
    trial: Trial,
    output_dir: Path,
    *,
    skip_so: bool = False,
) -> PipelineResult:
    """Run IK -> ID -> SO and write a summary JSON.

    `skip_so` lets you run just IK + ID for trials whose model has no
    muscles (rare, but a handy escape hatch).
    """
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = trial.name

    # Subject anthropometrics: rescale model body masses to match the
    # measured subject if `trial.subject_mass_kg` is set. No-op otherwise.
    # Done up front so IK/ID/SO all consume the same (possibly rescaled)
    # model. IK is mass-independent so this is harmless for IK.
    trial, anthropometrics_info = apply_subject_overrides(
        trial, output_dir, stem=stem
    )

    timings: dict[str, float] = {}

    t0 = time.perf_counter()
    ik_mot = run_ik(trial, output_dir, stem=stem)
    timings["ik_s"] = time.perf_counter() - t0

    t0 = time.perf_counter()
    id_sto = run_id(trial, output_dir, ik_mot=ik_mot, stem=stem)
    timings["id_s"] = time.perf_counter() - t0

    activations: Optional[Path] = None
    forces: Optional[Path] = None
    if not skip_so:
        t0 = time.perf_counter()
        activations, forces = run_so(trial, output_dir, ik_mot=ik_mot, stem=stem)
        timings["so_s"] = time.perf_counter() - t0

    summary_path = output_dir / f"{stem}_summary.json"
    summary = _build_summary(
        trial=trial,
        stem=stem,
        ik_mot=ik_mot,
        id_sto=id_sto,
        activations=activations,
        forces=forces,
        timings=timings,
        anthropometrics_info=anthropometrics_info,
    )
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

    return PipelineResult(
        output_dir=output_dir,
        stem=stem,
        kinematics_mot=ik_mot,
        inverse_dynamics_sto=id_sto,
        activations_sto=activations,
        muscle_forces_sto=forces,
        summary_json=summary_path,
        timings_s=timings,
    )


def _build_summary(
    *,
    trial: Trial,
    stem: str,
    ik_mot: Path,
    id_sto: Path,
    activations: Optional[Path],
    forces: Optional[Path],
    timings: dict,
    anthropometrics_info: dict,
) -> dict:
    kin = read_storage(ik_mot)
    n_frames = len(kin.data.index)
    duration = (
        float(kin.data.index.values[-1] - kin.data.index.values[0])
        if n_frames > 1
        else 0.0
    )
    fps = kin.fps
    muscle_count = 0
    if activations is not None:
        muscle_count = len(read_storage(activations).columns)

    opensim_version = "unknown"
    try:
        import opensim  # type: ignore

        opensim_version = str(opensim.GetVersion())
    except Exception:  # pragma: no cover — only fails without opensim
        pass

    return {
        "stem": stem,
        "source": "desktop",
        "model": str(trial.osim),
        "motion": str(trial.trc),
        "frame_count": n_frames,
        "duration_s": duration,
        "fps": fps,
        "dof_count": len(kin.columns),
        "muscle_count": muscle_count,
        "lowpass_hz": trial.lowpass_hz,
        "start_time": trial.start_time,
        "end_time": trial.end_time,
        "grf_mot": str(trial.grf_mot) if trial.grf_mot else None,
        "grf_xml": str(trial.grf_xml) if trial.grf_xml else None,
        "subject_mass_kg": trial.subject_mass_kg,
        "subject_height_m": trial.subject_height_m,
        "anthropometrics": anthropometrics_info,
        "timings_s": timings,
        "opensim_version": opensim_version,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "outputs": {
            "kinematics_mot": str(ik_mot.name),
            "inverse_dynamics_sto": str(id_sto.name),
            "activations_sto": activations.name if activations else None,
            "muscle_forces_sto": forces.name if forces else None,
        },
    }
