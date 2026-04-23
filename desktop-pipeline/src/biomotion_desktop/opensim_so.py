"""OpenSim Static Optimization driver.

Runs `StaticOptimization` analysis through `AnalyzeTool`. OpenSim names its
outputs `<prefix>_StaticOptimization_activation.sto` and
`<prefix>_StaticOptimization_force.sto`; we rename them to
`<stem>_activations.sto` and `<stem>_muscle_forces.sto` so the comparison
layer can pair them with the iOS exports purely by filename.

Default knobs (mirrors what opencap-processing's MuscleAnalysis setup uses
for OpenCap models):
- `use_muscle_physiology = True`
- `optimizer_max_iterations = 100`
- `activation_exponent = 2`
- residual reserve actuators are NOT auto-added; if your model has none and
  the SO fails to converge, supply a model with reserve actuators.
"""

from __future__ import annotations

import shutil
from pathlib import Path
from typing import Optional

from .trial import Trial


def _opensim():
    try:
        import opensim  # type: ignore
    except ImportError as exc:
        raise ImportError(
            "OpenSim Python bindings are required for IK/ID/SO. "
            "Install via `conda env create -f environment.yml`."
        ) from exc
    return opensim


def run_so(
    trial: Trial,
    output_dir: Path,
    *,
    ik_mot: Path,
    stem: Optional[str] = None,
) -> tuple[Path, Path]:
    """Run Static Optimization. Returns (activations_path, forces_path)."""
    osim = _opensim()
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = stem or trial.name
    ik_mot = Path(ik_mot)
    if not ik_mot.exists():
        raise FileNotFoundError(f"IK mot not found: {ik_mot}")

    t0, t1 = _mot_time_range(ik_mot)
    if trial.start_time is not None:
        t0 = max(t0, float(trial.start_time))
    if trial.end_time is not None:
        t1 = min(t1, float(trial.end_time))

    so = osim.StaticOptimization()
    so.setStartTime(t0)
    so.setEndTime(t1)
    so.setUseMusclePhysiology(True)
    so.setActivationExponent(2.0)
    so.setConvergenceCriterion(1e-4)
    so.setMaxIterations(100)

    # Build the AnalyzeTool, then write it to XML and reload it. This is the
    # canonical opencap-processing pattern — when AnalyzeTool is constructed
    # from a setup XML, it derives the state trajectory from the coordinates
    # file internally, which avoids the
    #   "verifyControlsStates: a storage object containing the time histories
    #    of states was not specified"
    # error you get when you try to wire a Model in programmatically.
    analyze = osim.AnalyzeTool()
    analyze.setName(stem)
    analyze.setModelFilename(str(trial.osim))
    analyze.setCoordinatesFileName(str(ik_mot))
    analyze.setLowpassCutoffFrequency(float(trial.lowpass_hz))
    analyze.setStartTime(t0)
    analyze.setFinalTime(t1)
    analyze.setResultsDir(str(output_dir))
    analyze.setSolveForEquilibrium(False)
    analyze.setReplaceForceSet(False)

    if trial.grf_xml is not None:
        analyze.setExternalLoadsFileName(str(trial.grf_xml))

    analyze.getAnalysisSet().cloneAndAppend(so)

    setup_xml = output_dir / f"{stem}_so_setup.xml"
    analyze.printToXML(str(setup_xml))

    runner = osim.AnalyzeTool(str(setup_xml))
    ok = runner.run()
    if not ok:
        raise RuntimeError(
            f"AnalyzeTool/StaticOptimization failed for trial {trial.name}"
        )

    raw_activations = output_dir / f"{stem}_StaticOptimization_activation.sto"
    raw_forces = output_dir / f"{stem}_StaticOptimization_force.sto"
    if not raw_activations.exists() or not raw_forces.exists():
        raise RuntimeError(
            "Static Optimization reported success but expected outputs are "
            f"missing:\n  {raw_activations}\n  {raw_forces}"
        )

    activations = output_dir / f"{stem}_activations.sto"
    forces = output_dir / f"{stem}_muscle_forces.sto"
    shutil.move(str(raw_activations), str(activations))
    shutil.move(str(raw_forces), str(forces))

    # AnalyzeTool may also drop a controls file we don't need.
    for stray in output_dir.glob(f"{stem}_StaticOptimization_*"):
        try:
            stray.unlink()
        except OSError:
            pass
    # And the setup XML — keep it under .opensim_setup so the user-facing
    # output dir mirrors what the iOS app produces (no clutter).
    try:
        setup_xml.unlink()
    except OSError:
        pass

    return activations, forces


def _mot_time_range(mot_path: Path) -> tuple[float, float]:
    from .storage_io import read_storage

    storage = read_storage(mot_path)
    times = storage.data.index.values
    if len(times) == 0:
        raise ValueError(f"{mot_path}: empty motion")
    return float(times[0]), float(times[-1])
