"""OpenSim Inverse Dynamics driver.

Takes the IK output (`.mot`), low-pass filters the kinematics, and runs
`InverseDynamicsTool` to produce per-DOF generalized forces. GRF is optional;
without it pelvis residuals will be large (expected for OpenCap field data
with no force plates — we keep the diff against iOS, not the absolute physics).
"""

from __future__ import annotations

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


def run_id(
    trial: Trial,
    output_dir: Path,
    *,
    ik_mot: Path,
    stem: Optional[str] = None,
) -> Path:
    """Run InverseDynamicsTool. Returns path to `<stem>_inverse_dynamics.sto`.

    The .sto is written with `inDegrees=no` and one column per DOF, matching
    the iOS exporter (`OfflineAnalysisExporter.stoTextForInverseDynamics`).
    """
    osim = _opensim()
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = stem or trial.name
    ik_mot = Path(ik_mot)
    if not ik_mot.exists():
        raise FileNotFoundError(f"IK mot not found: {ik_mot}")

    model = osim.Model(str(trial.osim))
    model.initSystem()

    coord_set = model.getCoordinateSet()
    excluded = osim.ArrayStr()
    excluded.append("ground")  # never excluded but OpenSim wants the field set

    id_tool = osim.InverseDynamicsTool()
    id_tool.setModel(model)
    id_tool.setModelFileName(str(trial.osim))
    id_tool.setCoordinatesFileName(str(ik_mot))
    id_tool.setLowpassCutoffFrequency(float(trial.lowpass_hz))
    id_tool.setStartTime(_window_start(trial, ik_mot))
    id_tool.setEndTime(_window_end(trial, ik_mot))
    id_tool.setResultsDir(str(output_dir))

    output_name = f"{stem}_inverse_dynamics.sto"
    id_tool.setOutputGenForceFileName(output_name)

    # External loads (force plates). Optional.
    if trial.grf_xml is not None:
        id_tool.setExternalLoadsFileName(str(trial.grf_xml))

    # Make sure all coordinates participate (default behavior, but explicit).
    excluded_coords = osim.ArrayStr()
    id_tool.setExcludedForces(excluded_coords)

    ok = id_tool.run()
    if not ok:
        raise RuntimeError(f"InverseDynamicsTool failed for trial {trial.name}")

    out = output_dir / output_name
    if not out.exists():
        raise RuntimeError(f"ID reported success but {out} was not written.")
    return out


def _window_start(trial: Trial, ik_mot: Path) -> float:
    if trial.start_time is not None:
        return float(trial.start_time)
    return _mot_time_range(ik_mot)[0]


def _window_end(trial: Trial, ik_mot: Path) -> float:
    if trial.end_time is not None:
        return float(trial.end_time)
    return _mot_time_range(ik_mot)[1]


def _mot_time_range(mot_path: Path) -> tuple[float, float]:
    """Read first/last time stamp from a .mot without needing OpenSim."""
    from .storage_io import read_storage

    storage = read_storage(mot_path)
    times = storage.data.index.values
    if len(times) == 0:
        raise ValueError(f"{mot_path}: empty motion")
    return float(times[0]), float(times[-1])
