"""OpenSim Inverse Kinematics driver.

Builds an `IKTaskSet` from the model's marker set automatically, runs
`InverseKinematicsTool`, and returns the path to the produced `.mot`.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from .trial import Trial


def _opensim():
    """Lazy import so the rest of the package is usable without OpenSim."""
    try:
        import opensim  # type: ignore
    except ImportError as exc:
        raise ImportError(
            "OpenSim Python bindings are required for IK/ID/SO. "
            "Install via `conda env create -f environment.yml`."
        ) from exc
    return opensim


def _build_marker_task_set(model, default_weight: float = 1.0):
    """Mirror every marker in the model into an IKMarkerTask with a unit weight.
    OpenCap-augmented models have ~63 markers; we keep them all by default."""
    osim = _opensim()
    task_set = osim.IKTaskSet()
    marker_set = model.getMarkerSet()
    for i in range(marker_set.getSize()):
        name = marker_set.get(i).getName()
        task = osim.IKMarkerTask()
        task.setName(name)
        task.setApply(True)
        task.setWeight(default_weight)
        task_set.cloneAndAppend(task)
    return task_set


def _trc_time_range(trc_path: Path) -> tuple[float, float]:
    osim = _opensim()
    table = osim.TimeSeriesTableVec3(str(trc_path))
    times = table.getIndependentColumn()
    return float(times[0]), float(times[-1])


def run_ik(
    trial: Trial,
    output_dir: Path,
    *,
    stem: Optional[str] = None,
    accuracy: float = 1e-5,
) -> Path:
    """Run IK and return the path to `<stem>_kinematics.mot`."""
    osim = _opensim()
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = stem or trial.name

    model = osim.Model(str(trial.osim))
    model.initSystem()

    task_set = _build_marker_task_set(model)

    t0, t1 = _trc_time_range(trial.trc)
    if trial.start_time is not None:
        t0 = max(t0, float(trial.start_time))
    if trial.end_time is not None:
        t1 = min(t1, float(trial.end_time))
    if t1 <= t0:
        raise ValueError(
            f"Empty IK time window for trial {trial.name}: [{t0}, {t1}]"
        )

    output_mot = output_dir / f"{stem}_kinematics.mot"

    ik = osim.InverseKinematicsTool()
    ik.setModel(model)
    ik.setMarkerDataFileName(str(trial.trc))
    ik.set_IKTaskSet(task_set)
    ik.setStartTime(t0)
    ik.setEndTime(t1)
    ik.setOutputMotionFileName(str(output_mot))
    ik.set_accuracy(accuracy)
    ik.setResultsDir(str(output_dir))

    ok = ik.run()
    if not ok:
        raise RuntimeError(
            f"InverseKinematicsTool returned False for trial {trial.name}"
        )
    if not output_mot.exists():
        raise RuntimeError(
            f"IK reported success but {output_mot} was not written."
        )
    return output_mot
