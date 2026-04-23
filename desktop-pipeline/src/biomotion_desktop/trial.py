"""Trial config: a thin dataclass + YAML loader.

A `Trial` is everything the desktop pipeline needs to reproduce one motion:
the scaled OpenSim model, the marker file, optional GRF, and the analysis
window. Paths in the YAML are resolved relative to the YAML file's directory
so configs can stay portable across machines.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml


@dataclass(frozen=True)
class Trial:
    name: str
    osim: Path
    trc: Path
    grf_mot: Optional[Path] = None
    grf_xml: Optional[Path] = None
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    lowpass_hz: float = 6.0
    # Subject anthropometrics. When `subject_mass_kg` is set, the pipeline
    # uniformly rescales every body's mass in the model so the total matches
    # the measured value before running ID/SO — this directly improves the
    # pelvis residual and per-muscle force magnitudes when no GRF is
    # available (the dominant failure mode of OpenCap-style data; see
    # OPEN_ISSUES §4.1).
    #
    # `subject_height_m` is currently stored only for traceability — it is
    # reserved for the future contact-implicit GRF estimator (de Leva-style
    # segment scaling, foot contact sphere placement) and is intentionally
    # NOT used by IK/ID/SO yet. We accept it now so trial configs don't have
    # to be re-versioned later.
    subject_mass_kg: Optional[float] = None
    subject_height_m: Optional[float] = None
    extra: dict = field(default_factory=dict)

    def required_paths(self) -> list[Path]:
        paths = [self.osim, self.trc]
        if self.grf_mot is not None:
            paths.append(self.grf_mot)
        if self.grf_xml is not None:
            paths.append(self.grf_xml)
        return paths

    def validate_exists(self) -> None:
        missing = [str(p) for p in self.required_paths() if not p.exists()]
        if missing:
            raise FileNotFoundError(
                "Trial references files that do not exist:\n  - "
                + "\n  - ".join(missing)
            )


def load_trial(config_path: Path | str) -> Trial:
    """Load a Trial from a YAML file. Relative paths are resolved against
    the YAML file's parent directory."""
    config_path = Path(config_path).expanduser().resolve()
    if not config_path.exists():
        raise FileNotFoundError(f"Trial config not found: {config_path}")

    with config_path.open("r") as fh:
        raw = yaml.safe_load(fh) or {}

    base = config_path.parent

    def _resolve(value) -> Optional[Path]:
        if value is None:
            return None
        p = Path(value)
        if not p.is_absolute():
            p = (base / p).resolve()
        return p

    name = raw.get("name") or config_path.stem
    osim = _resolve(raw.get("osim"))
    trc = _resolve(raw.get("trc"))
    if osim is None or trc is None:
        raise ValueError(
            f"Trial config {config_path} must define both 'osim' and 'trc'."
        )

    known = {
        "name", "osim", "trc",
        "grf_mot", "grf_xml",
        "start_time", "end_time",
        "lowpass_hz",
        "subject_mass_kg", "subject_height_m",
    }
    extra = {k: v for k, v in raw.items() if k not in known}

    def _opt_float(key: str) -> Optional[float]:
        v = raw.get(key)
        return None if v is None else float(v)

    trial = Trial(
        name=str(name),
        osim=osim,
        trc=trc,
        grf_mot=_resolve(raw.get("grf_mot")),
        grf_xml=_resolve(raw.get("grf_xml")),
        start_time=raw.get("start_time"),
        end_time=raw.get("end_time"),
        lowpass_hz=float(raw.get("lowpass_hz", 6.0)),
        subject_mass_kg=_opt_float("subject_mass_kg"),
        subject_height_m=_opt_float("subject_height_m"),
        extra=extra,
    )
    trial.validate_exists()
    return trial
