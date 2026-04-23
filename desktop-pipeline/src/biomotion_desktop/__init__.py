"""BioMotion desktop validation pipeline.

Public surface:
- `Trial`            — trial config dataclass.
- `run_pipeline`     — IK -> ID -> SO orchestration.
- `compare_folders`  — diff a desktop output folder against an iOS export.
"""

from .trial import Trial, load_trial
from .pipeline import PipelineResult, run_pipeline
from .compare import (
    ChannelMetrics,
    FileComparison,
    FolderComparison,
    compare_folders,
)

__all__ = [
    "Trial",
    "load_trial",
    "PipelineResult",
    "run_pipeline",
    "ChannelMetrics",
    "FileComparison",
    "FolderComparison",
    "compare_folders",
]

__version__ = "0.1.0"
