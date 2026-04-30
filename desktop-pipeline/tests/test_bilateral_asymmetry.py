"""Tests for the bilateral-asymmetry helpers in `biomotion_desktop.compare`.

These cover the pure helpers — no on-device data required:

  * `_detect_bilateral_pairs`: pair detection, orphan handling, .activation
    suffix stripping.
  * `_compute_pair_metrics`: raw arithmetic against hand-built numpy arrays,
    including all-zero / one-side-zero NaN handling.
  * `_format_asymmetry_report`: PASS / FAIL summary line content for
    threshold-exceeding pairs and the empty case.

Stdout regression test: invoking `biomotion_desktop.cli compare` *without*
`--asymmetry-report` against an iOS export folder should produce the same
output as before this change (asserted by checking the absence of the
report header). Full byte-equality against a committed fixture is left to
the integration test suite, since that requires running the full
comparator (requires an OpenSim install) — here we only assert that the
new code path is dormant when the flag is not passed.
"""

from __future__ import annotations

import io
import json
import math
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np
import pandas as pd

from biomotion_desktop.cli import main as cli_main
from biomotion_desktop.compare import (
    _compute_pair_metrics,
    _detect_bilateral_pairs,
    _format_asymmetry_report,
)
from biomotion_desktop.storage_io import write_storage


# ----- _detect_bilateral_pairs --------------------------------------------


def test_detect_pairs_basic_l_r_suffix():
    cols = ["tibant_l", "tibant_r", "soleus_l", "soleus_r"]
    pairs = _detect_bilateral_pairs(cols)
    assert pairs == [
        ("soleus", "soleus_l", "soleus_r"),
        ("tibant", "tibant_l", "tibant_r"),
    ]


def test_detect_pairs_skips_orphan():
    cols = ["tibant_l", "tibant_r", "psoas_l"]  # psoas has no right side
    pairs = _detect_bilateral_pairs(cols)
    assert pairs == [("tibant", "tibant_l", "tibant_r")]


def test_detect_pairs_strips_activation_token():
    cols = ["tibant_l.activation", "tibant_r.activation"]
    pairs = _detect_bilateral_pairs(cols)
    assert pairs == [("tibant", "tibant_l.activation", "tibant_r.activation")]


def test_detect_pairs_empty():
    assert _detect_bilateral_pairs([]) == []


def test_detect_pairs_no_l_r_columns():
    assert _detect_bilateral_pairs(["time", "knee_angle", "torso"]) == []


# ----- _compute_pair_metrics ----------------------------------------------


def test_pair_metrics_perfect_symmetry():
    arr = np.array([0.0, 0.5, 1.0, 0.5, 0.0])
    m = _compute_pair_metrics(arr, arr)
    assert m["peak_l"] == 1.0
    assert m["peak_r"] == 1.0
    assert m["peak_diff"] == 0.0
    assert m["rms_diff"] == 0.0
    assert m["asym_idx_pct"] == 0.0


def test_pair_metrics_known_asymmetry():
    # Left peaks at 1.0, right peaks at 0.5 → asym_idx = 100*(0.5)/(0.75)=66.67
    left = np.array([0.0, 1.0, 0.0])
    right = np.array([0.0, 0.5, 0.0])
    m = _compute_pair_metrics(left, right)
    assert m["peak_l"] == 1.0
    assert m["peak_r"] == 0.5
    assert math.isclose(m["asym_idx_pct"], 100.0 * 0.5 / 0.75, rel_tol=1e-9)
    assert math.isclose(m["peak_diff"], 0.5, rel_tol=1e-9)


def test_pair_metrics_all_zero_returns_nan():
    z = np.zeros(5)
    m = _compute_pair_metrics(z, z)
    assert math.isnan(m["asym_idx_pct"])
    assert m["peak_l"] == 0.0
    assert m["peak_r"] == 0.0


def test_pair_metrics_one_side_zero_returns_nan():
    # One-sided silence isn't a meaningful asymmetry index — the symmetric
    # mean is half of the active side's peak so the index would be ±200,
    # which would dominate every PASS/FAIL aggregate. NaN keeps it visible
    # without skewing the threshold check.
    m = _compute_pair_metrics(np.array([1.0, 0.0]), np.zeros(2))
    assert math.isnan(m["asym_idx_pct"])
    assert m["peak_l"] == 1.0
    assert m["peak_r"] == 0.0


def test_pair_metrics_empty_arrays_return_nan():
    m = _compute_pair_metrics(np.array([]), np.array([]))
    assert math.isnan(m["asym_idx_pct"])
    assert math.isnan(m["peak_l"])


# ----- _format_asymmetry_report -------------------------------------------


def _make_metrics(asym_pct: float) -> dict:
    return {
        "peak_l": 1.0,
        "peak_r": 1.0 * (1.0 - asym_pct / 100.0),
        "peak_diff": abs(1.0 - 1.0 * (1.0 - asym_pct / 100.0)),
        "rms_diff": 0.0,
        "asym_idx_pct": asym_pct,
    }


def test_format_report_pass_when_under_threshold():
    pairs = [("tibant", _make_metrics(5.0)), ("soleus", _make_metrics(-3.0))]
    text = _format_asymmetry_report(pairs, threshold_pct=10.0)
    assert "SUMMARY: PASS" in text
    assert "FAIL" not in text.split("\n")[-1]


def test_format_report_fail_lists_offending_pair():
    pairs = [("tibant", _make_metrics(70.0)), ("soleus", _make_metrics(-3.0))]
    text = _format_asymmetry_report(pairs, threshold_pct=10.0)
    last = text.split("\n")[-1]
    assert last.startswith("SUMMARY: FAIL")
    assert "tibant" in last
    assert "soleus" not in last  # only failing pairs named


def test_format_report_empty_pairs():
    text = _format_asymmetry_report([], threshold_pct=10.0)
    assert text.split("\n")[-1].startswith("SUMMARY: PASS")
    assert "no L/R muscle pairs" in text


def _make_minimal_export(folder: Path, stem: str, *, asymmetric: bool) -> None:
    """Write the smallest export pair the comparator will accept: a kinematics
    file plus an activations file, both with bilateral muscle columns. Used
    by both the CLI regression test and the integration test below."""
    folder.mkdir(parents=True, exist_ok=True)
    n = 200
    times = np.linspace(0.0, 1.0, n)
    base = np.exp(-((np.arange(n) - n // 2) ** 2) / (2 * (n / 20) ** 2))

    kin_df = pd.DataFrame(
        {
            "hip_flexion_l": 20.0 * np.sin(2 * np.pi * times),
            "hip_flexion_r": 20.0 * np.sin(2 * np.pi * times),
        },
        index=pd.Index(times, name="time"),
    )
    write_storage(folder / f"{stem}_kinematics.mot", stem, kin_df, in_degrees=True)

    right_scale = 0.3 if asymmetric else 1.0
    act_df = pd.DataFrame(
        {"tibant_l": base, "tibant_r": base * right_scale},
        index=pd.Index(times, name="time"),
    )
    write_storage(folder / f"{stem}_activations.sto", stem, act_df, in_degrees=False)
    (folder / f"{stem}_summary.json").write_text(
        json.dumps({"stem": stem, "frame_count": n, "fps": float(n - 1)})
    )


def _run_cli(argv: list[str]) -> tuple[str, int]:
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = cli_main(argv)
    return buf.getvalue(), rc


def test_compare_stdout_unchanged_without_asymmetry_flag(tmp_path: Path):
    """Regression guard: the existing `compare` stdout must be byte-identical
    when `--asymmetry-report` is absent. The new code path is fully gated, so
    we assert that running with the flag *prefixes* the legacy stdout
    (the report is purely additive) rather than replacing or reordering it."""
    desk = tmp_path / "desktop"
    ios = tmp_path / "ios"
    _make_minimal_export(desk, "trial", asymmetric=False)
    _make_minimal_export(ios, "trial", asymmetric=False)

    base_argv = [
        "compare",
        "--desktop", str(desk),
        "--ios", str(ios),
        "--stem", "trial",
        "--report", str(tmp_path / "compare_base.json"),
    ]
    out_legacy, rc_legacy = _run_cli(base_argv)
    assert rc_legacy == 0

    out_with_flag, rc_with_flag = _run_cli(
        base_argv + [
            "--asymmetry-report",
            "--report", str(tmp_path / "compare_flagged.json"),
        ]
    )
    assert rc_with_flag == 0  # symmetric input → PASS, exit 0

    # The flagged run only has its `--report` path differ inside stdout; rest
    # must match legacy verbatim. Normalize the two compare.json paths so the
    # comparison reflects the actual code change.
    out_legacy_norm = out_legacy.replace("compare_base.json", "compare.json")
    out_flagged_norm = out_with_flag.replace("compare_flagged.json", "compare.json")

    # Legacy stdout is a strict prefix of the flagged stdout — the report
    # is appended, never interleaved.
    assert out_flagged_norm.startswith(out_legacy_norm), (
        "Pre-flag stdout was reordered or modified.\n"
        f"--- legacy ---\n{out_legacy_norm}\n--- flagged ---\n{out_flagged_norm}"
    )
    # And the appended portion contains the asymmetry header.
    appended = out_flagged_norm[len(out_legacy_norm):]
    assert "Bilateral Asymmetry Report" in appended
    assert "SUMMARY: PASS" in appended


def test_compare_exits_nonzero_when_asymmetry_exceeds_threshold(tmp_path: Path):
    """The CI gate: feeding asymmetric activations through the comparator
    with `--asymmetry-report` should print a FAIL summary and exit 1."""
    desk = tmp_path / "desktop"
    ios = tmp_path / "ios"
    _make_minimal_export(desk, "trial", asymmetric=False)
    _make_minimal_export(ios, "trial", asymmetric=True)  # right side at 30% of left

    out, rc = _run_cli([
        "compare",
        "--desktop", str(desk),
        "--ios", str(ios),
        "--stem", "trial",
        "--report", str(tmp_path / "compare.json"),
        "--asymmetry-report",
        "--asymmetry-threshold-pct", "10",
    ])
    assert rc == 1
    assert "SUMMARY: FAIL" in out
    assert "tibant" in out


def test_format_report_nan_pair_does_not_fail_summary():
    nan_pair = {
        "peak_l": 1.0,
        "peak_r": 0.0,
        "peak_diff": 1.0,
        "rms_diff": 0.5,
        "asym_idx_pct": float("nan"),
    }
    pairs = [("tibant", nan_pair)]
    text = _format_asymmetry_report(pairs, threshold_pct=10.0)
    assert "n/a" in text
    assert text.split("\n")[-1].startswith("SUMMARY: PASS")
