"""Smoke tests that run without OpenSim installed.

Cover:
- storage_io: read/write roundtrip
- compare:    end-to-end on synthetic data with a known phase shift,
              ROM compression, and channel mismatch
- ios_export: load from a synthetic export folder
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from biomotion_desktop import compare
from biomotion_desktop.compare import compare_folders, write_report
from biomotion_desktop.ios_export import autodetect_stem, load_export
from biomotion_desktop.storage_io import (
    StorageFile,
    common_time_grid,
    read_storage,
    resample,
    write_storage,
)


# ---------------------------------------------------------------------------
# storage_io
# ---------------------------------------------------------------------------


def test_storage_roundtrip(tmp_path: Path) -> None:
    times = np.linspace(0, 1, 11)
    df = pd.DataFrame(
        {"hip_flexion_r": np.sin(times), "knee_angle_r": np.cos(times)},
        index=pd.Index(times, name="time"),
    )
    out = tmp_path / "demo_kinematics.mot"
    write_storage(out, "demo", df, in_degrees=True)

    loaded = read_storage(out)
    assert loaded.in_degrees is True
    assert loaded.columns == ["hip_flexion_r", "knee_angle_r"]
    np.testing.assert_allclose(loaded.data.values, df.values, rtol=1e-5)


def test_resample_drops_out_of_range() -> None:
    times = np.linspace(0, 1, 6)
    df = pd.DataFrame({"x": np.arange(6.0)}, index=pd.Index(times, name="time"))
    new = np.linspace(-0.5, 1.5, 21)
    out = resample(df, new)
    # Anything < 0 or > 1 is dropped, not extrapolated.
    assert out.index.min() >= 0.0
    assert out.index.max() <= 1.0


def test_common_time_grid_picks_overlap() -> None:
    t_a = np.linspace(0.0, 1.0, 11)   # dt=0.1
    t_b = np.linspace(0.5, 1.5, 11)   # dt=0.1
    a = pd.DataFrame({"x": np.zeros_like(t_a)}, index=pd.Index(t_a, name="time"))
    b = pd.DataFrame({"x": np.zeros_like(t_b)}, index=pd.Index(t_b, name="time"))
    grid = common_time_grid([a, b])
    assert grid[0] == pytest.approx(0.5)
    assert grid[-1] == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# compare
# ---------------------------------------------------------------------------


def _write_mot(path: Path, stem: str, columns: list[str], samples: np.ndarray,
               *, in_degrees: bool, kind: str) -> None:
    """Helper to write a synthetic .mot/.sto with a known time grid."""
    times = np.linspace(0, 1, samples.shape[0])
    df = pd.DataFrame(samples, columns=columns, index=pd.Index(times, name="time"))
    suffix = {
        "kin": "_kinematics.mot",
        "id":  "_inverse_dynamics.sto",
        "act": "_activations.sto",
        "frc": "_muscle_forces.sto",
    }[kind]
    write_storage(path / f"{stem}{suffix}", stem, df, in_degrees=in_degrees)


def _write_summary(folder: Path, stem: str, payload: dict) -> None:
    (folder / f"{stem}_summary.json").write_text(json.dumps(payload))


def _make_synthetic_export(
    folder: Path,
    stem: str,
    *,
    rom_scale: float = 1.0,
    phase_shift_samples: int = 0,
    activation_offset_samples: int = 0,
    extra_kinematics_col: str | None = None,
) -> None:
    """Build a fake export directory. `phase_shift_samples` rolls every signal
    forward by N samples; `rom_scale` scales the kinematics signal to simulate
    range-of-motion compression."""
    folder.mkdir(parents=True, exist_ok=True)
    n = 200
    t = np.linspace(0, 1, n)

    # Kinematics: a sine + cosine in degrees.
    base_kin = np.column_stack([
        20.0 * np.sin(2 * np.pi * t),
        15.0 * np.cos(2 * np.pi * t),
    ])
    base_kin = np.roll(base_kin, phase_shift_samples, axis=0) * rom_scale
    kin_cols = ["hip_flexion_r", "knee_angle_r"]
    if extra_kinematics_col is not None:
        # Add a column the OTHER side won't have.
        base_kin = np.column_stack([base_kin, np.zeros(n)])
        kin_cols = kin_cols + [extra_kinematics_col]
    _write_mot(folder, stem, kin_cols, base_kin, in_degrees=True, kind="kin")

    # ID: torque-like signal in Nm.
    id_data = np.column_stack([
        50.0 * np.sin(2 * np.pi * t),
        30.0 * np.cos(2 * np.pi * t),
    ])
    id_data = np.roll(id_data, phase_shift_samples, axis=0)
    _write_mot(folder, stem, ["hip_flexion_r", "knee_angle_r"], id_data,
               in_degrees=False, kind="id")

    # Activations: a Gaussian burst. Shift by `activation_offset_samples`
    # so onset/peak/offset diffs are non-zero between sides.
    burst = np.exp(-((np.arange(n) - n // 2) ** 2) / (2 * (n / 20) ** 2))
    burst_shifted = np.roll(burst, activation_offset_samples)
    act = np.column_stack([burst_shifted, burst_shifted * 0.5])
    _write_mot(folder, stem, ["soleus_r", "gastroc_r"], act,
               in_degrees=False, kind="act")

    # Forces follow the same shape (just scaled).
    frc = act * 800.0
    _write_mot(folder, stem, ["soleus_r", "gastroc_r"], frc,
               in_degrees=False, kind="frc")

    _write_summary(folder, stem, {
        "stem": stem,
        "frame_count": n,
        "fps": float(n - 1),  # 199 Hz for 200 samples over 1 s
    })


def test_compare_detects_phase_lag_and_rom_compression(tmp_path: Path) -> None:
    desk = tmp_path / "desktop"
    ios = tmp_path / "ios"
    _make_synthetic_export(desk, "trial", rom_scale=1.0, phase_shift_samples=0,
                           activation_offset_samples=0)
    _make_synthetic_export(ios,  "trial", rom_scale=0.8,  # iOS clips ROM by 20%
                           phase_shift_samples=4,         # iOS lags by 4 samples
                           activation_offset_samples=10)  # muscle onset later

    report = compare_folders(desk, ios, "trial")

    # Tier 1 sanity
    kin = report.files["kinematics.mot"]
    assert kin.overall["channel_count"] == 2
    assert kin.overall["rmse_mean"] > 0
    assert 0.0 < kin.overall["r2_mean"] <= 1.0

    # Tier 2 ROM compression: iOS - desktop should be negative for both DOFs.
    rom_err = kin.tier2["rom_error"]
    assert rom_err["hip_flexion_r"] < 0
    assert rom_err["knee_angle_r"] < 0

    # Tier 2 phase lag: positive (iOS lags desktop)
    lag = kin.tier2["phase_lag_ms"]
    assert lag["hip_flexion_r"] != 0

    # Tier 3 muscle timing: shifted by ~10 samples on a 199-Hz grid.
    act = report.files["activations.sto"]
    assert act.tier3 is not None
    onset = act.tier3["onset_diff_ms"]
    assert "soleus_r" in onset
    assert onset["soleus_r"] > 0  # iOS onset later

    # JSON serialization round-trip must work.
    out = tmp_path / "report.json"
    write_report(report, out)
    loaded = json.loads(out.read_text())
    assert loaded["stem"] == "trial"
    assert "kinematics.mot" in loaded["files"]


def test_compare_strips_opensim_id_suffixes(tmp_path: Path) -> None:
    """Desktop ID columns carry OpenSim's `_moment` / `_force` tags; iOS does not.
    The compare layer must normalise those out so the same DOFs line up."""
    desk = tmp_path / "desktop"
    ios = tmp_path / "ios"
    desk.mkdir()
    ios.mkdir()
    n = 100
    times = np.linspace(0, 1, n)
    rot = 50.0 * np.sin(2 * np.pi * times)
    trans = 5.0 * np.cos(2 * np.pi * times)

    desk_df = pd.DataFrame(
        {"ankle_angle_r_moment": rot, "pelvis_tx_force": trans},
        index=pd.Index(times, name="time"),
    )
    ios_df = pd.DataFrame(
        {"ankle_angle_r": rot, "pelvis_tx": trans},
        index=pd.Index(times, name="time"),
    )
    write_storage(desk / "trial_inverse_dynamics.sto", "trial", desk_df, in_degrees=False)
    write_storage(ios / "trial_inverse_dynamics.sto", "trial", ios_df, in_degrees=False)

    # Both folders also need a kinematics file so load_export doesn't throw.
    kin = pd.DataFrame({"hip_flexion_r": rot}, index=pd.Index(times, name="time"))
    write_storage(desk / "trial_kinematics.mot", "trial", kin, in_degrees=True)
    write_storage(ios / "trial_kinematics.mot", "trial", kin, in_degrees=True)

    report = compare_folders(desk, ios, "trial")
    id_cmp = report.files["inverse_dynamics.sto"]
    assert id_cmp.overall["channel_count"] == 2
    assert "ankle_angle_r" in id_cmp.channels
    assert "pelvis_tx" in id_cmp.channels
    assert id_cmp.unmatched["desktop_only"] == []
    assert id_cmp.unmatched["ios_only"] == []
    # Identical signals → RMSE essentially zero.
    assert id_cmp.overall["rmse_max"] < 1e-9


def test_compare_excludes_locked_dofs_from_aggregates(tmp_path: Path) -> None:
    """A DOF that's locked on the desktop side (e.g. OpenSim's mtp_angle)
    must be reported per-channel but not pollute rmse_mean / r2_mean."""
    desk = tmp_path / "desktop"
    ios = tmp_path / "ios"
    desk.mkdir()
    ios.mkdir()
    n = 200
    times = np.linspace(0, 1, n)
    moving = 20.0 * np.sin(2 * np.pi * times)
    locked_zero = np.zeros(n)
    nonzero_const = np.full(n, 7.5)

    # Both sides expose the same two channels. hip_flexion_r matches well;
    # mtp_angle_l is locked at 0 on desktop and held at +7.5° on iOS.
    desk_df = pd.DataFrame(
        {"hip_flexion_r": moving, "mtp_angle_l": locked_zero},
        index=pd.Index(times, name="time"),
    )
    ios_df = pd.DataFrame(
        {"hip_flexion_r": moving, "mtp_angle_l": nonzero_const},
        index=pd.Index(times, name="time"),
    )
    write_storage(desk / "trial_kinematics.mot", "trial", desk_df, in_degrees=True)
    write_storage(ios / "trial_kinematics.mot", "trial", ios_df, in_degrees=True)

    report = compare_folders(desk, ios, "trial")
    kin = report.files["kinematics.mot"]

    # Per-channel data is preserved for both DOFs, including the locked one.
    assert "mtp_angle_l" in kin.channels
    assert "hip_flexion_r" in kin.channels
    assert kin.channels["mtp_angle_l"].status == "locked_both"
    assert kin.channels["hip_flexion_r"].status == "ok"

    # Aggregates count both channels but only score the moving one.
    assert kin.overall["channel_count"] == 2
    assert kin.overall["scored_count"] == 1
    assert kin.overall["locked_channels"] == ["mtp_angle_l"]
    # The moving channel matches exactly → mean RMSE essentially zero.
    assert kin.overall["rmse_max"] < 1e-9


def test_compare_flags_sign_flip_suspect(tmp_path: Path) -> None:
    """A kinematics channel that's anti-correlated (sign-convention flipped
    between Nimble and OpenSim) should appear in tier2.sign_flip_suspects."""
    desk = tmp_path / "desktop"
    ios = tmp_path / "ios"
    desk.mkdir()
    ios.mkdir()
    n = 400
    times = np.linspace(0, 1, n)
    swing = 30.0 * np.sin(2 * np.pi * 3 * times)

    desk_df = pd.DataFrame(
        {"hip_flexion_r": swing, "knee_angle_r": swing},
        index=pd.Index(times, name="time"),
    )
    ios_df = pd.DataFrame(
        # hip matches; knee is the same shape but flipped sign + small noise
        {"hip_flexion_r": swing, "knee_angle_r": -swing + 0.5},
        index=pd.Index(times, name="time"),
    )
    write_storage(desk / "trial_kinematics.mot", "trial", desk_df, in_degrees=True)
    write_storage(ios / "trial_kinematics.mot", "trial", ios_df, in_degrees=True)

    report = compare_folders(desk, ios, "trial")
    suspects = report.files["kinematics.mot"].tier2["sign_flip_suspects"]
    assert "knee_angle_r" in suspects
    assert "hip_flexion_r" not in suspects
    s = suspects["knee_angle_r"]
    # Flipping desktop should drop RMSE far below the original.
    assert s["flipped_rmse"] < s["original_rmse"]
    assert s["rmse_ratio"] < 0.1
    # Pearson R² is invariant under sign flip and stays high.
    assert s["r2"] > 0.99


def test_compare_phase_compensation_recovers_uniform_shift(tmp_path: Path) -> None:
    """When the two pipelines disagree by a constant N-sample whole-body time
    shift (the fingerprint of mismatched filter topology — e.g., causal
    1-Euro vs zero-phase Butterworth filtfilt), the phase-compensation tier
    should:
      • recover the shift on every channel,
      • cut RMSE far below the original,
      • report uniform sign across channels in the aggregate.

    Convention exercised here: positive `best_lag_samples` means iOS leads
    desktop. The motivating real-data finding was +3 samples / +25 ms on
    `knee_angle_r` — desktop's heavy 6 Hz filtfilt smoothed peaks and lagged
    iOS's lightly-filtered signal.
    """
    desk = tmp_path / "desktop"
    ios = tmp_path / "ios"
    n = 600
    times = np.linspace(0, 1, n)
    base_kin = np.column_stack([
        20.0 * np.sin(2 * np.pi * 4 * times),
        15.0 * np.cos(2 * np.pi * 4 * times),
    ])

    # Inject "iOS leads desktop by 12 samples". `np.roll(arr, -k)` advances
    # the signal so element i becomes element (i + k) — i.e., the rolled
    # signal reaches each value k samples earlier. Use a shift comfortably
    # above PHASE_COMPENSATION_MIN_LAG_MS (= 12 ms at 599 Hz fps).
    shift = 12
    desk.mkdir(); ios.mkdir()
    df_d = pd.DataFrame(base_kin, columns=["hip_flexion_r", "knee_angle_r"],
                        index=pd.Index(times, name="time"))
    ios_kin = np.roll(base_kin, -shift, axis=0)
    df_i = pd.DataFrame(ios_kin, columns=["hip_flexion_r", "knee_angle_r"],
                        index=pd.Index(times, name="time"))
    write_storage(desk / "trial_kinematics.mot", "trial", df_d, in_degrees=True)
    write_storage(ios  / "trial_kinematics.mot", "trial", df_i, in_degrees=True)

    report = compare_folders(desk, ios, "trial")
    tier2 = report.files["kinematics.mot"].tier2
    comp = tier2["phase_compensated"]
    summary = tier2["phase_compensation_summary"]

    # Both channels should surface — uniform shift means uniform finding.
    assert "hip_flexion_r" in comp
    assert "knee_angle_r" in comp
    knee = comp["knee_angle_r"]
    # Positive lag = iOS leads desktop = exactly what we injected.
    assert knee["best_lag_samples"] == shift
    assert knee["rmse_at_best_lag"] < 0.1 * knee["original_rmse"]
    assert knee["r2_at_best_lag"] > 0.99
    # Aggregate must show uniform sign — the fingerprint of a filter-
    # topology mismatch as opposed to per-channel noise.
    assert summary["uniform_sign"] is True
    assert summary["median_lag_ms"] > 0


def test_compare_flags_unmatched_columns(tmp_path: Path) -> None:
    desk = tmp_path / "desktop"
    ios = tmp_path / "ios"
    _make_synthetic_export(desk, "trial",
                           extra_kinematics_col="lumbar_bending")  # desktop has it
    _make_synthetic_export(ios, "trial",
                           extra_kinematics_col="lumbar_lat")      # iOS uses other name

    report = compare_folders(desk, ios, "trial")
    unmatched = report.files["kinematics.mot"].unmatched
    assert "lumbar_bending" in unmatched["desktop_only"]
    assert "lumbar_lat" in unmatched["ios_only"]


def test_compare_flags_missing_files(tmp_path: Path) -> None:
    desk = tmp_path / "desktop"
    ios = tmp_path / "ios"
    _make_synthetic_export(desk, "trial")
    _make_synthetic_export(ios, "trial")
    # Drop activations from iOS to simulate a model with no muscles.
    (ios / "trial_activations.sto").unlink()
    (ios / "trial_muscle_forces.sto").unlink()

    report = compare_folders(desk, ios, "trial")
    assert "activations.sto" in report.files_missing["ios_only"]
    assert "muscle_forces.sto" in report.files_missing["ios_only"]
    # Kinematics must still be compared.
    assert "kinematics.mot" in report.files


# ---------------------------------------------------------------------------
# ios_export
# ---------------------------------------------------------------------------


def test_load_export_and_autodetect_stem(tmp_path: Path) -> None:
    folder = tmp_path / "OfflineExport-trial-ABC12345"
    _make_synthetic_export(folder, "trial")

    stem = autodetect_stem(folder)
    assert stem == "trial"

    bundle = load_export(folder, "trial")
    assert bundle.has_muscles()
    assert "kinematics.mot" in bundle.files_present()
    assert bundle.summary is not None
    assert bundle.summary["stem"] == "trial"


def test_load_trial_parses_subject_anthropometrics(tmp_path: Path) -> None:
    """`subject_mass_kg` and `subject_height_m` round-trip through the YAML
    loader as floats. Default to None when omitted so existing configs keep
    working."""
    from biomotion_desktop.trial import load_trial

    osim = tmp_path / "model.osim"
    trc = tmp_path / "motion.trc"
    osim.write_text("<dummy/>")  # load_trial only checks existence
    trc.write_text("dummy")

    cfg = tmp_path / "trial.yaml"
    cfg.write_text(
        "name: subj01\n"
        f"osim: {osim.name}\n"
        f"trc: {trc.name}\n"
        "subject_mass_kg: 72.5\n"
        "subject_height_m: 1.78\n"
    )
    t = load_trial(cfg)
    assert t.subject_mass_kg == pytest.approx(72.5)
    assert t.subject_height_m == pytest.approx(1.78)

    # Same file without the anthropometrics → both fields stay None.
    cfg.write_text(f"name: subj02\nosim: {osim.name}\ntrc: {trc.name}\n")
    t = load_trial(cfg)
    assert t.subject_mass_kg is None
    assert t.subject_height_m is None


def test_load_export_raises_when_empty(tmp_path: Path) -> None:
    empty = tmp_path / "empty"
    empty.mkdir()
    with pytest.raises(FileNotFoundError):
        load_export(empty, "trial")
