# BioMotion Desktop Validation Pipeline

Desktop ground-truth pipeline for the BioMotion iOS app. Runs **OpenSim IK + ID +
Static Optimization** on the same OpenCap trial data the phone consumes, then
diffs the result against the iOS app's exported `.mot` / `.sto` / `.json` files.

> **Status**: scaffold complete, IK+ID validated on demo data, SO + iOS
> comparison still to verify end-to-end. See
> [`OPEN_ISSUES.md`](./OPEN_ISSUES.md) before picking this up next session.

> Open this folder (`desktop-pipeline/`) directly in Cursor as its own workspace
> when you only want to work on the validation side. The parent `BioMotionYeah/`
> repo holds the iOS app; this sub-project never touches Swift or Xcode.

## What it does

```
OpenCap trial (.osim + .trc)
        │
        ├──► OpenSim IK   ──►  *_kinematics.mot          (joint angles, deg)
        │
        ├──► OpenSim ID   ──►  *_inverse_dynamics.sto    (joint moments, Nm)
        │
        └──► OpenSim SO   ──►  *_activations.sto         (muscle activations)
                          └──► *_muscle_forces.sto       (muscle forces, N)

iOS export (same stems)
        │
        └──► compare ─────►  per-column RMSE / max-abs / R²  +  plots
```

The output file naming is **identical** to what
`BioMotion/Offline/OfflineAnalysisExporter.swift` writes, so the comparison
tool just loads matching pairs and computes per-DOF / per-muscle errors.

## Layout

```
desktop-pipeline/
├── src/biomotion_desktop/      Python package
│   ├── trial.py                Trial dataclass (osim, trc, optional grf, output dir)
│   ├── opensim_ik.py           InverseKinematicsTool driver
│   ├── opensim_id.py           InverseDynamicsTool driver
│   ├── opensim_so.py           StaticOptimization analysis driver
│   ├── pipeline.py             IK → ID → SO orchestration
│   ├── storage_io.py           .mot / .sto reader (no OpenSim required)
│   ├── ios_export.py           Load iOS app's exported folder
│   ├── compare.py              Column-aligned metrics (RMSE, max abs, R²)
│   ├── plot.py                 Matplotlib comparison plots (optional)
│   └── cli.py                  argparse entry points
├── scripts/                    Thin CLI wrappers
│   ├── run_pipeline.py
│   ├── compare_to_ios.py
│   └── plot_comparison.py
├── configs/                    Trial registry (YAML/JSON)
│   └── trail2.yaml
├── tests/                      pytest smoke tests
├── results/                    Pipeline output (gitignored)
├── environment.yml             Conda env (recommended — opensim wheel is conda-only)
├── requirements.txt            Pip-only deps (numpy/pandas/matplotlib/pyyaml)
├── pyproject.toml              Package metadata, console_scripts
├── CLAUDE.md                   LLM-facing context
└── README.md                   This file
```

## Setup

OpenSim's Python bindings are only reliably distributed through the
**`opensim-org` conda channel**. Use conda/mamba.

```bash
cd desktop-pipeline

# 1) Create the env (Python 3.11 + OpenSim 4.5)
conda env create -f environment.yml
conda activate biomotion-desktop

# 2) Install this package in editable mode
pip install -e .

# 3) Sanity check that OpenSim loaded
python -c "import opensim; print(opensim.GetVersion())"
```

If you must avoid conda, follow the official OpenSim source-build instructions
(<https://github.com/opensim-org/opensim-core>) and ensure `import opensim`
works inside any venv you create. The pure-Python bits (loader, comparator,
plots) work without OpenSim — only the `opensim_*.py` drivers and
`run_pipeline` need it.

## Quick start (uses bundled `trail2` demo data)

```bash
# 1) Run the desktop ground truth pipeline
python -m biomotion_desktop.cli run \
    --config configs/trail2.yaml \
    --output results/trail2_desktop

# Produces:
#   results/trail2_desktop/run2_kinematics.mot
#   results/trail2_desktop/run2_inverse_dynamics.sto
#   results/trail2_desktop/run2_activations.sto
#   results/trail2_desktop/run2_muscle_forces.sto
#   results/trail2_desktop/run2_summary.json

# 2) Diff against an iOS export folder shared from the phone
python -m biomotion_desktop.cli compare \
    --desktop results/trail2_desktop \
    --ios     /path/to/OfflineExport-run2-XXXXXXXX \
    --stem    run2 \
    --report  results/trail2_desktop/compare.json

# 3) Optional plots (requires matplotlib)
python -m biomotion_desktop.cli plot \
    --desktop results/trail2_desktop \
    --ios     /path/to/OfflineExport-run2-XXXXXXXX \
    --stem    run2 \
    --out     results/trail2_desktop/plots
```

## Adding a new trial

Drop a YAML file in `configs/`:

```yaml
name: my_trial
osim:    ../demodata/opencap-demodata/<dir>/Model_scaled.osim
trc:     ../demodata/opencap-demodata/<dir>/take.trc
# Optional — Static Optimization improves with GRF; OpenCap data usually has none.
grf_mot: null
grf_xml: null
# Optional — restrict the analyzed time window
start_time: null
end_time:   null
# Override default low-pass filter (Hz) applied to IK kinematics before ID.
lowpass_hz: 6.0
```

…then run with `--config configs/my_trial.yaml`.

## How the comparison works

For every shared file pair (`*_kinematics.mot`, `*_inverse_dynamics.sto`,
`*_activations.sto`, `*_muscle_forces.sto`):

1. Load both as time-indexed `pandas.DataFrame`s.
2. Resample the iOS series onto the desktop time grid via linear interpolation
   (the iOS app runs at the source-video framerate, but the analysis cadence
   may differ).
3. For each column present in **both** files, compute:
   - RMSE
   - Max absolute error
   - Pearson R²
   - 95th percentile absolute error
4. Aggregate per file (`overall_rmse`, etc.) and dump a JSON report.

Columns missing from one side are flagged in `unmatched_columns` so you can
spot DOF / muscle naming mismatches between the iOS Nimble model and the
OpenSim ground truth.

### Bilateral asymmetry diagnostic

Append `--asymmetry-report` to `compare` to surface per-pair left/right
asymmetry on the **iOS** activations file. Useful for catching contact-detection
or GRF-estimator bias that produces lopsided muscle drive even when the input
kinematics are bilaterally symmetric (e.g. an OpenCap `.mot`).

```bash
python -m biomotion_desktop.cli compare \
    --desktop results/trail2_desktop \
    --ios     /path/to/OfflineExport-run2-XXXXXXXX \
    --stem    run2 \
    --asymmetry-report \
    --asymmetry-threshold-pct 10
```

Output is appended after the existing tier metrics and looks like:

```
=== Bilateral Asymmetry Report (threshold ±10.0%) ===
pair        peak_l   peak_r  peak_diff  rms_diff  asym_idx_pct  status
tibant       0.452    0.481      0.034     0.018          +6.2  ok
gaslat       0.612    0.198      0.414     0.221         +102.3  FAIL
...
SUMMARY: FAIL (1 pair(s) exceed ±10.0%): gaslat
```

The asymmetry index is `100 * (peak_l - peak_r) / mean(peak_l, peak_r)`,
NaN when either side is all-zero. Process exits non-zero when **any** pair
exceeds the threshold. Without `--asymmetry-report`, stdout and exit code
stay byte-identical to the pre-flag behavior.

> **Known metric limitation: peak-only, saturation-blind.** When both sides
> saturate to `1.0` at *different* times, `peak_l = peak_r = 1.0` and the
> asymmetry index reads `+0.0%` even though the temporal series are wildly
> different. `rms_diff` in the same row exposes this (values of 0.3+ on
> "+0.0%" rows are the giveaway). A future change is expected to fold
> `rms_diff` (or a cross-correlation lag) into the PASS/FAIL gate; until
> then, **do not treat a "0 FAIL" summary as proof of bilateral symmetry**
> on its own.

A committed reference fixture lives at
`tests/fixtures/bilateral_grf_baseline/` — it is a deliberate
**negative-result snapshot** of the `fix-bilateral-grf-bias` change, kept
in git so the falsification evidence and the saturation-blindness exhibit
are reproducible. See its `README.md` for the run command. CI gating on
this fixture is intentionally **not yet wired** because the fixture
currently FAILs by design.

## Why this exists

The iOS pipeline trades accuracy for latency:

- ARKit gives only 91 surface joints (no `_study` markers).
- Nimble IK runs in ~1 ms with a much smaller marker set.
- OSQP muscle SO is solved per-frame with simplified constraints.

The desktop pipeline runs the **canonical OpenSim solvers** with the full
augmented OpenCap marker set on the same source motion, providing a defensible
ground-truth baseline. Per-DOF and per-muscle error reports tell you exactly
where the on-device approximations break down.
