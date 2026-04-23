# Status & Open Issues — Desktop Pipeline

Snapshot taken **2026-04-23** at the end of the first end-to-end iOS↔desktop
comparison session. Previous snapshot (initial scaffold + IK/ID validation)
is preserved in `git log` for the prior revision of this file.

---

## 1. Current pipeline state

| Stage | Status | Notes |
|---|---|---|
| Conda env (micromamba @ `.tools/envs/biomotion-desktop/`) | ✅ Stable | OpenSim `4.5.2-2025-04-07-2c9fc5bc9`, NumPy 1.25.2 (pin required — see "Gotchas" in `CLAUDE.md`). |
| Smoke tests | ✅ 9/9 pass | `pytest desktop-pipeline/tests/` — runs without OpenSim. |
| IK | ✅ Validated on full 32 s `run2` and 5 s short window. | Marker RMS ≈ 10 mm, max ≈ 19 mm. |
| ID | ✅ Validated on both windows. | No GRF → big pelvis residuals, expected for OpenCap data. |
| **SO** | ✅ **Confirmed converging** on 5 s short window. | Took **1530 s ≈ 25.5 min** for 5 s of data → **~5 min compute per 1 s of trial**. Full 32 s would be ~2.7 h. |
| `compare` CLI | ✅ Run end-to-end against a real iOS export folder. | Report: `results/run2_short/compare.json`. |
| `plot` CLI | ⚠️ Not yet exercised on real data. | Code path identical to compare; expected to work. |

Artefacts on disk after this session:

```
desktop-pipeline/
├── results/run2/                  # IK + ID only, full 32 s (SO never re-run)
│   ├── run2_kinematics.mot
│   ├── run2_inverse_dynamics.sto
│   ├── _ik_marker_errors.sto
│   └── run2_so_setup.xml          # leftover, SO not finished here
└── results/run2_short/            # full IK + ID + SO, 5 s window
    ├── run2_short_kinematics.mot
    ├── run2_short_inverse_dynamics.sto
    ├── run2_short_activations.sto
    ├── run2_short_muscle_forces.sto
    ├── run2_short_summary.json
    ├── _ik_marker_errors.sto
    └── compare.json               # iOS↔desktop diff, this session

fixtures/ios-exports/run2/         # checked-in iOS export, UUID-stemmed
└── 46A017D9-…_{kinematics.mot, inverse_dynamics.sto, activations.sto,
                 muscle_forces.sto, summary.json}
```

---

## 2. Resolved this session

### 2.1  Static Optimization actually completes (was P0)
- `opensim_so.py` `AnalyzeTool(setup_xml)` reload pattern works.
- Empirical timing on 5 s × 80 muscles (Lai/Uhlrich, no GRF): **~5 min compute per 1 s of motion**. The earlier "interrupted after 5–8 min" runs were impatience, not a hang.
- 5 s short config (`configs/trail2_short.yaml`) updated from 1 s window to **1.0 s → 6.0 s** so the comparison spans ~10 running steps.

### 2.2  ID column-name drift between iOS and desktop (new bug, fixed)
- **Symptom**: zero ID channels matched in the first compare run.
- **Cause**: OpenSim's `InverseDynamicsTool` tags rotational DOFs `_moment` and translational DOFs `_force` (`ankle_angle_l_moment`, `pelvis_tx_force`). The iOS `OfflineAnalysisExporter` writes plain DOF names (`ankle_angle_l`, `pelvis_tx`).
- **Fix**: `compare._normalize_columns(file_key, side, columns)` strips those suffixes from desktop-side ID columns. Plot path uses the same normaliser. Test: `tests/test_smoke.py::test_compare_strips_opensim_id_suffixes`.
- **Result**: ID went from 0/35 channels matched to **33/33** (the two unmatched on desktop are `knee_angle_*_beta` patella tracking coords that iOS doesn't export — by design).

### 2.3  iOS-export comparator supports a separate UUID stem
- **Symptom**: iOS app names every export file after a per-run UUID, e.g. `46A017D9-568C-409D-B384-22DDFFE0592A_kinematics.mot`. The desktop pipeline uses the trial name (`run2_short_…`).
- **Fix**: `compare_folders(...)` and `plot_comparison(...)` accept an `ios_stem` kwarg; CLI gets `--ios-stem` (autodetected from `*_summary.json` in the iOS folder if omitted).

### 2.4  iOS exports are now version-controlled fixtures
- Path convention: `desktop-pipeline/fixtures/ios-exports/<trial>/`. Documented in `fixtures/ios-exports/README.md`.
- `desktop-pipeline/.gitignore` now whitelists `fixtures/` and excludes `.DS_Store` so future drops can be committed without surprises.

---

## 3. First real iOS↔Desktop comparison — findings

Source: `results/run2_short/compare.json` on 5 s window, 601 frames @ 120 Hz.

### 3.1  Kinematics — mostly OK, two real issues + one expected gap

Overall: 33 channels matched, **RMSE mean 4.0°, R² mean 0.74**. Worst columns:

| Channel | RMSE (°) | max (°) | R² | bias (°) | Diagnosis |
|---|---|---|---|---|---|
| `knee_angle_r/l` | 12.7 / 9.8 | 24 / 21 | **0.83 / 0.80** | +0.5 | Shape tracks well, **systematic offset** — most likely a Nimble↔OpenSim knee zero-position convention mismatch. **Needs convention check.** |
| `mtp_angle_l/r` | 7.5 | 8.2 | **0.00** | **+7.5** | One side reports a near-constant value, the other reports ~0. OpenSim locks the metatarsophalangeal joint by default; iOS reports something nonzero. **Decide whether to exclude locked DOFs or fix the iOS export.** |
| `ankle_angle_l` | 7.4 | **39.7** | 0.74 | +1.2 | Single 40° spike against generally good tracking. **Needs a plot to localise** — could be a 1-euro filter glitch or gimbal-lock instant on the iOS side. |
| `hip_flexion_r/l` | 5.9 / 6.0 | 12.6 / 12.7 | 0.86 / 0.87 | -0.6 | Healthy. |
| `arm_flex_r` | 5.7 | 13.5 | **0.90** | 0.0 | Healthy. |

Unmatched (expected): `knee_angle_l_beta`, `knee_angle_r_beta` (Lai/Uhlrich patella aux DOFs, iOS doesn't expose them).

### 3.2  Inverse Dynamics — magnitudes diverge wildly, R² 0.30

After the suffix fix, 33/33 channels match. RMSE mean **328 Nm**, R² mean 0.30.
Hip flexion alone is RMSE 2200 Nm with bias -1900 Nm.

**Root cause: no ground reaction force.** This OpenCap trial has no force-plate
data (`grf_mot: null` in the trial config). OpenSim ID without GRF dumps all
ground-contact load into pelvis residuals, which then propagate through the
chain. The iOS pipeline handles the same missing GRF differently. The two
"pseudo-ID" results are not comparable in magnitude — only in shape, and the
shape correlation R² 0.30 confirms they're doing materially different things.

### 3.3  Muscle activations & forces — patterns disagree (R² ≈ 0.08)

- 80 / 80 muscle channels match by name (no alias map needed).
- Activations: RMSE mean **0.37 (out of 0–1 range)**, R² mean **0.075**.
- Forces: RMSE mean **283 N**, R² mean **0.083**.
- Both sides saturate muscles at ~0.99. Desktop top: `tfl_r=0.96`, `tfl_l=0.84`, `addlong_l=0.71`. iOS top (per its own summary): same `tfl_r=0.96`, `tfl_l=0.84`. But the *patterns over time* are uncorrelated.
- **Best-correlated muscle is only R² 0.35** (`perbrev_r`); most are < 0.10.

This is the downstream of §3.2 — once ID is unphysical, SO has nothing to
match against, and each side's optimiser fills the gap with its own bias.
**Comparing muscle activations between iOS and desktop is meaningless until
both sides have GRF.**

---

## 4. Open decisions / next steps

Ranked by ROI. None are blocking the pipeline as a tool — they're blocking
the **scientific claim** "this validates the iOS pipeline".

### 4.1  GRF is the real blocker for muscle/ID validation (P0)
The OpenCap demo data has no force plates. Three options, in order of effort:
- **(a)** Pick a different trial that *does* have GRF (HamnerRunningGuide, OpenSim sample data, lab-collected). One-off scripting work; preserves the rest of the pipeline.
- **(b)** Wrap an opencap-processing GRF estimator (`StaticOptimizationContact`, contact-implicit Moco). Bigger dependency story but keeps OpenCap data usable.
- **(c)** Accept the limitation, document explicitly that muscle comparison on no-GRF data is qualitative only, and use it for kinematics + ID-shape validation only.

### 4.2  Knee-angle convention check (P1, ~30 min)
Either the Nimble model or the OpenSim model has a different knee zero. Verify by inspecting both `.osim` files' `<Coordinate name="knee_angle_r">` default values and Nimble's joint definition. If different, add a per-channel offset in `compare._normalize_columns` (NOT silent — record the offset in `compare.json`).

### 4.3  Decide what to do with locked DOFs like `mtp_angle` (P1, ~15 min)
Either:
- Have iOS exporter skip locked DOFs (bigger, requires Swift edit), OR
- Have the comparator drop channels where the desktop side is constant within ε. Cleaner separation: the comparator already knows which side is constant.

### 4.4  Inspect `ankle_angle_l` 40° spike (P2)
Generate the per-channel plot (`plot` CLI) and find the timestamp. Cheap.

### 4.5  Skip the full 32 s SO for now (P3)
~2.7 h compute, no new information until §4.1 is solved. The 5 s window already exposed every issue worth fixing. Re-run at full length only after we have GRF and want a clean publishable diff.

### 4.6  Promote `validate_demo.sh` to a real fixture-driven CI test (P3)
Hooks up `configs/trail2_short.yaml` + `fixtures/ios-exports/run2/` end-to-end. Worth doing once §4.1–4.3 stabilise — otherwise the "expected" numbers will keep drifting.

### 4.7  Minor cleanup
- `opensim_id.py` still has the dead `excluded = osim.ArrayStr()` from the original draft (different from the live `excluded_coords`). Delete or wire up.
- `results/run2/run2_so_setup.xml` is leftover from an interrupted run. Either clean it up or just nuke the directory and rerun IK+ID on demand (cheap, ~16 s).
- README still tells users `conda env create`; the `micromamba` path that actually works on this dev box isn't documented.

---

## 5. What we now believe with confidence

- ✅ The desktop pipeline runs end-to-end (IK + ID + SO + summary) on real OpenCap data.
- ✅ All file names line up with the iOS exporter (after the 2.3.1 / 2.3.2 / 2.3.3 fixes).
- ✅ The comparator produces actionable signal — it surfaced one real bug (ID suffix), one likely bug (`ankle_l` spike), and two model-convention issues (`knee_angle` zero, `mtp_angle` lock) in the very first run.
- ⚠️ Quantitative iOS↔desktop validation of **muscles and ID magnitudes** requires GRF. Without it, the comparison is qualitative (shape, channel coverage, naming) but not magnitude.
- ⚠️ SO is the dominant cost: ~5 minutes per second of trial. For routine validation use the 5 s short window; reserve the full 32 s for a once-off publishable run.
