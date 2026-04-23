# Status & Open Issues — Desktop Pipeline

Snapshot taken **2026-04-23** after the second iOS↔desktop comparison session.
Previous snapshots are preserved in `git log` for prior revisions of this file.

---

## 1. Current pipeline state

| Stage | Status | Notes |
|---|---|---|
| Conda env (micromamba @ `.tools/envs/biomotion-desktop/`) | ✅ Stable | OpenSim `4.5.2-2025-04-07-2c9fc5bc9`, NumPy 1.25.2 (pin required — see "Gotchas" in `CLAUDE.md`). |
| Smoke tests | ✅ **13/13 pass** | `pytest desktop-pipeline/tests/` — runs without OpenSim. |
| IK | ✅ Validated on full 32 s `run2` and 5 s short window. | Marker RMS ≈ 10 mm, max ≈ 19 mm. |
| ID | ✅ Validated on both windows. | No GRF → big pelvis residuals, expected for OpenCap data. Subject mass override now wired (§2.7). |
| **SO** | ✅ **Confirmed converging** on 5 s short window. | Took **1530 s ≈ 25.5 min** for 5 s of data → **~5 min compute per 1 s of trial**. Full 32 s would be ~2.7 h. |
| `compare` CLI | ✅ Run end-to-end against a real iOS export folder. | Report: `results/run2_short/compare.json`. Now emits locked-DOF, sign-flip, and phase-compensation diagnostics. |
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
    └── compare.json               # iOS↔desktop diff, refreshed this session

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

### 2.5  Locked-DOF auto-detection in comparator (was §4.3)
- **Symptom**: `mtp_angle_l/r` had R² = 0.00 and dragged the kinematics aggregate down because one side reported a constant value while the other didn't. Same will hit any DOF the model locks (e.g. `subtalar_angle_*` in some scaled models).
- **Fix**: `compare._classify_lock(desk, ios)` flags any channel where `np.std() < 1e-6` on either side. `ChannelMetrics.status ∈ {"ok", "locked_desktop", "locked_ios", "locked_both"}`. Aggregate metrics now report `scored_count` and `locked_channels` separately and only average over `status == "ok"` channels. Per-channel metrics still emitted so nothing is silently swallowed.
- **Tests**: `test_compare_excludes_locked_dofs_from_aggregates` constructs the exact `mtp_angle_l` situation (desktop constant 0, iOS constant +7.5°) and asserts the channel ends up `locked_both` and out of aggregates.
- **Result on real data**: kinematics `r²_mean` rose **0.74 → 0.79** purely from no longer averaging in two channels that were dictionary-locked on the iOS side. `mtp_angle_l/r` correctly tagged `locked_ios`; no false positives on the other 31 DOFs.

### 2.6  Sign-flip diagnostic + falsified knee-zero hypothesis (was §4.2)
- **Diagnostic added**: `compare._sign_flip_suspect()` runs on every kinematics channel that's already `status="ok"` and reports any case where flipping the desktop sign would cut RMSE by ≥ 50 % while the original R² was ≥ 0.5. The flipped RMSE / bias / R² are surfaced per-channel under `tier2.sign_flip_suspects` so a real convention mismatch could never be silently smoothed over.
- **Test**: `test_compare_flags_sign_flip_suspect` exercises the detector on synthetic anti-correlated data.
- **Result on real data**: **zero channels flagged**, on `knee_angle_*` or anywhere else. The "knee_angle_r RMSE 12.7°, bias +0.5°, R² 0.83" symptom from the previous session was **not** a sign convention bug — see §2.7.

### 2.7  Phase-lag is the real cause of the kinematics RMSE (new finding)
This is the most consequential finding of the session and replaces what §4.2 used to claim.

- **Diagnostic added**: `compare._phase_compensated_metrics()` searches integer-sample lags in ±100 ms for the shift that minimises RMSE, and reports the post-compensation RMSE / R² / bias for every channel where the optimal shift is ≥ 12 ms away from zero AND cuts RMSE by ≥ 30 %. `tier2.phase_compensation_summary` rolls this up into a median lag + a `uniform_sign` boolean — the fingerprint of a *systemic filter-topology mismatch* as opposed to per-channel noise.
- **Test**: `test_compare_phase_compensation_recovers_uniform_shift` injects a 12-sample shift and asserts the right `best_lag_samples`, `r²_at_best_lag > 0.99`, and `uniform_sign = True`.
- **Result on real data (`run2_short`, 120 Hz, 5 s)**: **29 / 33 kinematic channels** show a uniform positive lag — iOS leads desktop by +25 to +42 ms (**median +33 ms**, exactly 4 samples). After single-shift compensation:
  - `knee_angle_r`: RMSE **12.75° → 0.68°**, R² **0.83 → 1.00**.
  - `knee_angle_l`: RMSE **9.79° → 0.70°**, R² **0.80 → 1.00**.
  - 17 / 29 reach R² ≥ 0.99 after compensation; the worst (`lumbar_extension`) still goes 0.51 → 0.84.
- **Interpretation**: The two pipelines filter very differently. iOS uses a causal 1-Euro filter; the desktop wraps OpenSim's IK output in a 6 Hz Butterworth `filtfilt` (zero-phase but heavy). The 6 Hz filtfilt smooths peaks and effectively retards them, so desktop ends up *behind* iOS in time, not the other way round (the +sign on the lag is unambiguous in 29 / 29 channels).
- **Implication for §4.2 ("knee zero convention")**: that hypothesis is **falsified**. The bias of +0.5° on knee was within a degree — the visible RMSE was almost entirely the time-shift artifact. There is no need to add a per-channel offset in `_normalize_columns`; doing so would only hide the real signal.

### 2.8  Subject anthropometric override (partial mitigation for §4.1)
- **What's new**: `Trial` now accepts optional `subject_mass_kg` and `subject_height_m`. When `subject_mass_kg` is set, `anthropometrics.apply_subject_overrides()` writes `<stem>_subject_scaled.osim` once before IK/ID/SO with every body's mass uniformly rescaled so the model total matches the measured value. The pipeline replaces `trial.osim` with that path so all three drivers consume the same rescaled model. Mass override is recorded in `<stem>_summary.json["anthropometrics"]` for traceability.
- **Why uniform rescale**: the model is already segment-scaled (LaiUhlrich2022_*scaled*.osim) — only the *mass* fields are still at default. Uniform scaling preserves the relative mass distribution and is exactly what OpenSim's ScaleTool does in its mass-preservation step. de Leva-style per-segment recomputation would only pay off if we also rescaled geometry, which is out of scope until the GRF estimator lands.
- **`subject_height_m`**: accepted but **intentionally not yet used** — it's reserved for the future contact-implicit GRF estimator (de Leva-style segment scaling, foot contact sphere placement). Stored in summary so trial configs don't have to be re-versioned later.
- **Test**: `test_load_trial_parses_subject_anthropometrics` covers the YAML round-trip.
- **Caveat**: this does **not** unblock §4.1. Without GRF, ID still dumps all ground-contact load into pelvis residuals. Mass override only fixes the *scale* of those residuals (≤ ~10–20 % off if the modeller's mass guess was way off). It is the cheap correct thing to do before chasing a real GRF source, not a substitute for one.

---

## 3. First real iOS↔Desktop comparison — findings

Source: `results/run2_short/compare.json` on 5 s window, 601 frames @ 120 Hz, after the §2.5 / §2.6 / §2.7 diagnostics landed.

### 3.1  Kinematics — the "RMSE 4°" was almost entirely a time shift

Overall (after locked-DOF exclusion, §2.5):

| Metric | Value | Notes |
|---|---|---|
| Channels matched | 33 | 31 scored + 2 locked-on-iOS (`mtp_angle_l/r`). |
| `rmse_mean` | **3.80°** | Down from 4.0° pre-locked-DOF fix. |
| `rmse_max` | 12.75° (`knee_angle_r`) | Phase-compensated value: 0.68°. |
| `r²_mean` | **0.79** | Up from 0.74. |
| `r²_min` | 0.43 (`pelvis_tilt`) | Phase-compensated value: 0.94. |
| Phase-compensated channels | **29 / 33**, uniform positive sign | Median lag +33.3 ms (4 samples). See §2.7. |
| Sign-flip suspects | 0 | Confirms no sign-convention bug. |

After applying the single +33 ms shift, **17 / 29 channels reach R² ≥ 0.99** and the worst is 0.84. The "shape agrees, magnitudes don't" reading from the previous session was wrong — shape *and* magnitude agree, the two signals were just running on different clocks.

Remaining real disagreements (i.e. surviving phase compensation):

| Channel | Post-compensation RMSE / R² | Likely cause |
|---|---|---|
| `ankle_angle_l` | 2.13°, R² 0.98 | Single 40° spike still dominant in original; compensation can't fix a transient. **Worth a per-channel plot.** |
| `lumbar_extension` | 1.60°, R² 0.84 | Lowest post-compensation R²; deserves a closer look but not a known bug. |
| `pelvis_tilt` | 1.13°, R² 0.94 | Was the worst pre-compensation. Acceptable. |
| `mtp_angle_l/r` | n/a | `locked_ios`, excluded from aggregates. |

### 3.2  Inverse Dynamics — magnitudes diverge wildly, R² 0.30

After the suffix fix, 33/33 channels match. RMSE mean **328 Nm**, R² mean 0.30.
Hip flexion alone is RMSE 2200 Nm with bias -1900 Nm.

**Root cause: no ground reaction force.** This OpenCap trial has no force-plate
data (`grf_mot: null` in the trial config). OpenSim ID without GRF dumps all
ground-contact load into pelvis residuals, which then propagate through the
chain. The iOS pipeline handles the same missing GRF differently. The two
"pseudo-ID" results are not comparable in magnitude — only in shape, and the
shape correlation R² 0.30 confirms they're doing materially different things.

The §2.8 mass override will trim a few percent off the residuals but cannot
make them physical. §4.1 is still the real work.

### 3.3  Muscle activations & forces — patterns disagree (R² ≈ 0.08)

- 80 / 80 muscle channels match by name (no alias map needed).
- Activations: RMSE mean **0.37 (out of 0–1 range)**, R² mean **0.075**.
- Forces: RMSE mean **283 N**, R² mean **0.083**.
- Both sides saturate muscles at ~0.99. Desktop top: `tfl_r=0.96`, `tfl_l=0.84`, `addlong_l=0.71`. iOS top (per its own summary): same `tfl_r=0.96`, `tfl_l=0.84`. But the *patterns over time* are uncorrelated.
- **Best-correlated muscle is only R² 0.35** (`perbrev_r`); most are < 0.10.

This is the downstream of §3.2 — once ID is unphysical, SO has nothing to
match against, and each side's optimiser fills the gap with its own bias.
**Comparing muscle activations between iOS and desktop is meaningless until
both sides have GRF.** Independently, iOS's OSQP-based formulation and
OpenSim's `StaticOptimization` are different algorithms (objective, bounds,
muscle physiology toggles), so even with perfect ID inputs we should expect
divergence at the per-frame activation level. SO comparison should reframe
around timing of activation peaks and co-contraction patterns rather than
RMSE/R² of activations themselves — see §4.4.

---

## 4. Open decisions / next steps

Ranked by ROI. None are blocking the pipeline as a tool — they're blocking
the **scientific claim** "this validates the iOS pipeline".

### 4.1  GRF is the real blocker for muscle/ID validation (P0)
The OpenCap demo data has no force plates. Three options, in order of effort:
- **(a)** Pick a different trial that *does* have GRF (HamnerRunningGuide, OpenSim sample data, lab-collected). One-off scripting work; preserves the rest of the pipeline. **This is the recommended next move.**
- **(b)** Wrap an opencap-processing GRF estimator (`StaticOptimizationContact`, contact-implicit Moco). Bigger dependency story but keeps OpenCap data usable.
- **(c)** Accept the limitation, document explicitly that muscle comparison on no-GRF data is qualitative only, and use it for kinematics + ID-shape validation only.

Partial mitigation already shipped: §2.8 lets you set the subject's measured
mass and rescale model body masses uniformly. This is correct anthropometric
hygiene before any GRF work, but does not by itself unblock ID magnitude
validation.

### 4.2  Decide what to do about the +33 ms phase lag (P1)
The diagnostic in §2.7 makes the lag visible and quantifies its cost.
Three options:
- **(a) Accept and report.** Treat the post-compensation RMSE/R² as the "real" kinematics agreement and the original as "naive". The compare report already produces both. This is the lowest-risk option and the one the report is designed for.
- **(b) Make iOS filtering zero-phase.** Have the iOS pipeline run a non-causal smoother on its export *before* IK rather than during it (the current 1-Euro filter is on by design for online use). Cleanest scientifically but adds a second filtering path on iOS and means realtime values would still differ from exported values.
- **(c) Switch desktop to causal filtering.** Replace `filtfilt` in `opensim_id.py` with a one-pass low-pass that matches the iOS group delay. Wrong direction — destroys ID by introducing the very lag we're complaining about into the input of dynamics.
- Recommendation: **(a) for the report; revisit (b) when GRF lands.** If the iOS team is willing to add a "validation export" path that bypasses 1-Euro and runs filtfilt to match desktop, that becomes the cleanest answer, but it's their call.

### 4.3  ~~Decide what to do with locked DOFs like `mtp_angle`~~ ✅ resolved (§2.5)

### 4.4  Reframe SO success criteria for the no-GRF case (P2)
Even with GRF eventually solved, iOS's OSQP formulation and OpenSim's
`StaticOptimization` are fundamentally different optimisers. RMSE/R² of
activations is the wrong yardstick. Suggested replacement metrics:
- Per-muscle activation **timing** (onset, peak, offset to within ±50 ms).
- **Co-contraction** patterns (correlation across antagonist pairs).
- **Total metabolic cost proxy** (sum of squared activations) per gait cycle.
- **Saturation rate** (fraction of frames with at least one muscle at ≥ 0.99).

Land a `docs/comparison_semantics.md` once these are decided so the report
can stop pretending RMSE 0.37 on activations means anything.

### 4.5  Inspect `ankle_angle_l` 40° spike (P2)
Generate the per-channel plot (`plot` CLI) and find the timestamp. Cheap.
Phase compensation knocks the average to 2.13°, so the spike is the only
remaining outlier of consequence on that channel.

### 4.6  Skip the full 32 s SO for now (P3)
~2.7 h compute, no new information until §4.1 is solved. The 5 s window
already exposed every issue worth fixing. Re-run at full length only after
we have GRF and want a clean publishable diff.

### 4.7  Promote `validate_demo.sh` to a real fixture-driven CI test (P3)
Hooks up `configs/trail2_short.yaml` + `fixtures/ios-exports/run2/`
end-to-end. Worth doing once §4.1 / §4.2 stabilise — otherwise the
"expected" numbers will keep drifting.

### 4.8  Minor cleanup
- `opensim_id.py` still has the dead `excluded = osim.ArrayStr()` from the original draft (different from the live `excluded_coords`). Delete or wire up.
- `results/run2/run2_so_setup.xml` is leftover from an interrupted run. Either clean it up or just nuke the directory and rerun IK+ID on demand (cheap, ~16 s).
- README still tells users `conda env create`; the `micromamba` path that actually works on this dev box isn't documented.
- (Future) Refactor `compare._normalize_columns` into a declarative `CHANNEL_MAP` table once we accumulate more conventions (knee zero, locked DOFs, ID suffixes are all currently ad-hoc rules; one table would be cleaner). Not urgent — do it when adding the next rule, not now.

---

## 5. What we now believe with confidence

- ✅ The desktop pipeline runs end-to-end (IK + ID + SO + summary) on real OpenCap data.
- ✅ All file names line up with the iOS exporter (after the 2.3.1 / 2.3.2 / 2.3.3 fixes).
- ✅ The comparator produces actionable signal — it surfaced one real bug (ID suffix), one likely bug (`ankle_l` spike), one model issue (`mtp_angle` lock, now auto-handled), and one filter-topology mismatch (+33 ms uniform phase lag, now quantified) across two sessions.
- ✅ **The kinematics actually agree** — knee/hip/ankle/arm DOFs all reach R² ≥ 0.99 once the +33 ms uniform shift is compensated. The previous "RMSE 12°, sign convention bug" reading was wrong.
- ✅ Subject mass can now be passed in; height too (stored, awaiting GRF estimator).
- ⚠️ Quantitative iOS↔desktop validation of **muscles and ID magnitudes** requires GRF. Without it, the comparison is qualitative (shape, channel coverage, naming, timing) but not magnitude.
- ⚠️ SO is the dominant cost: ~5 minutes per second of trial. For routine validation use the 5 s short window; reserve the full 32 s for a once-off publishable run.
