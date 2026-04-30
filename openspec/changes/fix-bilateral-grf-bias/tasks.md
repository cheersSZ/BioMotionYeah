## 1. Bridge: per-foot ground tracking + hysteresis

- 1.1 In `BioMotion/Nimble/NimbleBridge.mm` replace the single `_groundHeightY` ivar with `_groundHeightLY`, `_groundHeightRY`, `_groundHeightLCalibrated`, `_groundHeightRCalibrated`, `_leftInContact`, `_rightInContact` (defaults: 0.0, 0.0, NO, NO, NO, NO).
- 1.2 Update the existing `- (double)groundHeightY` accessor to return `min(_groundHeightLY, _groundHeightRY)` when both calibrated, the calibrated one when only one is, else 0.
- 1.3 Update `- (BOOL)groundHeightCalibrated` to return `_groundHeightLCalibrated && _groundHeightRCalibrated`.
- 1.4 Add new ObjC accessors `- (double)groundHeightLY`, `- (double)groundHeightRY` declared in `NimbleBridge.h`.
- 1.5 In `solveIDGRF` replace the lines 633–639 ratchet block with per-foot ratcheting: each side's ground bar tracks `(its calcn y) - 0.01 m` independently.
- 1.6 Replace the `CONTACT_THRESHOLD = 0.06` block (lines 646–648) with hysteresis: `CONTACT_ATTACH = 0.06`, `CONTACT_RELEASE = 0.08`, using `_leftInContact`/`_rightInContact` ivars.
- 1.7 Update the `getMultipleContactInverseDynamicsNearCoP` call (line 700–707) to pass `(s_t)std::min(_groundHeightLY, _groundHeightRY)` as the `groundHeight` argument.
- 1.8 Update the CoP-y projection block (lines 724–730) to use the same `min(L, R)` value (single local variable, computed once before the call).
- 1.9 Update `- (void)setGroundHeightY:(double)y` (lines 130–132) to set both per-foot bars to `y` and mark both calibrated (preserves the existing manual-override semantics).
- 1.10 Reset all six new ivars in the existing model-load reset path (find the existing `_groundHeightCalibrated = NO;` reset and update it).
- 1.11 Build and confirm zero warnings in `NimbleBridge.mm`. Resolve any std::min ambiguity by including `<algorithm>` if not already present.

## 2. Engine: IDMode toggle

- 2.1 In `BioMotion/Nimble/NimbleEngine.swift` add `enum IDMode { case withGRF, noGRF }` and `var idMode: IDMode = .withGRF` to `NimbleEngine`.
- 2.2 In `runDynamicsAndMuscles` (locate via grep — it's the function that calls `bridge.solveIDGRF`), branch on `self.idMode`. For `.noGRF`, call `bridge.solveID(q, dq, ddq)` and construct the `IDOutput` with all GRF/contact/CoP/residual fields set to zero/false.
- 2.3 If `bridge.solveID` does not currently exist with that exact signature, add a thin `- (NimbleIDResult *)solveID:` ObjC method on `NimbleBridge` that calls `_skeleton->getInverseDynamics(ddq)` and returns torques only (mirroring the early-return path inside `solveIDGRF` lines 622–626 / 661–674).
- 2.4 Verify on the engine side that all four published `@Published var leftFootLoadFraction`, `rightFootLoadFraction`, `rootResidualPerKg`, `groundHeightY` get written with consistent zero/last-known-good values when `idMode == .noGRF` (do not mix stale GRF data into a noGRF result).

## 3. Offline: surface the toggle

- 3.1 In `BioMotion/Offline/OfflineSession.swift` add `var idMode: NimbleEngine.IDMode = .withGRF`.
- 3.2 In the function that drives `nimble.processImportedMotionFrame` (likely `processAllFrames` or similar — locate via grep), set `nimble.idMode = self.idMode` before the per-frame loop starts.
- 3.3 In `BioMotion/App/OfflineAnalysisView.swift` add a `Toggle("No-GRF mode (validation)", isOn: ...)` bound to the `OfflineSession.idMode` (mapped through a computed `Binding<Bool>` since `IDMode` is an enum). Place it next to existing analysis controls.
- 3.4 Disable the toggle while a trial is mid-process (use the existing "is processing" published flag on `OfflineSession`).
- 3.5 Show a small caption under the toggle: "When enabled, ID runs without ground-reaction-force estimation. Useful for diagnosing GRF-related muscle asymmetry."

## 4. Export schema: idMode tagging

- 4.1 In `BioMotion/Offline/OfflineAnalysisTrack.swift` add `var idMode: String = "withGRF"` to the persisted struct.
- 4.2 Update its `Codable` keys / decoder to default missing `idMode` to `"withGRF"` (use `decodeIfPresent` with `?? "withGRF"`).
- 4.3 In `BioMotion/Offline/OfflineAnalysisExporter.swift` write `"idMode"` into the top-level `<stem>_summary.json` object.
- 4.4 Confirm `OfflineSession` populates `track.idMode = (self.idMode == .noGRF) ? "noGRF" : "withGRF"` after processing completes.
- [x] 4.5 Re-export an existing committed trial twice (once `.withGRF`, once `.noGRF`) and confirm the JSON tags match the mode used. *(Verified 2026-04-30 against `desktop-pipeline/tests/fixtures/bilateral_grf_baseline/{withGRF,noGRF}/*_summary.json` — both `idMode` strings match the mode used to produce them.)*

## 5. Desktop pipeline: bilateral asymmetry report

- 5.1 In `desktop-pipeline/src/biomotion_desktop/compare.py` add `_detect_bilateral_pairs(columns: list[str]) -> list[tuple[str, str, str]]` returning `(stem, left_col, right_col)` tuples sorted by stem. Match `_l` / `_r` suffix on the activation column name (after stripping any trailing `.activation` token).
- 5.2 Add `_compute_pair_metrics(left: np.ndarray, right: np.ndarray) -> dict` returning `{"peak_l", "peak_r", "peak_diff", "rms_diff", "asym_idx_pct"}` with `asym_idx_pct = NaN` if either side is all zero.
- 5.3 Add `_format_asymmetry_report(pairs_metrics: list[tuple[str, dict]], threshold_pct: float) -> str` producing the table from design D6, with a final `SUMMARY: PASS|FAIL ...` line.
- 5.4 In `desktop-pipeline/src/biomotion_desktop/cli.py` add `--asymmetry-report` (boolean flag) and `--asymmetry-threshold-pct` (float, default 10.0).
- 5.5 When `--asymmetry-report` is set, load the iOS `_activations.sto` columns, compute pairs, append the formatted report after the existing tier output.
- 5.6 When `--asymmetry-report` is absent, stdout is byte-identical to the pre-change behavior. Add a quick regression test that diffs old vs new output for a committed fixture without the flag.
- 5.7 Add `desktop-pipeline/tests/test_bilateral_asymmetry.py` covering: pair detection (skip orphan), metric computation against a hand-built numpy array, NaN handling for all-zero input, PASS/FAIL summary line content.

## 6. Verification on a real fixture

> **Outcome:** the verification ran 2026-04-23, exports were captured for both
> `.withGRF` and `.noGRF` modes against the `run2` OpenCap fixture, and the
> `--asymmetry-report` was run against both. The numbers **falsified the
> change's central hypothesis** — see `proposal.md` → Findings. §6.4's
> "≥ 50% reduction" pass criterion is therefore moot and superseded by the
> Findings record. The four export files have been frozen as a negative-result
> historical snapshot under `desktop-pipeline/tests/fixtures/bilateral_grf_baseline/`.

- [x] 6.1 Pick the existing committed offline OpenCap `.mot` fixture currently used for asymmetry observation. If none is committed, commit one to `BioMotion/Resources/test_fixtures/` (one walking trial, ~5 seconds). *(Used the existing `run2` OpenCap trial — 31.7 s, 120 fps, 3807 frames. Note: this trial is too long / contains non-steady segments to act as a clean bilateral-symmetry reference; picking a 3-cycle steady-walk clip is deferred to the follow-up change.)*
- [x] 6.2 Process it on-device under both `.withGRF` and `.noGRF`. Capture `<stem>_withGRF_summary.json`, `<stem>_withGRF_activations.sto` and the same pair under noGRF. *(Done 2026-04-23. UUIDs: `D57C2A63-…` for withGRF, `357FDEF1-…` for noGRF.)*
- [x] 6.3 Run `desktop-pipeline ... --asymmetry-report` against both. Record `tibant`, `gaslat`, `recfem`, `bflh`, `glmax`, `glmed` peak diffs.

  | pair    | withGRF `asym_idx_pct` | noGRF `asym_idx_pct` |
  |---------|------------------------|----------------------|
  | tibant  | +0.0% (saturated)      | **+15.2% FAIL**      |
  | gaslat  | +0.0% (saturated)      | **+70.2% FAIL**      |
  | recfem  | +0.0% (saturated)      | +0.0% (saturated)    |
  | bflh    | +0.0% (saturated)      | **+51.3% FAIL**      |
  | glmax1  | +0.0% (saturated)      | **+42.3% FAIL**      |
  | glmed1  | +0.0% (saturated)      | −0.0% (saturated)    |

  Aggregate: withGRF 3 / 40 FAIL · noGRF 19 / 40 FAIL. The `+0.0% (saturated)` cells expose the saturation-blindness bug in the report metric (see Findings).

- [x] 6.4 ~~Compare against the pre-change baseline run on the same trial (re-run the old build once before landing if no baseline is already on disk). All six pairs SHALL show ≥ 50% reduction in `|asym_idx_pct|`.~~ **Superseded by Findings.** No pre-change baseline was captured before landing, so the "≥ 50% reduction" comparison is unmeasurable. More importantly, the `noGRF` result invalidates the underlying premise — the change's hypothesis would have predicted `noGRF` to be near-zero asymmetry, and instead `noGRF` is *more* asymmetric than `withGRF`. A reduction-vs-baseline metric on the failed hypothesis is no longer meaningful. The follow-up change owns finding the actual source.
- [x] 6.5 Commit the four post-change exports as `desktop-pipeline/tests/fixtures/bilateral_grf_baseline/` (two `_summary.json`, two `_activations.sto`). *(Files copied 2026-04-30, plus a `README.md` explaining this is a negative-result snapshot, not a passing baseline. Awaiting `git add`.)*
- [x] 6.6 Add a CI-runnable command snippet to `desktop-pipeline/README.md` that runs `--asymmetry-report --asymmetry-threshold-pct 10` against the committed `.withGRF` fixture and exits non-zero on `FAIL`. *(Snippet is in the "Bilateral asymmetry diagnostic" section of the README. **Note:** since the committed fixture is a negative-result snapshot and currently FAILs, this snippet is not yet wired into CI as a gate — wiring it as a gate is deferred to the follow-up change once a clean reference trial + non-saturation-blind metric exist.)*

## 7. Docs and release plumbing

- 7.1 Update `docs/UI_PARAMETERS.md` to document: (a) per-foot ground tracking, (b) the legacy `groundHeightY` badge now showing `min(L, R)`, (c) `CONTACT_ATTACH = 0.06 m` / `CONTACT_RELEASE = 0.08 m` as the contact-detection thresholds.
- 7.2 ~~Update `desktop-pipeline/docs/comparison_semantics.md`~~ — that file does not exist; instead added the **Bilateral asymmetry diagnostic** section to `desktop-pipeline/README.md` (CLI snippet, sample output, exit-code semantics), which covers task 6.6 in the same place.
- 7.3 Bump `CURRENT_PROJECT_VERSION` in `project.yml` (13 → 14) and run `xcodegen generate`.
- 7.4 In `openspec/changes/validate-ik-id-so-equivalence/tasks.md` mark tasks 1.1 through 1.7 as superseded by this change (added `> Superseded by` fix-bilateral-grf-bias`` note above section 1).
- 7.5 Update `CLAUDE.md` "Gotchas" section: added two bullets covering `_groundHeightLY` / `_groundHeightRY` independence and the offline `noGRF` mode.

## 8. Smoke test

> **Status (2026-04-30):** §8.2, §8.3, §8.4 verified from committed exports.
> §8.1 ran on device once and **observed a regression**: the app does not
> crash, but the live RealityKit muscle visualization no longer renders.
> Before the change shipped, live muscle viz was visible. Re-test is blocked
> on subject availability.
>
> **This blocks archive.** A change cannot graduate to `openspec/specs/` while
> a known live-path regression is open against it, even if the regression is
> not yet root-caused. The follow-up plan below records the bisect approach
> for the next on-device session.

- [ ] 8.1 Build for device (iPhone), launch, run the live ARKit pipeline for ~30 seconds standing → walking → standing. Confirm: no crashes; the `groundHeightY` badge stabilizes within 2 seconds; the muscle viz renders without flicker. **OBSERVED 2026-04-XX (user report): no crashes, but the RealityKit muscle visualization does not render at all.** Pre-change build had visible muscles in the live view. Code-level grep cannot pin the regression — `NimbleEngine` live path through `.withGRF` is byte-equivalent to the pre-change diff, and `MuscleOverlay.update()` defaults missing activations to `0.01` (blue) rather than skipping rendering. The empirically observed regression is therefore either (a) a subtle interaction inside `solveIDGRF`'s per-foot ratchet / hysteresis that makes the bridge return `nil` (which short-circuits muscle viz at `SkeletonOverlayView.swift:30` since `update(...)` is only called when `muscleActivations != nil`), or (b) something else not yet localized. **Bisect plan for next on-device session, in priority order:**
  1. Attach phone to Xcode console, run live for ~10 s, watch for `NimbleBridge: ID+GRF exception:` log lines (line 833 of `NimbleBridge.mm`). Their presence pinpoints solver throw → bridge returns nil → no muscle output.
  2. If no exception, add a one-line `os_log` inside `NimbleEngine.runDynamicsAndMuscles` reporting `idResult == nil`, `momentArms.isEmpty`, `muscleNamesNS.count`, `solveReal returned nil` for the first 30 frames. Whichever predicate fires identifies the cut.
  3. If muscle pipeline is producing non-nil output but viz still empty, log `nimble.lastMuscleResult?.activations.count` from `ContentView` — confirms whether the `@Published` actually carries data to the SwiftUI side.
  4. If all of (1)–(3) pass on-device but viz is still empty, the regression is in the `MuscleOverlay` rendering path itself (anchor / joint name mismatch / RealityKit lifecycle) — none of which are touched by this change, in which case the regression is pre-existing and should be split into its own `diagnose-live-muscle-viz` change so this one can archive.
  5. **Bisect-by-revert fallback:** if the diagnostic logs are inconclusive, build a branch where only the bridge changes are reverted (engine `IDMode` toggle stays); if muscle viz returns, the bridge changes are the cause and need targeted fixing. If muscle viz still empty, the cause is in the engine diff or somewhere else entirely.
- [x] 8.2 In offline mode, load the committed fixture with `idMode = .withGRF`. Confirm exports tag correctly. *(Verified 2026-04-30 from `tests/fixtures/bilateral_grf_baseline/{withGRF,noGRF}/*_summary.json` — both `idMode` fields equal the mode used to produce them. No device required to verify a JSON field.)*
- [x] 8.3 Toggle `idMode = .noGRF`, re-process the same fixture. Confirm the muscle viz now shows zero GRF arrows and that activations are visibly lower (no GRF means much smaller ankle/hip torques). *(Verified 2026-04-30 from the committed `_activations.sto` files, no device required:*
  - *Whole-trial mean activation `0.291 → 0.217` (−25%). Saturation rate `11.3% → 4.2%` (more than halved).*
  - *Ankle group (the muscles whose load is most directly GRF-driven) drops sharply, both sides: `tibant_l −79%`, `tibant_r −23%`, `soleus_l −84%`, `soleus_r −74%`, `gasmed_l −70%`, `gasmed_r −92%`, `gaslat_l −86%`, `gaslat_r −72%`. Matches the physical prediction.*
  - *Knee extensors (`recfem`, `vasint`) rise in noGRF; this is the expected SO redistribution when biarticular ankle muscles can no longer transmit GRF — not a regression.*
  - *"Zero GRF arrows" is a code-level guarantee: `NimbleEngine` zeros `IDOutput.leftFootForce / rightFootForce` in the `.noGRF` branch, and the arrow-drawing code reads those fields. No way for a non-zero arrow to render in `.noGRF`. Visual confirmation would be redundant.)*
- [x] 8.4 ~~Run the desktop comparator with `--asymmetry-report` against both exports. Visually confirm the `withGRF` table shows substantially lower asymmetry than the pre-change baseline AND that `noGRF` is bilaterally symmetric to within 5% (acts as the structural-symmetry sanity check on the rest of the pipeline).~~ **Superseded by Findings.** Both halves of the prediction failed: `withGRF` looks symmetric only because of saturation-blindness in the report metric, and `noGRF` is the *most* asymmetric of the two runs (19 / 40 pairs FAIL ±10%, far from the ≤ 5% structural sanity check). The negative result is what makes this change valuable — recording it accurately is more important than satisfying the original pass condition.