## Why

Offline runs on OpenCap `.mot` trials show structurally asymmetric muscle activations: when the left foot strikes, `tibant_l` activates noticeably higher than `tibant_r` does on the mirrored right-foot strike — and the same pattern shows up across other antagonist pairs around the ankle and hip. The OpenCap input is multi-camera triangulated IK and is L/R-symmetric to within video noise, so the asymmetry is being *introduced* somewhere downstream. The two structural suspects are:

1. `**solveIDGRF` ground tracking** — `_groundHeightY` is a single scalar that ratchets to the running minimum of *either* heel across the whole trial, then the contact threshold is `(calcn_y − ground) < 6 cm` for both feet against that one number. If one heel ever reaches lower than the other (gait asymmetry, sub-cm pelvis lean, OpenCap residual), that side's ground bar drags down for the entire rest of the trial and the *opposite* foot then crosses contact threshold less often. ID then attributes whole-body weight asymmetrically across the cycle, and SO faithfully reproduces the asymmetric ankle moment as asymmetric tibant activation.
2. **No way to A/B verify** — there is currently no production-safe way to re-run the same trial with GRF estimation disabled, so we cannot empirically confirm or rule out (1) without manual code edits.

The user-visible outcome we want: bilateral muscle activations on a symmetric gait trial should be symmetric within ~10% peak-amplitude tolerance, and we should never lose the ability to verify that claim on any future trial.

## What Changes

- Add `idMode: IDMode = .withGRF` engine-level state to `NimbleEngine`, exposed as a "No-GRF mode (validation)" toggle in `OfflineAnalysisView`. In `.noGRF` the engine calls `solveID` (no contact detection, no Newton-Euler-near-CoP) and zeros all GRF/CoP/contact fields on the resulting `IDOutput`. Defaults to `.withGRF` — production behavior unchanged.
- Replace the single-scalar `_groundHeightY` ratchet in `NimbleBridge.solveIDGRF` with **per-foot independent ground heights** (`_groundHeightLY`, `_groundHeightRY`), each tracked as the running minimum of its own heel y-coordinate (with the existing 1 cm offset). Contact detection uses each foot's own ground bar.
- Add **hysteresis** to per-foot contact detection: once a foot is in contact, it stays in contact until `(calcn_y − ground) > CONTACT_RELEASE` (default 8 cm); previously it dropped to swing the instant it crossed the 6 cm threshold, which combined with the asymmetric ground bar to produce frame-level chatter that the SO's warm-start then smeared.
- Extend `OfflineAnalysisTrack` and `OfflineAnalysisExporter` with `idMode: String` (`"withGRF"` | `"noGRF"`) so the same trial can be exported under both modes and the desktop pipeline can tell them apart.
- Add a `bilateral-asymmetry` report mode to `desktop-pipeline/src/biomotion_desktop/compare.py`: for every L/R muscle pair (suffix `_l` / `_r`), compute peak activation difference, RMS-of-(left − right), and a per-trial asymmetry index. Surface a single pass/fail summary that catches regressions on the committed fixture.
- Document in `desktop-pipeline/docs/comparison_semantics.md` how to read the bilateral-asymmetry report and how the per-foot ground tracking changes the GRF semantics relative to the legacy single-scalar version.

**Non-goals:**

- Do **not** change the contact solver itself (`getMultipleContactInverseDynamicsNearCoP`). Only change which feet are passed in and what ground plane is used.
- Do **not** introduce new bridge surface or new C++ exports. All changes use existing `NimbleBridge.solveID` / `solveIDGRF` calls.
- Do **not** rewrite the SO solver. If the bilateral-asymmetry report still shows residual L/R bias after the fix lands, that's a separate change (likely the iPhone-SO-on-OpenSim-IK+ID isolation experiment, intentionally deferred).
- Do **not** modify OpenCap input handling, the `.mot` parser, or `processImportedMotionFrame` — the angles flowing in are treated as ground truth.
- Do **not** regress the existing live ARKit path. The new `_groundHeightLY/RY` ivars apply uniformly to live and offline use; the toggle is offline-only.

## Capabilities

### New Capabilities

- `bilateral-grf-tracking`: `NimbleBridge.solveIDGRF` tracks the ground plane independently per foot, with hysteresis on contact transitions, so a single asymmetric heel position cannot bias the contact detection of the opposite foot.
- `id-grf-toggle`: `NimbleEngine` exposes an `idMode` selector that swaps `solveIDGRF` for `solveID` on the offline pipeline, so bilateral asymmetry can be A/B-tested with GRF estimation in or out of the loop.
- `bilateral-asymmetry-report`: `desktop-pipeline` analyses any iOS `<stem>_activations.sto` for L/R muscle-pair symmetry and emits a peak-difference + asymmetry-index report, with a documented pass threshold for CI.

### Modified Capabilities

- (none — `openspec/specs/` is empty; this is the first OpenSpec change to introduce these capabilities, so no prior requirements are being amended.)

## Impact

**Code touched:**

- `BioMotion/Nimble/NimbleBridge.h/.mm` — add `_groundHeightLY`/`_groundHeightRY` ivars, replace the single-scalar `_groundHeightY` ratchet with per-foot tracking, add `CONTACT_RELEASE` hysteresis, keep `groundHeightY` accessor returning `min(L, R)` for back-compat with the existing UI badge.
- `BioMotion/Nimble/NimbleEngine.swift` — add `enum IDMode { case withGRF, noGRF }`, `var idMode: IDMode = .withGRF`, branch `runDynamicsAndMuscles` on it, zero GRF/CoP/contact fields when `.noGRF`.
- `BioMotion/Offline/OfflineSession.swift` — add `var idMode: NimbleEngine.IDMode = .withGRF`, propagate to `nimble.idMode` before `processAllFrames`.
- `BioMotion/App/OfflineAnalysisView.swift` — "No-GRF mode (validation)" toggle bound to `OfflineSession.idMode`.
- `BioMotion/Offline/OfflineAnalysisTrack.swift` — add `idMode: String` field (default `"withGRF"`).
- `BioMotion/Offline/OfflineAnalysisExporter.swift` — write `idMode` into `<stem>_summary.json` and `<stem>_processed.json`; tolerate absent key on older blobs.
- `desktop-pipeline/src/biomotion_desktop/compare.py` — new `_bilateral_asymmetry(...)` helper, opt-in via `--asymmetry-report` CLI flag, emitted alongside existing tier output.
- `desktop-pipeline/src/biomotion_desktop/cli.py` — wire the flag.
- `desktop-pipeline/docs/comparison_semantics.md` — section on per-foot ground tracking + bilateral asymmetry semantics.

**No new bridge surface.** Reuses `NimbleBridge.solveID` and `solveIDGRF` as-is. No `nimblephysics` patches touched.

**Overlap with existing change `validate-ik-id-so-equivalence`:**
That change's tasks 1.1–1.7 introduce the same `idMode = .noGRF` toggle as a precondition for its ID-equivalence comparator. This change subsumes those tasks — once it lands, drop tasks 1.1–1.7 from `validate-ik-id-so-equivalence` (its tasks 1.5–1.6 stay relevant because they cover the export-schema change, but they become "already done"). The two changes have no other overlap.

**Build system:** no `project.yml` changes. New Swift source files (none introduced here — all edits are in-place on existing files) would be picked up by XcodeGen globs anyway.

**Backward compatibility:** older `<stem>_processed.json` blobs lacking `idMode` decode with default `"withGRF"`. Older `compare.py` invocations without `--asymmetry-report` behave exactly as today. Production live ARKit pipeline uses the same per-foot ground tracking — this is a quality improvement, not a regression risk, but it does change the single-scalar `groundHeightY` reading shown in the UI badge to `min(L, R)`; documented in `docs/UI_PARAMETERS.md`.

## Findings (post-implementation, 2026-04-30)

The change shipped end-to-end (per-foot ground tracking + hysteresis + `IDMode` toggle + bilateral asymmetry report). It was then run on the `run2` OpenCap fixture (31.7 s, 120 fps, 3807 frames, mass 56.07 kg) under both modes back-to-back. The frozen exports live at `desktop-pipeline/tests/fixtures/bilateral_grf_baseline/{withGRF,noGRF}/`.

**Result: the central hypothesis is falsified.**


| run       | `idMode`  | FAIL pairs (±10%) | example asymmetries                                                                                                                               |
| --------- | --------- | ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withGRF` | `withGRF` | 3 / 40            | `fdl +45.3%`, `fhl +28.4%`, `tibpost +82.9%`                                                                                                      |
| `noGRF`   | `noGRF`   | **19 / 40**       | `tibant +15.2%`, `gaslat +70.2%`, `gasmed −62.0%`, `bflh +51.3%`, `glmax1 +42.3%`, `glmed2 −45.5%`, `soleus +85.1%`, `edl +123.1%`, `ehl +112.3%` |


**§8.4 predicted that `noGRF` would be bilaterally symmetric within 5% as a structural sanity check on "the rest of the pipeline".** It came back the most asymmetric of the two runs — i.e. when GRF estimation is bypassed entirely (`bridge.solveID` instead of `solveIDGRF`, no contact detection, no Newton-Euler-near-CoP), bilateral muscle activations are *more* lopsided, not less. This rules out the single-scalar `_groundHeightY` ratchet as the dominant source of the asymmetry the change was named after.

`**design.md`'s first-principles argument is therefore broken.** That doc claimed "Stage 1 cannot introduce L/R asymmetry … Stage 3 cannot … Stage 4 is structurally symmetric … the only stage that mixes L/R information is stage 2". The `noGRF` evidence shows this elimination chain is wrong — at least one of stages 1 / 3 / 4 (SG smoothing, moment arms, OSQP SO) **does** introduce L/R asymmetry, or the OpenCap input itself is not as symmetric as the change assumed. Locating which is the job of the follow-up change, not this one.

**Secondary finding: `bilateral-asymmetry-report` is saturation-blind.** In the `withGRF` table, ~30 muscle pairs report `peak_l = peak_r = 1.000` and `asym_idx_pct = +0.0%`, but their `rms_diff` is 0.3–0.6 — both sides saturate to the activation upper bound (1.0) at *different times*, and the peak-only index cannot see the temporal mismatch. The "3 / 40 FAIL" number for `withGRF` is therefore misleadingly low. The follow-up change must add an `rms_diff` (or cross-correlation lag) gate to the PASS/FAIL summary before any future claim of "symmetry restored" is trustworthy.

### What this change still delivers (kept)

The three capabilities below are real, correct, and shipped — independent of the falsified hypothesis:

- `**bilateral-grf-tracking*`* — per-foot ground ratchet + hysteresis is **physically more correct** than a single-scalar bar and a no-hysteresis 6 cm gate, regardless of whether it explains the observed asymmetry. It removes a known frame-level chatter source. Keep.
- `**id-grf-toggle`** — the `IDMode = .noGRF` switch is what produced the falsifying evidence above. Without it the assumption would still be live and unverified. This is the change's most important deliverable, full stop.
- `**bilateral-asymmetry-report**` — the diagnostic table is what surfaced the saturation-blindness AND the noGRF result. The metric needs strengthening (above), but the framework is the right shape.

### Open regression blocking archive

**Live ARKit muscle visualization stopped rendering after this change shipped.** The user verified `8.1` once on device: no crashes, kinematics tracking works, badge values appear, but the RealityKit muscle capsules that were visible in the pre-change build no longer appear at all. Code-level analysis could not localize the regression — the `NimbleEngine` live `.withGRF` path is byte-equivalent to pre-change behavior in the diff, and `MuscleOverlay.update()` is designed to fall back to `0.01` (blue) for any muscle missing from the activations dict rather than skipping the entity. The most plausible cut point is `SkeletonOverlayView.swift:30`: `muscleOverlay.update(...)` is only called when `muscleActivations != nil`, so anything that makes `nimble.lastMuscleResult` stay `nil` for the whole session would silently kill the viz.

This regression is **not yet root-caused** and **not yet reproducible without a subject in front of an iPhone**. It is recorded here, with a step-by-step bisect plan in `tasks.md` §8.1, so the next on-device session can resolve it efficiently. Until it is resolved (or split into a separate `diagnose-live-muscle-viz` change after a more thorough bisect proves it's pre-existing rather than caused by this change), this change cannot be archived.

### What is deferred (intentionally not scoped into this change)

- Identifying the **actual** source of bilateral asymmetry (likely candidates: OpenCap input asymmetry on this particular trial, SG smoothing edge effects, moment-arm numerical-diff sign handling, or the OSQP warm-start). → Belongs in a new `diagnose-bilateral-asymmetry-source` change that uses the toggle + report shipped here as its primary tools.
- Strengthening `bilateral-asymmetry-report` with an `rms_diff` or cross-correlation gate. → Belongs as the first task of that follow-up change, since every diagnostic step depends on a non-saturation-blind metric.
- Picking a clean bilaterally-symmetric reference trial (the `run2` 31.7 s clip is long enough to contain non-steady segments). → Same follow-up change.

The `IDMode = .noGRF` toggle moves from "experimental validation flag" (this change's framing) to "permanent diagnostic surface" (since it's the only way to bisect GRF-estimation bias from upstream-pipeline bias on any future trial).