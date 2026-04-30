## Context

The offline pipeline takes OpenCap-derived `.mot` joint angles, runs `NimbleEngine.processImportedMotionFrame` per row, which:

1. SG-filters the angles to get `(q, dq, ddq)` per DOF.
2. Calls `bridge.solveIDGRF(q, dq, ddq)` — this is where contact detection, ground tracking, GRF estimation, and ID happen together.
3. Computes moment arms via `MomentArmComputer` (FK + numerical diff).
4. Calls `muscleSolver.solveReal(...)` (OSQP) to get per-muscle activations.

Stage 1 cannot introduce L/R asymmetry — it's a per-DOF symmetric filter. Stage 3 cannot introduce L/R asymmetry — moment arms come from FK on the same skeleton with the same numerical perturbation. Stage 4 is structurally symmetric — `½ aᵀ (λ·RᵀR + ε·I) a − λ (τᵀ R) a` with bounds `[0.01, 1.0]` doesn't bias one side over the other; if the inputs are symmetric the activations are symmetric. **The only stage that mixes L/R information is stage 2** — and inside it, the only state that carries L/R coupling is `_groundHeightY`, a single scalar that ratchets to the running min of *either* heel.

Concretely, in `BioMotion/Nimble/NimbleBridge.mm` around lines 633–648:

```
_groundHeightY = lowest - 0.01;          // lowest = min(calcnLY, calcnRY)
...
const double CONTACT_THRESHOLD = 0.06;
BOOL leftContact  = (calcnLY  - _groundHeightY) < CONTACT_THRESHOLD;
BOOL rightContact = (calcnRY  - _groundHeightY) < CONTACT_THRESHOLD;
```

If, on any earlier frame, the left heel reached y = 0.02 m while the right heel never went below y = 0.05 m, then `_groundHeightY` ratchets to 0.01 m and stays there. From that frame on, the right foot's contact gate becomes `calcnRY < 0.07 m` — narrower in absolute terms than the left foot's `calcnLY < 0.07 m` would have been if measured against its own heel low. The result: the right foot ends up declared in contact for fewer frames than the left, so right-side ankle ID (and thus right tibant in SO) sees less load support and produces lower activation.

This change replaces the single ground bar with a per-foot bar, adds hysteresis to remove single-frame contact chatter, and adds a no-GRF toggle so we can A/B-test the fix.

The pipeline runs on `solverQueue` (`DispatchQueue(label: "com.biomotion.nimble", qos: .userInteractive)`) — all bridge state mutations happen serially on that queue. No threading changes here.

## Goals / Non-Goals

**Goals:**
- Eliminate L/R asymmetry that originates in the GRF estimator. Target: bilateral peak-activation difference < 10% on the committed OpenCap fixture for all standard antagonist pairs (`tibant`, `gaslat`, `recfem`, `bflh`, `glmax`, `glmed`).
- Make the no-GRF code path a first-class production-safe toggle (no source edits, no DEBUG flags) so any future asymmetry suspicion can be A/B'd in seconds.
- Surface bilateral asymmetry as a CI-checkable metric in the desktop pipeline, so we don't lose ground silently again.

**Non-Goals:**
- Do **not** redesign `getMultipleContactInverseDynamicsNearCoP`. We keep using the existing solver as-is; only its inputs change.
- Do **not** introduce new bridge surface (no new `@interface` methods on `NimbleBridge` beyond the per-foot accessors needed for diagnostics; no new ObjC++ files).
- Do **not** change the live ARKit code path's behavior beyond inheriting the per-foot ground bar (which is strictly more accurate, not a regression).
- Do **not** add IPOPT, MarkerFitter, or any nimble feature gated by `DART_IOS_BUILD`. No `nimblephysics/` patches in this change.
- Do **not** refactor the SG filter, OSQP setup, or moment arm computer.

## Decisions

### D1. Per-foot ground bars, single-scalar passed to the solver

**Decision:** Track `_groundHeightLY` and `_groundHeightRY` independently inside `NimbleBridge`. Use them independently for **contact detection**. When passing the ground height into `_skeleton->getMultipleContactInverseDynamicsNearCoP`, pass `min(_groundHeightLY, _groundHeightRY)` as the single scalar `groundHeight` argument the solver requires.

**Why:** The solver only takes one scalar ground-plane y. Splitting it would require modifying nimblephysics (a `DART_IOS_BUILD` patch), which violates the non-goal. The contact-detection asymmetry is the dominant effect — the CoP solver projection happens *after* contact bodies are chosen, so it only matters for feet already declared in contact, and at that point both feet are within ~6 cm of `min(L, R)` anyway, so projecting both onto `min(L, R)` introduces <1 cm of CoP-y bias per side, negligible compared to the contact-set asymmetry.

**Alternatives considered:**
- *Solve per-foot with separate ground bars and aggregate.* Requires two ID solves per frame and reconciliation of the floating-root residual; doubles ID time (currently ~0.1 ms → ~0.2 ms, still fine, but adds genuinely more code than the contact-only fix and risks weight-distribution bugs).
- *Patch nimblephysics to take per-foot ground bars.* Worst option: violates the non-goal of touching `DART_IOS_BUILD` patches and ties us to a fork of an upstream solver for a fix that demonstrably doesn't need it.

### D2. Hysteresis with `CONTACT_ATTACH = 0.06 m`, `CONTACT_RELEASE = 0.08 m`

**Decision:** Track `_leftInContact`, `_rightInContact` as ivars on `NimbleBridge`. A foot enters contact when its heel-to-ground gap drops below `CONTACT_ATTACH`; it leaves contact only when the gap rises above `CONTACT_RELEASE`. Both thresholds are constexpr in `solveIDGRF` for now (no UI tunables yet).

**Why:** Without hysteresis, a foot whose heel oscillates around 0.06 m due to SG smoothing residual flips contact state every other frame. That makes `contactCount` toggle 1↔2 frame-to-frame, which makes `perFootForce` toggle `mass*g` ↔ `mass*g/2`, which makes ID torques discontinuous, which makes `MuscleSolver` (warm-started) chase a moving target. The 2 cm hysteresis band is roughly the standard-deviation of OpenCap heel y on a "stationary" stance, so it eliminates chatter without delaying real heel-strike/toe-off events by more than ~1 frame at 60 Hz.

**Alternatives considered:**
- *Single threshold.* Status quo. Demonstrably produces chatter on multiple recorded fixtures.
- *Larger hysteresis band (e.g. 0.06 / 0.12).* Delays toe-off detection unacceptably (~5 frames at 60 Hz). Not worth it for the marginal reduction in remaining chatter.
- *Velocity gate (`|dq_calcn_y| < threshold`).* Adds a second threshold to tune and depends on noisy SG `dq` near contact transitions. Hysteresis is cheaper, more local, and easier to reason about.

### D3. `IDMode` is engine-level, not bridge-level

**Decision:** `enum IDMode { case withGRF, noGRF }` lives on `NimbleEngine`, not `NimbleBridge`. The branching happens in `NimbleEngine.runDynamicsAndMuscles` (Swift), which calls either `bridge.solveIDGRF` or `bridge.solveID` with the same arguments. When `.noGRF`, the engine constructs the resulting `IDOutput` with all GRF/contact/CoP/residual fields zeroed.

**Why:** The bridge is intentionally dumb about what mode it's in — it offers two APIs, the orchestrator picks. Putting the toggle on the bridge would tempt future code to flip it deep in C++ without the Swift orchestrator knowing, which would make the published `lastIDResult.leftFootForce` semantics ambiguous. With the toggle in Swift, every reader of `lastIDResult` knows that "all GRF fields == zero AND `engine.idMode == .noGRF`" means "the bridge wasn't asked to estimate GRF" rather than "the bridge tried and failed."

**Alternatives considered:**
- *Toggle on `NimbleBridge`.* Rejected — see above.
- *Two completely separate engines.* Massive over-engineering for a one-bit branch.

### D4. `IDMode` is offline-only at the UI layer

**Decision:** Render the toggle only in `OfflineAnalysisView`. Live ARKit capture always uses `.withGRF`. This is enforced by where the toggle is declared (in the offline UI), not by a runtime check on `engine.idMode`.

**Why:** Live ARKit needs GRF for the muscle viz to be physically meaningful. Surfacing the toggle live would invite users to disable GRF and then wonder why all their activations look like noise. Offline analysis is the only place where "compare with vs. without GRF" is a meaningful question.

### D5. Backward-compatible `groundHeightY` accessor returns `min(L, R)`

**Decision:** Keep the existing `- (double)groundHeightY` ObjC accessor on `NimbleBridge`, returning `min(_groundHeightLY, _groundHeightRY)` (or 0 if neither calibrated). Add `- (double)groundHeightLY` and `- (double)groundHeightRY` as new accessors. The published Swift `@Published var groundHeightY: Double` continues to bind to the existing accessor.

**Why:** One existing UI badge ("Ground: 0.012 m") already binds to `bridge.groundHeightY`. Returning `min(L, R)` preserves the semantics ("lowest observed contact height across both feet") and keeps the badge correct for the common case where both feet land at similar heights. New per-foot accessors let us surface "(L: 0.01, R: 0.05)" in `docs/UI_PARAMETERS.md` and a future diagnostic view without breaking the current binding.

### D6. Bilateral-asymmetry report is opt-in on the desktop comparator

**Decision:** Add `--asymmetry-report` to `desktop-pipeline`'s comparator CLI. Default behavior is byte-identical to today.

**Why:** Existing CI consumers and diff scripts depend on the current stdout format. The asymmetry report is additional information, not a replacement. Opt-in keeps the downstream contract intact.

**Schema:** the report appends a section like:

```
=== Bilateral Asymmetry Report (threshold ±10.0%) ===
pair         peak_l    peak_r    peak_diff   asym_idx_pct   flag
tibant       0.413     0.198     0.215        70.6           FAIL
gaslat       0.612     0.589     0.023         3.8           ok
recfem       0.341     0.336     0.005         1.5           ok
...
SUMMARY: FAIL (1 pair exceeds threshold: tibant)
```

`asym_idx_pct = 100 * (peak_l - peak_r) / (0.5 * (peak_l + peak_r))`.

### D7. Per-frame budget: no impact

**Decision:** No new operations in the hot path. The fix replaces one scalar update + two threshold compares with two scalar updates + two threshold compares + two boolean state reads. ID time stays at ~0.1 ms / frame. Total per-frame budget unchanged at ~1.7 ms (well under the 16.7 ms / 60 Hz target).

## Risks / Trade-offs

- **[Risk]** *The asymmetry isn't actually GRF-driven.* The fix could land cleanly and still leave residual L/R asymmetry, indicating the source is upstream (OpenCap IK itself) or in a place I haven't inspected. → **Mitigation:** the `idMode = .noGRF` toggle (D3, D4) is the empirical verification — if `noGRF` activations are also asymmetric on the same trial, GRF is exonerated and the next change is the OpenSim-ID-into-iPhone-SO isolation experiment. The toggle is shipping in the same change precisely so we can answer this binary question without a follow-on PR.

- **[Risk]** *`min(L, R)` ground passed to `getMultipleContactInverseDynamicsNearCoP` biases CoP-y for the foot whose own ground bar is higher.* → **Mitigation:** quantified above (<1 cm CoP-y error per side, well below the typical foot-length scale of CoP variation). If a future trial shows this matters, the fallback is D1's "solve per-foot separately" alternative — easy to retrofit because the contact-detection refactor already separates per-foot state.

- **[Risk]** *Hysteresis introduces a 1-frame delay on toe-off detection.* → **Mitigation:** 1 frame at 60 Hz = 16.7 ms; the SG filter already imposes a 4-frame group delay (~66 ms) on `dq/ddq`, so an extra 16.7 ms on contact transition is dwarfed and downstream consumers won't notice. The compensating gain (no per-frame chatter) more than pays for it.

- **[Trade-off]** *The new accessors `groundHeightLY` / `groundHeightRY` widen the bridge surface.* → Acceptable: they are read-only doubles, no new C++ types crossing the bridge, and only consumed by Swift display code. Zero integration cost.

- **[Trade-off]** *We're shipping a behavior change to the live ARKit path (per-foot ground bar is now used live too).* → This is a quality improvement, not a regression — live capture currently has the *same* asymmetry bug we're fixing offline. Documented in `docs/UI_PARAMETERS.md`. No setting needed to revert; if a regression is observed, both `_groundHeightLY` and `_groundHeightRY` get the same value and behavior degrades gracefully to today's behavior.

## Migration Plan

This is a behavior fix, not a data migration. Roll-forward only.

1. Land `NimbleBridge` ivars + per-foot ground tracking + hysteresis (no UI change, no schema change). Existing trials re-process identically except for the asymmetry fix.
2. Land `NimbleEngine.IDMode` + `OfflineSession.idMode` + UI toggle + export schema change (`OfflineAnalysisTrack.idMode`).
3. Land desktop comparator `--asymmetry-report`.
4. Re-process the committed offline fixture under both modes; commit the resulting `_summary.json` blobs as baseline regression artifacts under `desktop-pipeline/tests/fixtures/bilateral_grf_baseline/`.
5. Add a CI check that runs the comparator with `--asymmetry-report --asymmetry-threshold-pct 10` against the committed fixture and fails the build on `FAIL`.

**Rollback:** revert in reverse order. The schema-change step is the only one that touches persisted data, and the decoder tolerates absent `idMode` (defaults to `"withGRF"`), so reverting the Swift code while leaving older blobs on disk is safe.

**TestFlight build number:** bump `CURRENT_PROJECT_VERSION` in `project.yml` after step 2 lands and before any TestFlight upload.

## Open Questions

- **Q1.** Should `CONTACT_ATTACH` / `CONTACT_RELEASE` be tunable per-trial (via `OfflineSession`) for users running deliberately non-standard motions (e.g., barefoot ladders, tip-toe)? **Default answer:** no — keep them as constants for now; revisit if a real fixture requires it.
- **Q2.** When both feet ground bars exist but disagree by >10 cm (e.g., one foot on a step), should the solver fall back to the higher of the two for safety? **Default answer:** no — the user's recorded floor isn't expected to be that uneven; if it becomes one, that's a separate "uneven terrain" feature.
- **Q3.** Does the iPhone live capture flow want a "(L: 0.01, R: 0.05)" diagnostic readout in the existing ground-height badge, or is the legacy `min(L, R)` enough? **Default answer:** legacy is enough for now; widen the badge in a future UI polish change if the diagnostic value becomes apparent.
