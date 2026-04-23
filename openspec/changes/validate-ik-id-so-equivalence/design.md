## Context

The current iOS↔desktop comparator (`desktop-pipeline/src/biomotion_desktop/compare.py`) is honest about *what* it measures (`compare.json` records RMSE/R²/phase lag per channel) but the inputs to those metrics aren't apples-to-apples:

| Stage | iOS offline path | Desktop path | What current diff actually says |
|---|---|---|---|
| **IK** | `OfflineSession.importMotion(.mot)` → `nimble.processImportedMotionFrame` (no IK) → SG-filter q | `opensim_ik` (Butterworth filtfilt) on `.trc` markers | Two different filters applied to two different sources |
| **ID** | `solveIDGRF` (Newton-Euler near-CoP, auto-estimated GRF, ratcheting `_groundHeightY`, 6 cm contact threshold) | `opensim_id` with no GRF | Two different physics problems |
| **SO** | OSQP soft-equality, rigid-tendon Hill, `aMin=0.01`, warm-start | OpenSim StaticOptimization, hard-equality + reserve actuators, equilibrium-tendon Millard | Two different convex programs |

Affected pipeline stages: **IK / ID / SO** (everything from kinematics to muscle activations). No ARKit, no RealityKit visualization, no on-device hot-path code. All work is in: `OfflineSession` (main actor), `NimbleEngine` (background `solverQueue`), and `desktop-pipeline/` (Python).

Stakeholders: pipeline accuracy work + future paper. Unblocks: deciding whether iOS Nimble can claim "same answer as OpenSim" within documented tolerances.

## Goals / Non-Goals

**Goals:**
- Make IK comparable: feed iOS Nimble IK and OpenSim IK the **same `.trc`** and diff the resulting joint angles per DOF.
- Make ID comparable: run iOS in a **no-GRF** mode that matches the desktop's no-GRF ID, isolating the algorithm from the GRF-estimation question.
- Make SO interpretable: replace per-frame Pearson R² with metrics that reflect what SO is actually solving (timing, co-contraction, saturation, energy proxy).
- Run all three diffs sequentially in a single CLI invocation so a regression in any solver lights up before merge.
- Document what each tier's numbers mean and what each one's "pass" threshold is, so the existing R² 0.30 / R² 0.08 numbers stop being read as "iOS is wrong".

**Non-Goals:**
- Do **not** replace or modify the existing `compare` CLI / `.mot`-driven offline path. Both stay as-is.
- Do **not** change the production iOS pipeline behavior. The new no-GRF flag defaults to `withGRF`; the marker-input path is opt-in via the file picker.
- Do **not** rewrite `MuscleSolver` to match OpenSim StaticOptimization (different muscle models is intentional). We measure agreement, we don't force it.
- Do **not** introduce new bridge surface (no new ObjC++ wrappers, no new C++ exports).
- Do **not** target real-time pipelines. The marker-input path is offline-only and gated by `OfflineSession`.

## Decisions

### D1. Marker input enters via `OfflineSession`, not via a new bridge call.

`NimbleBridge.solveIK` already exists and is exercised by the live ARKit path (`NimbleEngine.processFrame`). The marker-input path reuses it: `OfflineSession.importMarkers(.trc)` → `MarkerSeries` → per-frame `[NSNumber]+[String]` arrays → existing `solveIK` → existing `processRawKinematics`. No C++ changes.

**Alternative considered:** add a "batch IK" call to `NimbleBridge` that takes the whole trial. Rejected — keeps the bridge surface clean, and per-frame dispatch already drives the live path so we know it's stable.

**Thread/queue:** parsing on main actor (matches `MOTParser`); per-frame `solveIK` on `NimbleEngine.solverQueue` (same as live ARKit). No hot-path impact.

### D2. `idMode` is engine-level state, not a per-frame argument.

`NimbleEngine` gains `var idMode: IDMode = .withGRF`. `runDynamicsAndMuscles` branches on it: `.withGRF` calls `bridge.solveIDGRF`, `.noGRF` calls `bridge.solveID`. The flag is set once by `OfflineSession` before `processAllFrames` runs and read by the solver thread; protection: only mutated on main, only read inside `solverQueue.async` blocks (publish boundary already serializes via `solverQueue`).

**Alternative considered:** parameterize `processImportedMotionFrame(_:withGRF:)`. Rejected — pollutes the live `processFrame` path's API and forces every caller to pick. Engine-level flag matches how `isModelLoaded` / `isRecordingResults` already work.

**No-GRF muscle behavior:** when `idMode == .noGRF`, the resulting torques include the floating-root residual. The muscle solver still runs but only on actuated DOFs (root residuals aren't mapped to muscles), exactly like OpenSim's StaticOptimization on the same input. This is the apples-to-apples comparison.

### D3. `.trc` parser is hand-rolled, mirrors `MOTParser`.

The TRC ASCII format is small (5-line header + tab-separated marker XYZ rows). Adding a third-party dependency or pulling more nimble surface (`OpenSimParser::loadTRC` is gated behind iOS patches we don't want to touch) is overkill. `TRCParser.parse(url:) throws -> MarkerSeries` is ~80 lines of Swift. Same lifecycle conventions as `MOTParser` — pure value type result, no I/O outside the parse call.

### D4. Marker name reconciliation: model-side wins, missing markers warn.

`solveIK` already accepts `(positions, names)` and tolerates missing markers. `OfflineSession.importMarkers` emits a `warningMessage` listing TRC markers absent from `bridge.markerNames` (typo / wrong model) and model markers absent from the TRC (incomplete capture). Mirrors how `prepareImportedMotion` already reports unmatched DOFs. No silent failures.

### D5. Desktop comparator gets a new `equivalence` subcommand, not changes to `compare`.

`desktop-pipeline/src/biomotion_desktop/equivalence.py` is a new module. New CLI: `biomotion-desktop equivalence --desktop <dir> --ios <dir> --stages ik,id,so [--id-mode no_grf]`.

Three stages, three metric profiles:

- **Stage IK** — per-DOF: RMSE (deg), max\_abs (deg), R² (after best integer-sample shift up to ±100 ms), phase\_lag\_ms. Pass: RMSE ≤ 2° **after phase compensation**, R² ≥ 0.95 on every actuated DOF that has range > 5°.
- **Stage ID** — per-DOF: RMSE (Nm/kg of body mass), R², bias. Pass: only meaningful when both sides are no-GRF (verified via the iOS export's `idMode` field — see D6). RMSE ≤ 0.3 Nm/kg, R² ≥ 0.90 on actuated DOFs with peak |τ| > 0.5 Nm/kg.
- **Stage SO** — per-muscle: onset\_diff\_ms, peak\_time\_diff\_ms, offset\_diff\_ms (existing Tier-3 in `compare.py`); plus new metrics: agonist/antagonist co-contraction Pearson, saturation rate (% frames at `a ≥ 0.95`), `Σa²` per gait cycle. Pass criteria documented per metric, not gated as a single number.

`equivalence` calls into existing loaders (`storage_io`, `ios_export`) and the existing per-channel statistics where they apply; it adds shift-search-by-default for IK and the new SO metrics. Output: `equivalence.json` next to `compare.json` (independent files, both kept).

**Alternative considered:** modify `compare.py` to add the new stages. Rejected — `compare.py` is a one-shot diff with documented thresholds; making it conditionally apples-to-apples on a per-stage basis bloats its semantics. Equivalence is a separate capability.

### D6. iOS export records `inputType` and `idMode` in the summary JSON.

`OfflineAnalysisExporter` writes a small block:

```json
{ "inputType": "trc" | "mot", "idMode": "withGRF" | "noGRF", ... }
```

The desktop `equivalence` CLI reads this and refuses to run the ID stage with `--no-grf-pair-check` unless both sides are no-GRF. Prevents the "iOS auto-GRF vs desktop no-GRF" trap from happening silently again.

### D7. `validate_demo.sh` becomes the regression harness.

Existing script is a placeholder. Promote it to: run desktop pipeline on a checked-in OpenCap fixture, run iOS export via a previously-captured artifact (committed under `desktop-pipeline/fixtures/ios/<trial>-trc/` and `<trial>-mot/`), invoke `equivalence`, fail nonzero if any stage drops below its threshold. CI calls this script.

The iOS export fixture is captured manually once (offline run on a real device or the simulator's offline mode) and committed; we don't try to run iOS in CI.

## Risks / Trade-offs

- **Risk:** `.trc` files commonly use `mm` while the iOS Nimble model expects `m`. → Mitigation: `TRCParser` reads the `Units` row of the TRC header; if `mm`, scales positions by 1e-3 before emitting `MarkerSeries`. Any unknown unit raises a parse error.
- **Risk:** Marker-name drift between OpenCap TRC (`r.ASIS`, `L.PSIS`, …) and the iOS model's marker set (`RASI`, `LPSI`, …). → Mitigation: a small alias table in `MarkerSeries` (the same one OpenSim uses internally for OpenCap data); unmatched markers are reported via `warningMessage`, not silently dropped.
- **Risk:** `solveID` (no-GRF) torques on the floating root will be large by construction (no contact forces to balance gravity). → Mitigation: `equivalence` masks the 6 root DOFs out of the ID stage by default and reports them in a separate "root-residual" block; the pass threshold only applies to actuated DOFs. This matches how OpenSim's StaticOptimization treats reserve actuators.
- **Risk:** SG filter inside `NimbleEngine` introduces ~4-frame group delay; OpenSim's Butterworth `filtfilt` is zero-phase. → Mitigation: the IK-stage equivalence does best-integer-sample shift search (D5) before computing RMSE. Documented in `comparison_semantics.md` so the lag value is read as filter group delay, not "iOS is laggy".
- **Risk:** The marker-input path bypasses `prepareImportedMotion`, so the unmatched-DOF reporting that exists for the `.mot` path doesn't trigger. → Mitigation: emit equivalent warnings from `importMarkers` based on `bridge.markerNames` vs the parsed TRC marker set (D4).
- **Risk:** Adding `idMode` to the published summary changes the schema consumed by `loadProcessedAnalysisFolder`. → Mitigation: make the new fields optional with defaults (`idMode = "withGRF"`, `inputType = "mot"`) so old exports keep loading. Backward-compatible.
- **Trade-off:** SO equivalence will not produce a single "is it equivalent" number. We accept this — the muscle models are intentionally different, and a single Pearson R² is the wrong question. The pass criteria are per-metric and documented.
- **No nimblephysics patches required.** We only call existing exported functions on `NimbleBridge`. No `DART_IOS_BUILD` guards touched.

## Open Questions

- (None blocking implementation.) The OpenCap fixture used by `validate_demo.sh` should be small enough to commit (one short trial, ~5 s); pick a public OpenCap demo trial during task M3.
