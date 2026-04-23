## Why

Today's iOS↔desktop comparator (`desktop-pipeline/`) reports kinematics R² ≥ 0.99 (after a 33 ms phase shift), ID R² 0.30, and SO R² 0.08 — but those numbers are misleading because the two sides are not solving the same problems:

- **IK is not actually compared.** iOS offline mode imports OpenCap `.mot` (joint angles) and skips IK entirely; desktop runs OpenSim IK on `.trc` markers. The "kinematics" diff is `OpenCap-supplied .mot` vs `OpenSim-IK on the same markers`, with two different filters layered on top.
- **ID is not the same problem.** iOS estimates GRF via Newton-Euler near-CoP; desktop runs ID with no GRF on the OpenCap demo data. Magnitudes are incomparable by construction.
- **SO is not the same algorithm.** iOS uses a soft-equality OSQP with rigid-tendon Hill and a 0.01 activation floor; OpenSim uses hard equality + reserve actuators with equilibrium-tendon Millard. Per-frame Pearson R² is the wrong yardstick.

Result: we currently cannot answer the question "given identical input, do iOS Nimble IK / Nimble ID / OSQP SO produce the same physics as OpenSim IK / ID / StaticOptimization?" — and that's the question that gates publishability.

## What Changes

- Add a marker-driven path to iOS offline mode so IK can be exercised on the same `.trc` the desktop consumes.
- Add an opt-in **no-GRF** code path in `NimbleEngine` that calls plain `solveID` instead of `solveIDGRF`, exposed as a flag on the offline batch run, so iOS↔desktop ID can be compared on equal footing.
- Add three layered comparators in `desktop-pipeline/` that run in sequence:
  1. **IK comparator** — iOS Nimble IK on markers vs OpenSim IK on the same markers, single-frame and time-series, with `q_iOS − q_desktop` per DOF.
  2. **ID comparator** — both sides without GRF, after IK matches, comparing `τ_iOS` vs `τ_desktop` per DOF.
  3. **SO comparator** — replace per-frame R² with biomechanically meaningful metrics: per-muscle activation timing (onset / peak / offset within ±50 ms), antagonist co-contraction correlation, saturation rate, and `Σa²` per gait cycle.
- Promote the three-stage validation to a CI fixture (`validate_demo.sh`) using a checked-in OpenCap trial so regressions in any of the three solvers are caught before merge.
- Document the three-tier validation semantics in `desktop-pipeline/docs/comparison_semantics.md` so the existing R² 0.30 / R² 0.08 numbers stop being read as "iOS is wrong".

## Capabilities

### New Capabilities
- `offline-marker-ik`: iOS offline mode accepts `.trc` markers and runs Nimble IK per frame, producing the same `OfflineAnalysisTrack` as the existing `.mot` path.
- `id-no-grf-mode`: `NimbleEngine` exposes a configurable flag that selects `solveID` (no GRF) vs `solveIDGRF` (with auto-estimated GRF) for the offline batch run.
- `solver-equivalence-validation`: `desktop-pipeline/` adds an `equivalence` CLI / test suite that diffs IK, ID (no-GRF), and SO results between iOS and OpenSim using metrics calibrated for what each solver step is allowed to disagree on.

### Modified Capabilities
- (none — this is the first OpenSpec change to introduce these capabilities; no existing spec files in `openspec/specs/`.)

## Impact

**Code touched:**
- `BioMotion/Offline/OfflineSession.swift` — new `importMarkers(.trc)` path; `processAllFrames` branches on motion-source type.
- `BioMotion/Offline/` — new `TRCParser.swift`, `MarkerSeries.swift` (mirrors `MOTParser` / `ImportedMotionSeries`).
- `BioMotion/Nimble/NimbleEngine.swift` — new `processMarkerFrame(_:)` that calls `bridge.solveIK` then `processRawKinematics`; new `idMode: IDMode` (`withGRF` / `noGRF`) on the engine.
- `BioMotion/App/OfflineAnalysisView.swift` — file picker accepts `.trc`; toggle for "no-GRF mode (validation)".
- `BioMotion/Offline/OfflineAnalysisExporter.swift` — summary JSON records `idMode` and `inputType` so the desktop comparator knows what it's diffing.
- `desktop-pipeline/src/biomotion_desktop/equivalence.py` — new module: per-stage comparator with stage-appropriate metrics.
- `desktop-pipeline/src/biomotion_desktop/cli.py` — new `equivalence` subcommand chaining IK → ID → SO comparisons.
- `desktop-pipeline/tests/test_equivalence.py` — synthetic + fixture-based tests.
- `desktop-pipeline/scripts/validate_demo.sh` — promote to a real fixture-driven harness invoked from CI.
- `desktop-pipeline/docs/comparison_semantics.md` — new document.

**No breaking changes** to the existing `.mot` offline path or the existing `compare` CLI — both stay as-is. The new capabilities are additive.

**New bridge surface:** none. Reuses existing `NimbleBridge.solveIK` / `solveID` / `solveIDGRF` and existing `MuscleSolver.solveReal`. No new ObjC++ wrappers needed.

**Dependencies:** no new third-party libs. `.trc` is plain text (TRC ASCII format) — parser is hand-rolled like the existing `MOTParser`.

**Build system:** no `project.yml` changes beyond adding the new Swift files to the existing `BioMotion` target (XcodeGen picks them up by glob).
