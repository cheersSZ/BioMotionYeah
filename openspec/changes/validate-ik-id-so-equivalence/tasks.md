> **Superseded by `fix-bilateral-grf-bias`** (sections 2–4 of that change).
> The `IDMode` toggle, offline-session plumbing, UI toggle, and `idMode` tag
> in the export schema are all implemented there. `inputType` tagging is the
> only piece deferred — pick it up under task 2.x of this change when the
> `.trc` import path lands.

## 1. Stage ID — iOS no-GRF mode (smallest, isolates ID)

- [x] 1.1 Add `enum IDMode { case withGRF, noGRF }` and `var idMode: IDMode = .withGRF` to `BioMotion/Nimble/NimbleEngine.swift` *(done in `fix-bilateral-grf-bias`)*
- [x] 1.2 Branch `runDynamicsAndMuscles` on `idMode`: call `bridge.solveIDGRF` for `.withGRF`, `bridge.solveID` for `.noGRF`; in `.noGRF` zero out the GRF/CoP/contact fields on the resulting `IDOutput` *(done in `fix-bilateral-grf-bias`)*
- [x] 1.3 Add `var idMode: NimbleEngine.IDMode = .withGRF` to `OfflineSession`; propagate to `nimble.idMode` inside `processAllFrames` before dispatching frames *(done in `fix-bilateral-grf-bias`)*
- [x] 1.4 Add a "No-GRF mode (validation)" toggle to `BioMotion/App/OfflineAnalysisView.swift`, bound to `OfflineSession.idMode`, defaulting to off *(done in `fix-bilateral-grf-bias`)*
- [x] 1.5 Extend `OfflineAnalysisTrack` with `idMode: String` (default `"withGRF"`) ~~and `inputType: String` (default `"mot"`)~~ — `inputType` deferred to section 2 of this change *(done in `fix-bilateral-grf-bias`)*
- [x] 1.6 Update `OfflineAnalysisExporter` to write `idMode` ~~and `inputType`~~ into the summary JSON; update `loadProcessed` to parse with documented defaults when keys are absent *(done in `fix-bilateral-grf-bias`)*
- [ ] 1.7 Manual smoke: import an existing `.mot` trial, run with `idMode = .noGRF`, confirm `IDOutput.leftFootForce/rightFootForce` are zero and torques are non-zero on actuated DOFs *(falls under `fix-bilateral-grf-bias` section 8)*

## 2. Stage IK — `.trc` import path on iOS (largest Swift change)

- [ ] 2.1 Add `BioMotion/Offline/MarkerSeries.swift` defining `MarkerSeries { let markerNames: [String]; let frames: [(timestamp: TimeInterval, positions: [SIMD3<Double>])] }`
- [ ] 2.2 Add `BioMotion/Offline/TRCParser.swift` mirroring `MOTParser`: parse the 5-line TRC header, honor the `Units` row (`m`, `mm`, `meters`), throw on unknown units, return `MarkerSeries`
- [ ] 2.3 Add an alias table (`MarkerAlias.swift`) covering at least the OpenCap → OpenSim renames: `r.ASIS↔RASI`, `L.ASIS↔LASI`, `r.PSIS↔RPSI`, `L.PSIS↔LPSI`, plus the documented OpenCap full-body marker set
- [ ] 2.4 Add `enum MotionSource { case angles(ImportedMotionSeries); case markers(MarkerSeries) }` to `OfflineSession`; replace direct `importedMotion` reads with `motionSource`-driven branches; keep the existing `importedMotion` published for back-compat where the UI reads it
- [ ] 2.5 Add `OfflineSession.importMarkers(from:)` that copies the file to the sandbox, parses it into `MarkerSeries`, sets `motionSource = .markers(...)`, sets `lastSelectedImportName`, and clears `analysisTrack`
- [ ] 2.6 Emit marker-reconciliation warnings: compute the symmetric difference between `MarkerSeries.markerNames` (post-alias) and `bridge.markerNames`, surface up to 8 of each side via `warningMessage`
- [ ] 2.7 Extend `processAllFrames` to handle `motionSource == .markers(...)`: iterate marker frames, dispatch `nimble.processMarkerFrame(frame)` instead of `processImportedMotionFrame`
- [ ] 2.8 Add `NimbleEngine.processMarkerFrame(_ frame: MarkerFrame)` that maps marker positions+names through the alias table, calls `bridge.solveIK`, and feeds results into the existing `processRawKinematics` (no SG semantics changes)
- [ ] 2.9 Update `OfflineAnalysisExporter` to set `inputType = "trc"` when the source was markers
- [ ] 2.10 Add the `.trc` UTType to `BioMotion/App/OfflineAnalysisView.swift` file picker (alongside `.mot`); route the selection to `importMarkers` when extension is `.trc`
- [ ] 2.11 Manual smoke: import a small OpenCap TRC, run batch processing, confirm `analysisTrack.samples` is non-empty, IK error per frame is < 5 cm, and `inputType == "trc"` in the export

## 3. Desktop equivalence module

- [ ] 3.1 Create `desktop-pipeline/src/biomotion_desktop/equivalence.py` skeleton: `run_equivalence(desktop_dir, ios_dir, stages, options) -> EquivalenceReport` with `EquivalenceReport` dataclass holding `ik`, `id`, `so` blocks
- [ ] 3.2 Implement IK stage: load both sides' kinematics via `storage_io`, resample to desktop grid, per-DOF compute RMSE/max_abs after best-integer-sample shift in `[-100, +100] ms`, recover `phase_lag_ms`; gate on actuated DOFs with desktop range > 5°; pass = RMSE ≤ 2° AND R² ≥ 0.95
- [ ] 3.3 Implement ID stage: read `idMode` from iOS summary; if either side ran with GRF, set `id.status = "skipped_grf_mismatch"` and return; else mask the 6 floating-root DOFs into `id.root_residual`, compute per-DOF RMSE in Nm/kg-of-body-mass and R² on actuated DOFs, gate on peak |τ| > 0.5 Nm/kg; pass = RMSE ≤ 0.3 Nm/kg AND R² ≥ 0.90
- [ ] 3.4 Implement SO stage timing metrics: reuse Tier-3 onset/peak/offset code from `compare.py` (extract into shared helper); per-muscle pass = |diff| ≤ 50 ms when both sides have an onset event
- [ ] 3.5 Implement SO stage saturation-rate: per-muscle fraction of frames where `a ≥ 0.95` on each side; report delta; pass = |delta| ≤ 0.10
- [ ] 3.6 Implement SO stage co-contraction: accept `--antagonist-pairs "a:b,c:d"` on the CLI, compute Pearson on each side, report delta; pass = `min_pearson ≥ 0.7 AND |delta| ≤ 0.3`
- [ ] 3.7 Implement SO stage `Σa²` per cycle: detect gait cycles via vertical pelvis displacement zero-crossings (or, if absent, a single full-trial integration); pass = within ±25% relative
- [ ] 3.8 Wire `equivalence` subcommand into `desktop-pipeline/src/biomotion_desktop/cli.py`: flags `--desktop`, `--ios`, `--stages ik,id,so`, `--require-id`, `--antagonist-pairs`, `--report`; print the per-stage one-line summary; exit non-zero if any gated stage fails
- [ ] 3.9 Add unit tests in `desktop-pipeline/tests/test_equivalence.py`: synthetic shift-by-N test (IK), root-DOF-mask test (ID), saturation-rate edge cases (SO), CLI invocation through `subprocess` on a tiny fixture

## 4. Regression harness + docs

- [ ] 4.1 Pick a small public OpenCap trial (~5 s); commit `desktop-pipeline/fixtures/opencap/<trial>/{<trial>.osim, <trial>.trc, <trial>.mot}` and a `trial.yaml`
- [ ] 4.2 Capture iOS export fixtures by running the `.trc` and `.mot` paths against the same trial; commit under `desktop-pipeline/fixtures/ios/<trial>-trc/` and `<trial>-mot/`
- [ ] 4.3 Replace the placeholder `desktop-pipeline/scripts/validate_demo.sh` with: (a) `biomotion-desktop run --config fixtures/opencap/<trial>/trial.yaml --output build/desktop-<trial>` (b) `biomotion-desktop equivalence --desktop build/desktop-<trial> --ios fixtures/ios/<trial>-trc --stages ik` (c) same for the no-GRF `mot` fixture with `--stages id,so --antagonist-pairs ...`; exit non-zero on failure
- [ ] 4.4 Add `desktop-pipeline/docs/comparison_semantics.md` covering: what each stage compares, why metrics differ from `compare.py`, pass-threshold rationale, known sources of legitimate disagreement (filter group delay, GRF estimation, muscle-model differences); link to `OPEN_ISSUES.md`
- [ ] 4.5 Link `comparison_semantics.md` from `desktop-pipeline/README.md`
- [ ] 4.6 Add a CI step (or document the manual command in `README.md`) that runs `validate_demo.sh` on every PR

## 5. Verification

- [ ] 5.1 Run `xcodegen generate` after touching `project.yml` (only if new Swift files require explicit listing; XcodeGen globs should pick them up)
- [ ] 5.2 Build BioMotion for iOS Simulator and device; verify no regressions in the live ARKit path or the existing `.mot` offline path
- [ ] 5.3 Run `pytest desktop-pipeline/tests/` — all tests pass including the new equivalence tests
- [ ] 5.4 Run `desktop-pipeline/scripts/validate_demo.sh` end-to-end on the committed fixture; confirm IK passes, ID passes (no-GRF), SO per-metric pass counts match the documented expectations in `comparison_semantics.md`
- [ ] 5.5 Run `openspec validate validate-ik-id-so-equivalence --strict` and fix any issues
