## ADDED Requirements

### Requirement: `NimbleEngine` SHALL expose a configurable inverse-dynamics mode

`NimbleEngine` SHALL expose an `idMode` property of type `IDMode` with cases `.withGRF` and `.noGRF`. The default value SHALL be `.withGRF` so the production live-ARKit and existing offline `.mot` flows are unchanged. When `idMode == .noGRF`, the engine SHALL invoke `NimbleBridge.solveID` (no ground-reaction-force estimation) instead of `NimbleBridge.solveIDGRF`.

#### Scenario: Default mode preserves existing behavior

- **WHEN** an existing call site (live ARKit or `.mot` offline) creates a `NimbleEngine` and runs the pipeline without setting `idMode`
- **THEN** the engine calls `solveIDGRF` for every frame and produces the same `IDOutput` (with GRF / CoP / contact fields populated) as before this change

#### Scenario: No-GRF mode bypasses GRF estimation

- **WHEN** `idMode` is set to `.noGRF` before `processAllFrames` is called
- **THEN** every frame's dynamics step calls `solveID` (no GRF), and the resulting `IDOutput` has `leftFootForce`, `rightFootForce`, `leftFootCoP`, `rightFootCoP` all `.zero`, and `leftFootInContact == false`, `rightFootInContact == false`

#### Scenario: Mode toggling between batch runs

- **WHEN** the user toggles the no-GRF flag in the offline UI and re-runs `processAllFrames`
- **THEN** the next batch run uses the new mode and produces a fresh `OfflineAnalysisTrack` whose `idMode` field reflects the active mode

### Requirement: Offline UI SHALL expose the no-GRF flag for validation runs

The offline analysis view SHALL provide a user-visible toggle, labelled "No-GRF mode (validation)", that sets `OfflineSession.idMode` and is propagated to `NimbleEngine.idMode` before the next `processAllFrames` invocation. The toggle SHALL default to off.

#### Scenario: Toggle defaults to off

- **WHEN** the offline view is presented
- **THEN** the no-GRF toggle is off and `NimbleEngine.idMode == .withGRF`

#### Scenario: Toggle is propagated before batch run

- **WHEN** the user enables the no-GRF toggle and taps "Process All Frames"
- **THEN** `NimbleEngine.idMode` is `.noGRF` for the duration of the batch run

### Requirement: Batch results SHALL record the active ID mode and input type

`OfflineAnalysisTrack` SHALL include `idMode: String` and `inputType: String` fields, persisted by `OfflineAnalysisExporter` into the `*_processed.json` summary. Old exports lacking these fields SHALL load with defaults `idMode = "withGRF"` and `inputType = "mot"`.

#### Scenario: Export records mode and input type

- **WHEN** a batch run completes and the user exports
- **THEN** the `*_processed.json` contains top-level keys `"idMode"` and `"inputType"` with values matching the run that produced it

#### Scenario: Loading a pre-change export

- **WHEN** the user loads an `OfflineExport-*` folder produced before this change (no `idMode` / `inputType` keys)
- **THEN** the load succeeds and the rehydrated `OfflineAnalysisTrack` has `idMode == "withGRF"` and `inputType == "mot"`

### Requirement: Muscle solver behavior SHALL be unchanged in no-GRF mode

When `idMode == .noGRF`, the muscle solver SHALL still run on the resulting joint torques and SHALL NOT attempt to compensate for the missing GRF. Floating-root residuals are reported in the IDOutput's torque map exactly as `solveID` produces them; the muscle solver consumes the same actuated-DOF subset it consumes in `.withGRF` mode.

#### Scenario: Muscle solver runs on no-GRF torques

- **WHEN** `idMode == .noGRF` and a frame's IK result is available
- **THEN** `MuscleSolver.solveReal` is invoked with the same arguments shape as in `.withGRF` mode and produces a `MuscleOutput`
