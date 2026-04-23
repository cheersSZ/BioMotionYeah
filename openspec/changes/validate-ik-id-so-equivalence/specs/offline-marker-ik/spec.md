## ADDED Requirements

### Requirement: Offline mode SHALL accept `.trc` marker files as a motion source

The iOS app's offline mode SHALL allow the user to import an OpenSim-format `.trc` marker file as the motion source, in addition to the existing `.mot` joint-angle path. When a `.trc` is imported, the app SHALL run iOS Nimble IK per frame using the existing `NimbleBridge.solveIK` and produce an `OfflineAnalysisTrack` with the same shape as the `.mot` path.

#### Scenario: User imports a `.trc` via the file picker

- **WHEN** the user selects a `.trc` file in the offline view's file picker
- **THEN** the app parses the file into a `MarkerSeries`, copies it into the sandbox, sets `OfflineSession.motionSource` to `.markers(MarkerSeries)`, and shows `Markers loaded: <name>` in `statusMessage`

#### Scenario: User runs batch processing on imported markers

- **WHEN** the user taps "Process All Frames" with a model and a `.trc` loaded
- **THEN** for each marker frame the engine calls `NimbleBridge.solveIK`, feeds the resulting joint angles into the existing `processRawKinematics` path, and produces an `OfflineAnalysisTrack` whose `samples` carry IK / ID / SO results
- **AND** `OfflineAnalysisTrack.inputType` records `"trc"`

### Requirement: TRC parser SHALL handle units and report unit errors

The `.trc` parser SHALL inspect the `Units` row of the TRC header and convert marker coordinates to meters. Unknown units SHALL fail the parse with an actionable error message.

#### Scenario: TRC declares units in mm

- **WHEN** the parser sees `Units` = `mm`
- **THEN** it scales every marker XYZ value by 1e-3 before constructing `MarkerSeries`

#### Scenario: TRC declares units in m

- **WHEN** the parser sees `Units` = `m`
- **THEN** it leaves marker XYZ values unchanged

#### Scenario: TRC declares an unsupported unit

- **WHEN** the parser sees a `Units` value other than `m`, `mm`, or `meters`
- **THEN** it throws a parse error containing the offending unit string and the source URL, surfaced via `OfflineSession.errorMessage`

### Requirement: Marker import SHALL warn on name reconciliation gaps

When markers are imported, the app SHALL compare the TRC marker set against `NimbleBridge.markerNames` and surface, via `OfflineSession.warningMessage`, any markers present in only one of the two sets — applying a documented alias table for OpenCap-style names (e.g. `r.ASIS` ↔ `RASI`) before reporting.

#### Scenario: TRC contains markers absent from the model

- **WHEN** the TRC includes marker names that, after alias resolution, do not appear in `bridge.markerNames`
- **THEN** the warning lists at most the first 8 such names with the suffix " — these will be ignored by IK"

#### Scenario: Model has markers absent from the TRC

- **WHEN** the model declares markers that, after alias resolution, are not present in the TRC
- **THEN** the warning lists at most the first 8 such names with the suffix " — IK will run with reduced marker support"

#### Scenario: All markers reconcile cleanly

- **WHEN** every TRC marker resolves to a model marker (and vice versa) after aliasing
- **THEN** no marker-reconciliation warning is emitted

### Requirement: Marker IK frames SHALL share the existing solver queue and SG warm-up semantics

The marker-input path SHALL dispatch each frame's `solveIK` onto `NimbleEngine.solverQueue` and SHALL feed the resulting joint angles through `processRawKinematics`. The existing Savitzky–Golay warm-up behavior (~9 frames, 4-frame center delay) SHALL apply unchanged.

#### Scenario: Batch processing of fewer than 9 marker frames

- **WHEN** the user runs batch processing on a `.trc` containing fewer than 9 frames
- **THEN** `OfflineAnalysisTrack.samples` is empty and `statusMessage` reads "Batch run produced no frames — SG filter may not have warmed up. Need at least 9 input frames."

#### Scenario: Batch processing of marker frames produces a center-shifted track

- **WHEN** the user runs batch processing on a `.trc` with N ≥ 9 frames
- **THEN** `OfflineAnalysisTrack.samples` contains at most N − 8 entries whose timestamps are the SG-centered timestamps emitted by the per-DOF filters
