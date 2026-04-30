## ADDED Requirements

### Requirement: IDMode selector on NimbleEngine

`NimbleEngine` SHALL expose an `IDMode` enum with cases `.withGRF` and `.noGRF`, and a mutable `idMode: IDMode` property defaulting to `.withGRF`. The property SHALL be readable and writable from Swift call sites without restarting the engine. Switching `idMode` SHALL take effect on the next call into `runDynamicsAndMuscles` (no buffered frames re-processed).

#### Scenario: Default mode preserves production behavior

- **GIVEN** a freshly constructed `NimbleEngine`
- **WHEN** `engine.idMode` is read
- **THEN** it SHALL equal `.withGRF`
- **AND** `runDynamicsAndMuscles` SHALL invoke `bridge.solveIDGRF` (matching legacy behavior)

#### Scenario: Switching to noGRF in offline mode

- **GIVEN** an `OfflineSession` mid-trial with `idMode = .noGRF` set before `processAllFrames`
- **WHEN** the engine processes each frame
- **THEN** `bridge.solveID` SHALL be invoked instead of `bridge.solveIDGRF`
- **AND** the resulting `IDOutput.contactLeft`, `contactRight`, `grfLeft`, `grfRight`, `copLeft`, `copRight`, `rootResidualNorm` fields SHALL all be zero/false (no GRF estimation performed)

### Requirement: Offline-only UI toggle

`OfflineAnalysisView` SHALL render a "No-GRF mode (validation)" toggle bound to `OfflineSession.idMode`. The toggle SHALL be visible only in offline analysis flows and SHALL NOT appear in the live ARKit capture view. Toggling it SHALL only affect subsequent re-runs of an offline trial; in-flight frames SHALL not be retroactively re-solved.

#### Scenario: Toggle visible in offline view

- **WHEN** the user opens `OfflineAnalysisView`
- **THEN** a labeled toggle "No-GRF mode (validation)" SHALL be present
- **AND** its initial state SHALL reflect `OfflineSession.idMode == .noGRF`

#### Scenario: Toggle absent from live capture

- **WHEN** the user opens the live ARKit capture view
- **THEN** no "No-GRF mode" toggle SHALL be present
- **AND** the live pipeline SHALL always run with `idMode = .withGRF`

### Requirement: Export records the active IDMode

`OfflineAnalysisTrack` SHALL carry an `idMode: String` field whose value is `"withGRF"` or `"noGRF"` matching the `NimbleEngine.IDMode` used to produce the track. `OfflineAnalysisExporter` SHALL write this field into both the `<stem>_summary.json` and the persisted `<stem>_processed.json`. The decoder SHALL tolerate older blobs that lack the field, defaulting them to `"withGRF"`.

#### Scenario: noGRF run is tagged in the summary

- **GIVEN** an offline trial processed with `idMode = .noGRF`
- **WHEN** the exporter writes `<stem>_summary.json`
- **THEN** the JSON SHALL contain `"idMode": "noGRF"` at the top level

#### Scenario: Legacy blob decode fallback

- **GIVEN** a `<stem>_processed.json` produced before this change (no `idMode` key)
- **WHEN** the file is loaded into `OfflineAnalysisTrack`
- **THEN** the decoder SHALL not throw
- **AND** `track.idMode` SHALL equal `"withGRF"`
