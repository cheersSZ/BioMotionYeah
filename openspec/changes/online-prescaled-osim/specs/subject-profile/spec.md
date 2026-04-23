## ADDED Requirements

### Requirement: Subject profile persists across app launches

The system SHALL maintain a list of subject profiles in `Documents/Profiles/profiles.json`, where each profile owns one musculoskeletal `.osim` file stored under `Documents/Profiles/<profileID>/model.osim` and a `modelSource` tag of either `genericBundled` or `prescaledImported`.

#### Scenario: Profile survives app relaunch
- **WHEN** the user creates a profile with a pre-scaled `.osim` and force-quits the app
- **THEN** on the next launch the profile appears in the profile list with the same name, source, and on-disk model file

#### Scenario: Profile JSON is corrupted
- **WHEN** `Documents/Profiles/profiles.json` cannot be parsed
- **THEN** the app SHALL log the error, continue with an empty profile list, and fall back to the bundled `Rajagopal2016.osim`

### Requirement: Active profile drives the online model load

The system SHALL load the active profile's `.osim` into the online ARKit pipeline at app start. If no profile is active, or the active profile's `.osim` file is missing, the system SHALL load the bundled `Rajagopal2016.osim` with `modelSource = .genericBundled`.

#### Scenario: Active prescaled profile is loaded at launch
- **WHEN** the app launches with an active profile whose `modelSource` is `prescaledImported` and whose `model.osim` exists
- **THEN** `NimbleEngine.modelSource` becomes `.prescaledImported` after model load completes
- **AND** `NimbleEngine.totalMassKg` reflects the imported model's `<mass>` sum (not the generic 75 kg)

#### Scenario: Active profile's file is missing
- **WHEN** the active profile's `model.osim` file no longer exists on disk
- **THEN** the system SHALL fall back to loading the bundled `Rajagopal2016.osim` with `modelSource = .genericBundled`
- **AND** SHALL surface a user-visible warning identifying the broken profile

#### Scenario: No profile selected
- **WHEN** the app launches with no active profile
- **THEN** the bundled `Rajagopal2016.osim` is loaded with `modelSource = .genericBundled` (current default behavior)

### Requirement: Calibration honors the active model source

The calibration step in the online pipeline SHALL skip the anthropometric scaling pass (`scaleModelWithHeight:`) when `NimbleEngine.modelSource` is `.prescaledImported`. Ground-plane calibration and motion-filter warm-up SHALL run regardless of source.

#### Scenario: Generic model gets full calibration
- **WHEN** calibration completes with `modelSource = .genericBundled`
- **THEN** the system invokes `nimble.scaleModel(height:markerPositions:markerNames:)` with the captured T-pose data
- **AND** invokes the existing ground-plane calibration

#### Scenario: Prescaled model skips anthropometric scaling
- **WHEN** calibration completes with `modelSource = .prescaledImported`
- **THEN** the system does NOT invoke `nimble.scaleModel(...)`
- **AND** invokes ground-plane calibration
- **AND** body masses, segment lengths, and inertia in the loaded skeleton remain exactly as imported

### Requirement: Profile import accepts only musculoskeletal `.osim` files

The profile import flow SHALL copy the user-selected `.osim` into the profile folder via security-scoped resource access, then verify that the loaded model exposes at least one muscle. If the model has zero muscles, the import SHALL fail with a clear error and the profile SHALL NOT be persisted.

#### Scenario: Import a valid OpenCap-scaled model
- **WHEN** the user picks a `*_scaled.osim` file with non-empty muscle set via the document picker
- **THEN** the file is copied to `Documents/Profiles/<newID>/model.osim`
- **AND** a new profile entry is added to `profiles.json` with `modelSource = .prescaledImported`
- **AND** the user is prompted whether to make it the active profile

#### Scenario: Import a model with no muscles
- **WHEN** the user picks an `.osim` whose `MuscleSolver.numMuscles` resolves to 0 after parsing
- **THEN** the import is rejected with an error message naming the problem
- **AND** no files are written to `Documents/Profiles/`
- **AND** the profile list is unchanged

### Requirement: Profile deletion releases storage and resets active state

Deleting a profile SHALL remove its `<profileID>/` directory and its entry from `profiles.json`. If the deleted profile was active, the system SHALL clear the active selection and reload the bundled `Rajagopal2016.osim`.

#### Scenario: Delete the active profile
- **WHEN** the user deletes the currently active profile
- **THEN** the profile's folder under `Documents/Profiles/` is removed
- **AND** `activeProfileID` is cleared
- **AND** the online pipeline reloads with the bundled generic model

#### Scenario: Delete an inactive profile
- **WHEN** the user deletes a non-active profile
- **THEN** the profile is removed from `profiles.json`
- **AND** the active profile and currently loaded model are unchanged

### Requirement: UI surfaces the active model source

The app SHALL display the active profile's name and source mode (e.g. "Profile: Default (generic)" or "Profile: Alex (scaled)") near the existing total-mass readout in the live view, so the user can verify which model is driving the pipeline at a glance.

#### Scenario: Generic model badge
- **WHEN** the bundled model is active
- **THEN** the live view displays a badge identifying the source as generic

#### Scenario: Prescaled model badge
- **WHEN** a `prescaledImported` profile is active
- **THEN** the live view displays the profile name and identifies the source as scaled
- **AND** the displayed mass equals the imported model's mass (within 0.1 kg)
