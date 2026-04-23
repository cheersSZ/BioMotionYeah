## ADDED Requirements

### Requirement: Single-screen playback layout

The Offline tab SHALL render the playback video, transport controls, and the per-frame analysis output (muscle activations, IK joint angles + torques, and the six analysis badges: SO convergence, |τ|, |τ|/m, root residual, left foot force, right foot force) on a single iPhone screen without requiring vertical scrolling on iPhone 14, 15, and 16 (Pro and non-Pro) models.

#### Scenario: All scrubbable surfaces visible after loading a session

- **WHEN** the user has imported a folder containing a model, motion, video, and keypoints, AND has run "Process All Frames"
- **THEN** the playback video, the IK overlay, the muscle activation overlay, the slider, the transport buttons (◀ ⏸ ▶▶ Reset), and all six analysis badges are simultaneously visible without scrolling on iPhone 14 / 15 / 16 (Pro and non-Pro)

#### Scenario: Stepping a frame updates every visible surface

- **WHEN** the user taps the ◀ or ▶ transport button while a processed session is loaded
- **THEN** the video frame, pose overlay, IK overlay values, muscle activation overlay values, slider position, frame counter, and all six analysis badges all update without the user needing to scroll or switch tabs

#### Scenario: Smaller screens fall back to scrolling

- **WHEN** the device's available content height is less than 600 points (e.g., iPhone SE 1st gen)
- **THEN** the layout is wrapped in a `ScrollView` and the user can scroll to reach all controls without losing access to any functionality

### Requirement: Floating analysis overlays on the video player

The playback video card SHALL render the IK joint-angle readout panel as a translucent overlay anchored to the top edge of the video and the muscle activation bar as a translucent overlay anchored to the bottom edge of the video. The overlays SHALL reuse the existing `MuscleActivationBar` and `IKReadoutPanel` views without modification.

#### Scenario: Overlays render on top of the video and pose

- **WHEN** the playback video is visible AND the current analysis sample is non-nil
- **THEN** the IK readout panel is rendered with `.topLeading` alignment inside the video bounds with 8 points of padding from the video edges
- **AND** the muscle activation bar is rendered with `.bottomLeading` alignment inside the video bounds with 8 points of padding from the video edges
- **AND** both overlays render above the `PoseOverlayView` skeleton

#### Scenario: Overlays hidden when no analysis exists yet

- **WHEN** a video is loaded AND the user has not yet run "Process All Frames" (`currentAnalysisSample` is nil)
- **THEN** neither the IK overlay nor the muscle activation overlay is rendered
- **AND** the video and pose overlay are still visible

### Requirement: Overlay-visibility toggle

The video card SHALL provide a single tap target in its top-trailing corner that toggles the visibility of both analysis overlays. The toggle SHALL default to overlays-visible on every appearance of the view and SHALL NOT persist its state across launches or tab switches.

#### Scenario: Toggle hides both overlays

- **WHEN** both overlays are visible AND the user taps the visibility toggle button
- **THEN** both the IK overlay and the muscle activation overlay fade to fully transparent within approximately 0.2 seconds
- **AND** the toggle button's SF Symbol changes from `eye` to `eye.slash`
- **AND** the pose overlay (`PoseOverlayView`) remains visible

#### Scenario: Toggle defaults to visible on view appearance

- **WHEN** the user navigates to the Offline tab
- **THEN** both overlays are visible by default regardless of the previous state of the toggle in any prior session

### Requirement: Compact six-badge analysis grid

Below the transport row the view SHALL render the six per-frame analysis badges in a 2-row, 3-column grid with equal-width columns. The order SHALL be row 1: `SO`, `|τ|`, `|τ|/m`; row 2: `Root residual`, `L Foot`, `R Foot`. Frame number and time SHALL NOT appear in the badge grid (they are already shown in the slider counter row).

#### Scenario: All six badges visible simultaneously

- **WHEN** a processed analysis sample is loaded for the current frame
- **THEN** the SO convergence, max |τ|, max |τ|/mass, root residual, left-foot force, and right-foot force badges all render in a single 2×3 grid
- **AND** all three columns in each row have equal width

#### Scenario: Empty grid before processing

- **WHEN** the session has frames loaded but `currentAnalysisSample` is nil
- **THEN** the badge grid region displays a single inline message instructing the user to run "Process All Frames" via the Batch sheet
- **AND** the layout dimensions of the badge grid region remain stable so that filling in values after processing does not shift the position of other elements

### Requirement: Source sheet for import and motion summary

The view SHALL provide a toolbar button in the top-trailing position of the navigation bar that opens a modal sheet containing the OpenCap folder importer, the imported file metadata (model, motion, video, keypoints), the import status / warning / error messages, and the prepared motion summary (frame count, duration, FPS). The button SHALL be icon-only using the SF Symbol `tray.and.arrow.down` with an accessibility label.

#### Scenario: Source sheet opens from toolbar

- **WHEN** the user taps the `tray.and.arrow.down` toolbar button
- **THEN** a modal sheet presents the OpenCap import controls, file metadata rows, status messages, and the prepared motion summary badges (frame count, duration, FPS)
- **AND** the sheet supports both `.medium` and `.large` presentation detents

#### Scenario: Source button has accessibility label

- **WHEN** VoiceOver is active AND the user focuses the source toolbar button
- **THEN** VoiceOver announces an accessibility label describing the button's purpose (e.g., "Source files")

### Requirement: Source sheet auto-opens on first launch when empty

The view SHALL automatically present the Source sheet exactly once across the lifetime of an installation, on the first appearance of the Offline tab, and only when no model and no video have been imported yet. The auto-open behavior SHALL NOT recur on subsequent appearances regardless of whether the user dismissed the sheet without importing anything.

#### Scenario: First launch with no data triggers auto-open

- **WHEN** the user opens the Offline tab for the first time after install AND no folder has been imported (`session.modelURL` and `session.videoURL` are both nil)
- **THEN** the Source sheet is presented automatically
- **AND** an `@AppStorage` flag (`offline.hasSeenSourceSheet`) is set to true so the auto-open does not recur

#### Scenario: Subsequent launches do not auto-open

- **WHEN** the user opens the Offline tab after the auto-open has already fired once
- **THEN** the Source sheet is not presented automatically, regardless of whether any folder is currently imported

### Requirement: Batch sheet for batch processing and export

The view SHALL provide a second toolbar button in the top-trailing position of the navigation bar that opens a modal sheet containing the "Process All Frames" control, the processing progress indicator, the post-processing summary badges (max |τ|/m, max root residual, total mass, SO converged count), the "Export" control, and the last-export folder reference. Triggering the share flow from inside this sheet SHALL present the existing `ShareSheet`. The button SHALL be icon-only using the SF Symbol `square.stack.3d.up` with an accessibility label.

#### Scenario: Batch sheet opens from toolbar

- **WHEN** the user taps the `square.stack.3d.up` toolbar button
- **THEN** a modal sheet presents the Process All Frames button, progress indicator (when processing), result summary badges, Export button, and the last-export-folder reference
- **AND** the sheet supports both `.medium` and `.large` presentation detents

#### Scenario: Export from batch sheet shows share sheet

- **WHEN** the user taps "Export" inside the Batch sheet AND export succeeds
- **THEN** the system share sheet is presented containing the kinematics .mot, inverse-dynamics .sto, summary .json, and any activations / muscle-forces .sto files

### Requirement: Pipeline and session state are not modified

The redesign SHALL NOT alter any code in `BioMotion/Nimble/`, `BioMotion/Muscle/`, `BioMotion/Offline/OfflineSession.swift`, `BioMotion/Offline/OfflineAnalysisExporter.swift`, the `MuscleActivationBar` view, or the `IKReadoutPanel` view. The redesign SHALL NOT introduce any new published state on `OfflineSession`, any new background work, or any change to per-frame compute timing.

#### Scenario: No bridge or pipeline changes

- **WHEN** the change is implemented
- **THEN** no source file under `BioMotion/Nimble/`, `BioMotion/Muscle/`, or `BioMotion/Offline/` has been modified
- **AND** no new property is added to `OfflineSession`
- **AND** the per-frame IK / ID / SO compute path is unchanged

#### Scenario: Reused overlay components are not modified

- **WHEN** the change is implemented
- **THEN** the `MuscleActivationBar` and `IKReadoutPanel` view definitions in `BioMotion/App/ContentView.swift` are unchanged
