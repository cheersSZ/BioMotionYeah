## 1. Engine API: source-aware model loading

- [ ] 1.1 Add `enum ModelSource { case genericBundled, prescaledImported }` in `BioMotion/Nimble/NimbleEngine.swift`
- [ ] 1.2 Add `@Published private(set) var modelSource: ModelSource = .genericBundled` to `NimbleEngine`
- [ ] 1.3 Add `loadModel(fromPath:source:completion:)` overload that sets `modelSource` on the main actor after the bridge load succeeds
- [ ] 1.4 Make the existing `loadModel(fromPath:completion:)` call the new overload with `source: .genericBundled` (zero behavior change for current callers)
- [ ] 1.5 Verify `OfflineSession.importModel` still compiles and behaves identically (it keeps using the legacy overload)

## 2. Profile model + persistence

- [ ] 2.1 Create `BioMotion/App/SubjectProfile.swift` with `struct SubjectProfile: Codable, Identifiable { id: UUID; name: String; modelSource: NimbleEngine.ModelSource; osimRelativePath: String; createdAt: Date }`
- [ ] 2.2 Add `Codable` conformance for `NimbleEngine.ModelSource` (raw-string `genericBundled` / `prescaledImported`)
- [ ] 2.3 Create `BioMotion/App/SubjectProfileStore.swift` as `@MainActor final class SubjectProfileStore: ObservableObject` with `@Published profiles: [SubjectProfile]` and `@Published activeProfileID: UUID?`
- [ ] 2.4 Implement `SubjectProfileStore.profilesURL` → `Documents/Profiles/profiles.json` and `activeIDURL` → `Documents/Profiles/activeProfileID`
- [ ] 2.5 Implement `loadFromDisk()`: schema-versioned JSON read, atomic, defensive against corruption (logs to `os_log`, returns empty list on parse failure)
- [ ] 2.6 Implement `saveToDisk()`: atomic write with `Data.write(to:options:.atomic)`; called after every mutation
- [ ] 2.7 Implement `absoluteOsimURL(for: SubjectProfile) -> URL` that joins the sandbox `Documents/Profiles/` base with the profile's relative path (must reconstruct on every launch — sandbox path can change)
- [ ] 2.8 Implement `setActive(_ profile: SubjectProfile?)` that updates `activeProfileID`, persists to disk, and posts a notification or `objectWillChange`

## 3. Profile import flow

- [ ] 3.1 Implement `SubjectProfileStore.importProfile(name:from sourceURL:into engine:) async throws -> SubjectProfile`
- [ ] 3.2 Inside import: open security-scoped resource, create `Documents/Profiles/<newID>/`, copy `.osim` to `model.osim`, release scope
- [ ] 3.3 Inside import: validate by attempting `engine.loadModel(fromPath: copiedPath, source: .prescaledImported)` and checking `engine.muscleSolver.numMuscles > 0` after the load completes
- [ ] 3.4 If muscle count is 0 or load fails: delete the just-created profile folder, throw a `ProfileImportError` describing the failure, do not append to `profiles`
- [ ] 3.5 If validation passes: append to `profiles`, persist, return the new `SubjectProfile`
- [ ] 3.6 Implement `SubjectProfileStore.deleteProfile(_ profile:)` that removes the folder, drops the entry, and clears `activeProfileID` if it matched

## 4. App-start integration

- [ ] 4.1 Wire `@StateObject private var profileStore = SubjectProfileStore()` into the app's root view (likely `BioMotionApp` or `ContentView`)
- [ ] 4.2 On first appear, call `profileStore.loadFromDisk()` then resolve the active profile and load its model
- [ ] 4.3 If active profile resolution fails (missing file, broken JSON entry): clear `activeProfileID`, set a banner-warning flag, and load the bundled `Rajagopal2016.osim` with `.genericBundled`
- [ ] 4.4 If no active profile: load the bundled model with `.genericBundled` (current code path, kept as the default)
- [ ] 4.5 Confirm the load happens before any view binds to `nimble.totalMassKg` so the displayed mass is correct on first paint

## 5. Calibration mode-awareness

- [ ] 5.1 In `BioMotion/App/CalibrationView.processCalibration()`, branch on `nimble.modelSource`
- [ ] 5.2 For `.genericBundled`: keep the existing `nimble.scaleModel(height:markerPositions:markerNames:)` call exactly as today
- [ ] 5.3 For `.prescaledImported`: skip `scaleModel(...)`, but still let the capture phase run so 1-euro filters warm up
- [ ] 5.4 Update the on-screen calibration label to say "Calibrating ground plane…" when `modelSource == .prescaledImported`, otherwise keep "Estimating your height…"
- [ ] 5.5 Verify ground-plane calibration (auto-calibrated from running min of ankle Y inside `NimbleBridge`) still triggers in both modes — no Swift-side changes needed if it's already automatic

## 6. UI: profile management surface

- [ ] 6.1 Create `BioMotion/App/SubjectProfileView.swift`: a SwiftUI list of profiles showing name, source badge, total mass (from a one-off `loadModel` probe or stored on the profile), import date
- [ ] 6.2 Add an "Add profile" button that presents a name field and a `.fileImporter` configured for `.osim` (reuse the UTType used by `OfflineSession`)
- [ ] 6.3 Add per-row "Make active" and "Delete" actions that call into `SubjectProfileStore`
- [ ] 6.4 Show an inline error if import fails (no muscles, copy failed, etc.)
- [ ] 6.5 Add a navigation entry from `ContentView` (settings cog or list row) that pushes/sheets `SubjectProfileView`

## 7. UI: active model badge in live view

- [ ] 7.1 In `BioMotion/App/ContentView.swift`, near the existing `nimble.totalMassKg` badge, add a small "Profile: <name> (<source>)" chip
- [ ] 7.2 When no profile is active, show "Profile: Default (generic)"
- [ ] 7.3 Bind the chip to `profileStore.activeProfile` and `nimble.modelSource` (single source of truth)

## 8. Project + docs

- [ ] 8.1 Add the new `.swift` files to `project.yml` source paths if they are not auto-globbed; run `xcodegen generate`
- [ ] 8.2 Bump `CURRENT_PROJECT_VERSION` in `project.yml`
- [ ] 8.3 Update `CLAUDE.md` to note: "Online pipeline supports a `prescaledImported` profile source that skips `scaleModelWithHeight:` at calibration"
- [ ] 8.4 Add a "Subject profiles" subsection to `docs/UI_PARAMETERS.md` describing the profile chip and what generic vs scaled means for downstream numbers
- [ ] 8.5 Add a brief "Bring your own scaled model" section to `README.md` pointing to OpenCap as one production source

## 9. Verification

- [ ] 9.1 Sanity build: `xcodegen generate && xcodebuild ... -destination 'generic/platform=iOS Simulator' build` (or device build per `README.md`)
- [ ] 9.2 Default-path smoke test: fresh install → bundled model loads → calibration runs `scaleModel` → mass badge shows scaled-from-generic value
- [ ] 9.3 Import-path smoke test: import `Resources/trail2/LaiUhlrich2022_scaled.osim` as a profile → make active → relaunch → mass badge shows the imported model's exact mass → calibration runs and does NOT call `scaleModel` (verified via log)
- [ ] 9.4 Failure-path test: delete the active profile's `model.osim` from disk via the simulator's Files.app → relaunch → app falls back to bundled with a warning banner
- [ ] 9.5 Reject-path test: try to import an `.osim` with no muscles (manually strip `<Muscle>` blocks from a copy) → import is rejected, no folder is created
- [ ] 9.6 Per-frame budget check: confirm IK+ID+SO timing on the live HUD is unchanged (~2 ms) before/after a prescaled load
