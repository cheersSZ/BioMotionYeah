## Context

The online ARKit pipeline today is hardwired around `Resources/Rajagopal2016.osim` plus a calibration-time `scaleModelWithHeight:` call that derives per-segment lengths from ARKit joint distances:

- `BioMotion/Nimble/NimbleEngine.swift` (`loadModel(fromPath:)`) — drives `NimbleBridge.loadModel`, `MuscleSolver.loadMuscles`, and `MomentArmComputer.parseMusclePaths` on the engine's serial `solverQueue`.
- `BioMotion/App/CalibrationView.swift` (`processCalibration`) — averages a few seconds of T-pose joint positions and unconditionally calls `nimble.scaleModel(height:markerPositions:markerNames:)`.
- `BioMotion/Nimble/NimbleBridge.mm` (`scaleModelWithHeight:`) — produces a `bodyScales` vector and feeds it to Nimble's `setBodyScales`, which propagates to mass/inertia.

Mass for the realtime path is therefore always derived (segment scale³ from a generic 75 kg model). Two facts make pre-scaled models trivially compatible:

1. **Body naming is shared.** `LaiUhlrich2022_scaled.osim` (the OpenCap model used in `Resources/trail2/`) has the same body names that `NimbleBridge.mm` uses in its `virtualMarkers[]` table (`pelvis`, `femur_r/l`, `tibia_r/l`, `talus_r/l`, `toes_r/l`, `torso`, `humerus_r/l`, `ulna_r/l`, `hand_r/l`). The cyclist-spine fallback rows in the same table cover models without a detailed spine.
2. **The offline path already proves it works.** `OfflineSession.importModel` calls `nimble.loadModel(fromPath:)` and never touches `scaleModelWithHeight:`. The same input file works there end-to-end (IK → ID → SO → OSQP), so the C++ side is already correct for pre-scaled inputs. The only blocker is the Swift-side calibration that always rescales.

## Goals / Non-Goals

**Goals:**
- Online pipeline can be backed by either the bundled generic model (current behavior, default) or a user-imported `*_scaled.osim`.
- The choice is persistent across launches via a "subject profile" that owns the .osim file and its source mode.
- Calibration is mode-aware: ground-plane calibration always runs; anthropometric scaling runs only for `generic-bundled` profiles.
- No regression to the default path. With no profile selected, the app boots to the bundled Rajagopal2016 + scaling-at-calibration path that ships today.
- No change to the per-frame hot path budget (~2 ms IK+ID+SO). All new work is at app start, model load, and calibration — none of which run per-frame.

**Non-Goals:**
- No changes to `NimbleBridge`, `MuscleSolver`, or `MomentArmComputer` C++/ObjC++ code.
- No new nimblephysics patches; no new `DART_IOS_BUILD` guards.
- No new third-party dependencies; OSQP / Eigen / tinyxml2 unchanged.
- No in-app scaling tool (no OpenSim ScaleTool port, no automatic OpenCap session orchestration).
- No iCloud sync or multi-device profile sharing.
- No support for swapping profiles mid-session — switching profile requires reloading the model and re-running calibration.
- No new project targets, no SDK-conditional library paths.

## Decisions

### D1 — Profile is owned by Swift, not by C++

A `SubjectProfile` is a pure Swift value (`id: UUID`, `name: String`, `modelSource: ModelSource`, `osimRelativePath: String`, `createdAt: Date`). `SubjectProfileStore` is an `@MainActor` `ObservableObject` that persists profiles to `Documents/Profiles/profiles.json` and copies imported `.osim` files into `Documents/Profiles/<id>/model.osim`.

**Why**: keeps the C++ bridge surface untouched. `NimbleBridge.loadModel:` only ever sees a sandbox path string, exactly like today. All policy lives in Swift where it can be evolved without rebuilding the Nimble vendor tree.

**Alternative considered**: extend `NimbleBridge` with a `loadModelPreScaled:` method or a new property. Rejected — the C++ side has no need to know about the source; it just consumes the .osim. Adding a flag there would create coupling without benefit.

### D2 — Calibration mode signal: a `ModelSource` published by `NimbleEngine`

`NimbleEngine` gains:

```swift
enum ModelSource { case genericBundled, prescaledImported }
@Published private(set) var modelSource: ModelSource = .genericBundled
```

set inside `loadModel(fromPath:source:completion:)` on the main actor after the bridge load succeeds. `CalibrationView.processCalibration()` reads `nimble.modelSource` and skips `nimble.scaleModel(...)` when it is `.prescaledImported`.

**Why**: a single source of truth that any downstream view or exporter can consult ("is this a personalized model?"). Published so SwiftUI can show a badge.

**Alternative considered**: pass a `Bool isPreScaled` through the calibration entry point. Rejected — the calibration view already has the engine reference; threading a bool through every call site is noisier than reading one published property.

### D3 — `loadModel` becomes source-aware; no behavioral change for existing callers

Add an overload:

```swift
func loadModel(fromPath path: String,
               source: ModelSource,
               completion: ((Bool) -> Void)? = nil)
```

The existing `loadModel(fromPath:completion:)` becomes a thin wrapper that passes `.genericBundled` (the only thing it ever used to be). All current call sites (`OfflineSession.importModel`, app-start bundle load) continue to work unchanged.

**Why**: zero source-churn for callers that don't care, explicit opt-in for the new path.

**Alternative considered**: silently infer `source` from the file path (e.g. `"_scaled.osim"` substring). Rejected — fragile, file naming is not under our control, and the user's intent (generic vs personalized) is policy that should be explicit.

### D4 — Offline path: still pass `.genericBundled` (don't infer)

`OfflineSession` keeps calling the legacy `loadModel(fromPath:)` overload. The offline pipeline has its own UI for ID/SO display and does not need calibration mode logic — it never calls `scaleModelWithHeight:` regardless. Surfacing `modelSource: .genericBundled` for offline-imported `*_scaled.osim` is technically wrong but inconsequential (no calibration step in offline). We accept the small lie because:

- The offline UI doesn't render `modelSource`.
- Mixing "what this enum means for online vs offline" creates real confusion.
- A future cleanup can introduce `.prescaledImportedOffline` if the offline UI ever needs to disambiguate.

**Why**: keep the change tight; offline is not in scope.

### D5 — Profile storage layout

```
Documents/
  Profiles/
    profiles.json                 # array of SubjectProfile
    activeProfileID                # tiny file with the active profile's UUID, or absent
    <uuid>/
      model.osim                  # imported file (canonical name, regardless of source filename)
```

`profiles.json` is a `JSONEncoder`-produced array. The `osimRelativePath` field stores `"<uuid>/model.osim"` so absolute paths can be reconstructed without persisting them (sandbox path changes between launches under TestFlight).

**Why**: small, obvious, no `Codable` migration needed when adding new fields later (use `decodeIfPresent` for additions). Atomic writes via `Data.write(to:options:.atomic)`.

**Alternative considered**: Core Data / SwiftData. Rejected — overkill for ≤10 profiles, no relational queries, and pulls in a runtime dependency the app otherwise doesn't need.

### D6 — App-start load order

`ContentView.onAppear` (or app-level `@main` init) does, in order:

1. `SubjectProfileStore.loadFromDisk()` (sync, tiny).
2. If `activeProfileID` resolves to an imported `.osim` that exists on disk: `nimble.loadModel(fromPath: profile.absoluteOsimURL.path, source: .prescaledImported)`.
3. Otherwise: load bundled `Rajagopal2016.osim` with `source: .genericBundled` (existing behavior).

If the imported file is missing (user deleted, restore-from-backup edge case), the store marks the profile as broken, falls back to bundled, and surfaces a warning banner.

**Why**: deterministic, no race with `solverQueue`'s async load; the engine's existing `solverQueue.async` inside `loadModel` already serializes the work.

### D7 — UI placement

`SubjectProfileView` is reachable from a settings entry on `ContentView`. Initial UI is intentionally minimal:

- List of profiles with name, source, total mass (read after load), date imported.
- "Add profile" → name field + document picker (UTType for `.osim` already used by `OfflineSession`).
- "Make active" → switches `activeProfileID`, triggers reload.
- "Delete" → removes folder + profile entry; if it was active, falls back to bundled.

A persistent "Profile: <name> (scaled)" or "Profile: Default (generic)" badge is added near the existing `totalMassKg` display in `ContentView`'s top stats so users can verify which model they're running on.

**Why**: keep the UI surface small for v1; profile management is not the value proposition. The value is the per-subject accuracy.

### D8 — Calibration UX for pre-scaled mode

When `modelSource == .prescaledImported`, `CalibrationView` still runs its capture phase but only to:

1. Establish `_groundHeightY` (auto-calibrated from the running min of ankle Y; existing logic).
2. Warm up the 1-euro filters with realistic input.

The scaling-related UI ("Estimating your height…") is replaced by "Calibrating ground plane…". Captured height is still displayed for sanity-checking but is not fed to `scaleModel`.

**Why**: avoids surprising users with a missing calibration step; keeps a consistent flow.

## Risks / Trade-offs

- **Risk**: muscle set in user-imported models may differ from Rajagopal2016 → `MuscleOverlay`'s 28 hard-coded muscle names render incompletely.
  → **Mitigation**: document this in `docs/UI_PARAMETERS.md` and on the import dialog. Fall back gracefully — `MuscleOverlay` already skips muscles it can't resolve. Filing a follow-up to make the overlay set data-driven is out of scope here.

- **Risk**: per-segment body scales in the imported model are anisotropic (e.g. `1.0377 1.0377 1.1983`); some downstream code may assume isotropic scaling.
  → **Mitigation**: `MomentArmComputer.h` already documents that it relies on `scaleModelWithHeight:` having run; for pre-scaled models the model is already in the same post-scaling state, so the moment-arm numerical-diff behavior is identical. Verified by the offline path already producing sensible muscle results on `LaiUhlrich2022_scaled.osim`.

- **Risk**: user imports a non-musculoskeletal `.osim` (no muscles, only joints) → `MuscleSolver.loadMuscles` returns 0 muscles, OSQP layer is a no-op, UI shows blank muscle stats.
  → **Mitigation**: import flow rejects models with `numMuscles == 0` and shows a clear error. IK + ID still work, but the app's value depends on muscles, so we fail loudly.

- **Risk**: profile JSON corruption (interrupted write, manual edit) → app refuses to start.
  → **Mitigation**: atomic writes, schema-versioned JSON (`{"schemaVersion": 1, "profiles": [...]}`), defensive load that drops unparseable entries and logs them to `os_log`. Bundled-generic fallback is always reachable.

- **Trade-off**: offline path keeps reporting `.genericBundled` even when consuming a `*_scaled.osim`. Documented in D4. If offline UI ever needs to differentiate, refactor at that point — premature now.

- **Trade-off**: profile storage in `Documents/` makes models visible in Files.app and included in iCloud Backup. We accept this — it's the correct location for user-owned data and matches App Store guidance. If model files become large enough to warrant exclusion, set `URLResourceKey.isExcludedFromBackupKey` later.

- **Pipeline impact**: zero per-frame impact. The new code paths are app-launch (~1 ms JSON read), model-load (already async on `solverQueue`), and calibration (already async). No changes to IK/ID/SO/OSQP/RealityKit stages.

## Migration Plan

This is additive; no migration of existing on-disk state is needed (the feature did not exist before). First-run users see the bundled-default behavior unchanged. Users who want personalized models go to Settings → Profiles → Import.

## Open Questions

- Should we ship the `LaiUhlrich2022_scaled.osim` from `Resources/trail2/` as a built-in "demo profile" so testers can try the feature without an OpenCap account? (Leaning: yes for TestFlight builds, gated by a debug flag for App Store builds.) — **defer to implementation**.
- Do we need a checksum on the imported `.osim` to detect tampering between launches? (Leaning: no for v1; users own these files.) — **defer**.
