## Why

On iPhone the **Offline** tab cannot show the playback video and the per-frame analysis (muscle activations, IK joint angles, joint torques, foot forces) on the same screen. The view is a single `ScrollView` of five vertically stacked cards (Import → Summary → Playback → Frame Inspector → Batch) totalling ~1370 pt of content against ~660 pt of usable height, so when the user steps frames with ◀ / ▶ they can either watch the video or read the data — never both. This defeats the purpose of frame-by-frame inspection: the whole reason to scrub a recorded session is to correlate what the body is doing visually with what the muscles and joints are doing numerically.

## What Changes

- **Overlay muscle activations and IK readouts directly on the video player** (mirroring the live ARKit capture screen in `ContentView.swift`). `MuscleActivationBar` floats at the bottom of the video, `IKReadoutPanel` floats at the top — both reuse the existing `.black.opacity(0.5)` styling those components already ship with.
- **Add an overlay-visibility toggle** (`eye` / `eye.slash`) in the top-right corner of the video, defaulting to overlays-on. State is view-local (no persistence).
- **Compress the Frame Inspector into a 2×3 equal-column badge grid** below the transport row: row 1 `SO ✓ | |τ| | |τ|/m`, row 2 `Root res | L Foot | R Foot`. Frame number and time are dropped from the inspector since they already appear next to the slider. All six analysis badges stay visible at all times (the user explicitly tracks all of them while scrubbing).
- **Move the OpenCap Import card and the Prepared Motion summary into a `Source` sheet** opened from a toolbar button. The sheet uses `.presentationDetents([.medium, .large])`. Since import is infrequent within a session, the toolbar button uses an SF Symbol only (`tray.and.arrow.down`) with an accessibility label.
- **Auto-open the Source sheet on first launch** when no folder has been imported yet (one-shot per fresh install, tracked by a `@AppStorage` flag). Subsequent launches do not auto-open.
- **Move the Batch Analysis card into a `Batch` sheet** opened from a second toolbar button (`square.stack.3d.up`). Same detent behavior. The export share sheet remains presented from this sheet.
- **Remove `ScrollView`** from the main Offline view body — the redesigned content is sized to fit on iPhone 14 / 15 / 16 (Pro and non-Pro) without scrolling. Smaller devices (iPhone SE) gracefully fall back to a scrolling layout via `ScrollView` only when the content overflows the available height.

### Non-goals

- **No changes to the analysis pipeline.** `OfflineSession`, `NimbleEngine`, `MuscleSolver`, and the IK/ID/SO compute path are untouched. This is a UI-only change.
- **No changes to the live ARKit capture screen** (`ContentView.swift`) — though the new playback layout matches its visual language.
- **No new export formats** or batch processing changes.
- **No iPad-specific layout** (split view / multi-column). Phone is the priority; iPad keeps the same layout in this change.
- **No keyboard / hardware-controller scrubbing shortcuts.**
- **No persistence of overlay-visibility state** across launches.

## Capabilities

### New Capabilities

- `offline-playback-ui`: Phone-friendly layout for the Offline tab — video with floating analysis overlays, transport controls, compact analysis badge grid, and demoted Source / Batch chrome behind toolbar sheets.

### Modified Capabilities

None — no existing specs in `openspec/specs/`.

## Impact

- **Affected source files** (Swift only):
  - `BioMotion/App/OfflineAnalysisView.swift` — substantial restructure: remove `importSection`, `summarySection`, `batchSection` from main body; replace `playbackSection` + `resultsSection` with the new overlay layout and compact badge grid; add `sourceSheet`, `batchSheet`, toolbar items, overlay-visibility state.
  - `BioMotion/App/ContentView.swift` — none required. `MuscleActivationBar` and `IKReadoutPanel` are reused as-is.
- **No bridge / C++ surface changes.** No `MuscleSolver`, `NimbleBridge`, or `MomentArmComputer` work.
- **No `project.yml` changes.** No new files, no new build settings, no `xcodegen generate` needed.
- **No new dependencies.** Only stock SwiftUI APIs (`sheet`, `toolbar`, `presentationDetents`, `@AppStorage`, `ZStack`, `overlay`).
- **Backward compatibility:** purely a UI change in one screen — no on-disk format, no exported file, no public API affected.
