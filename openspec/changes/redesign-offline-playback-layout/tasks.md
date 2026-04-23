## 1. Restructure OfflineAnalysisView body

- [x] 1.1 Add view state: `@State private var showOverlays = true`, `@State private var isShowingSourceSheet = false`, `@State private var isShowingBatchSheet = false`, `@AppStorage("offline.hasSeenSourceSheet") private var hasSeenSourceSheet = false`
- [x] 1.2 Replace the top-level `ScrollView { VStack { ... } }` body with a `GeometryReader`-gated layout: render the new compact layout directly when `geo.size.height >= 600`, otherwise wrap in a `ScrollView`
- [x] 1.3 Remove `importSection`, `summarySection`, and `batchSection` from the main body (they will move into sheets in tasks 4 and 5)

## 2. Build the video card with floating overlays

- [x] 2.1 Wrap `PlayerLayerView` + `PoseOverlayView` in a `ZStack` and render `IKReadoutPanel` aligned `.topLeading` with 8 pt padding, gated on `showOverlays && session.currentAnalysisSample != nil`
- [x] 2.2 In the same `ZStack`, render `MuscleActivationBar` aligned `.bottomLeading` with 8 pt padding, gated on `showOverlays && session.currentAnalysisSample != nil`
- [x] 2.3 Apply `.opacity(showOverlays ? 1 : 0).animation(.easeInOut(duration: 0.2), value: showOverlays)` to both overlays so the toggle fades them rather than snapping
- [x] 2.4 Add an `eye` / `eye.slash` SF Symbol button aligned `.topTrailing` of the video `ZStack` that flips `showOverlays`, with `.accessibilityLabel("Show overlays")` / `"Hide overlays"`
- [x] 2.5 Keep the existing 240 pt video frame and `RoundedRectangle(cornerRadius: 12)` clipping; verify pose overlay still renders correctly above the video and below the analysis overlays

## 3. Build the compact transport + badge grid

- [x] 3.1 Keep the existing slider + frame counter row unchanged below the video card
- [x] 3.2 Keep the existing `playbackControls` row (◀ ⏸ ▶▶ Reset) unchanged below the slider row
- [x] 3.3 Replace the three `HStack { summaryBadge ... }` rows in the old `resultsSection` with two `HStack`s of three `summaryBadge`s each: row 1 `SO` / `|τ|` / `|τ|/m`, row 2 `Root res` / `L Foot` / `R Foot`
- [x] 3.4 Drop the `Frame N/total`, `Time x.xxx s` badges from the new grid (they remain in the slider counter row)
- [x] 3.5 When `session.currentAnalysisSample == nil` but `session.frameCount > 0`, render a single inline message in the badge-grid region instructing the user to run "Process All Frames" via the Batch sheet, sized to the same height as the populated grid so layout does not shift after processing completes
- [x] 3.6 When `session.frameCount == 0 && session.videoPlayer == nil`, render only the existing import-prompt message in the playback area and skip overlays, transport, and badge grid

## 4. Source sheet (import + summary)

- [x] 4.1 Add a `sourceSheet` view that contains the existing `importSection` content (importer button, file metadata rows, status / warning / error text, "Selected" row) followed by the existing `summarySection` content (Frames / Duration / FPS badges)
- [x] 4.2 Add a `.toolbar { ToolbarItem(placement: .topBarTrailing) { ... } }` modifier with a button using `Image(systemName: "tray.and.arrow.down")` that toggles `isShowingSourceSheet`, plus `.accessibilityLabel("Source files")`
- [x] 4.3 Attach `.sheet(isPresented: $isShowingSourceSheet) { sourceSheet.presentationDetents([.medium, .large]) }`
- [x] 4.4 Move the existing `.fileImporter(...)` modifier so it remains active while the Source sheet is presented (attach it inside the sheet content, since the import flow originates there)
- [x] 4.5 Add `.onAppear` logic that sets `isShowingSourceSheet = true` and `hasSeenSourceSheet = true` only when `!hasSeenSourceSheet && session.modelURL == nil && session.videoURL == nil`

## 5. Batch sheet (process / export)

- [x] 5.1 Add a `batchSheet` view that contains the existing `batchSection` content (Process All Frames button, progress indicator, summary badges, Export button, last-export-folder row)
- [x] 5.2 Add a second `ToolbarItem(placement: .topBarTrailing)` button using `Image(systemName: "square.stack.3d.up")` that toggles `isShowingBatchSheet`, plus `.accessibilityLabel("Batch analysis")`
- [x] 5.3 Attach `.sheet(isPresented: $isShowingBatchSheet) { batchSheet.presentationDetents([.medium, .large]) }`
- [x] 5.4 Move the existing `.sheet(isPresented: $isShowingExportShare) { ShareSheet(items: exportShareItems) }` modifier so it presents from inside the Batch sheet (alternatively, keep it on the root and verify it still presents correctly while the Batch sheet is up — pick the path that does not stack two sheets simultaneously on iOS 17)

## 6. Verify on device sizes

- [ ] 6.1 Build and run on iPhone 15 Pro simulator: confirm the entire layout (nav bar, video + overlays, slider, transport, 6-badge grid) fits without scrolling
- [ ] 6.2 Build and run on iPhone SE (3rd gen) simulator: confirm the layout either fits or falls back to a `ScrollView` cleanly without dropping any controls
- [ ] 6.3 Verify ◀ / ▶ updates the video pose, IK overlay, muscle overlay, and all six badges in a single visible step (no scrolling required)
- [ ] 6.4 Verify the overlay toggle fades both overlays in ~0.2 s and leaves the pose overlay visible
- [ ] 6.5 Verify the Source sheet auto-opens on first launch (clear app state via Simulator → Device → Erase All Content) and does not auto-open on subsequent launches
- [ ] 6.6 Verify VoiceOver announces the labeled accessibility text for the two toolbar buttons

## 7. Regression checks

- [ ] 7.1 Re-run a complete OpenCap import → Process All Frames → Export flow end-to-end and confirm exported files match a pre-change reference (no pipeline change should mean byte-identical exports for the same input)
- [x] 7.2 Confirm `BioMotion/Nimble/`, `BioMotion/Muscle/`, `BioMotion/Offline/`, and the `MuscleActivationBar` / `IKReadoutPanel` definitions in `ContentView.swift` are unchanged in the diff
- [x] 7.3 Confirm `project.yml` is unchanged (no XcodeGen regeneration required for this change)

## 8. Pre-ship hygiene

- [x] 8.1 Run `xcodebuild` for the iOS target and resolve any compile warnings introduced by the edit
- [ ] 8.2 If preparing a TestFlight build from this change, bump `CURRENT_PROJECT_VERSION` in `project.yml` and run `xcodegen generate` per project convention
