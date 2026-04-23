## Context

The Offline tab (`BioMotion/App/OfflineAnalysisView.swift`) is the per-frame review surface for recordings imported from OpenCap or generated from a live capture. The current view body wraps every concern — file import, source metadata, prepared-motion summary, video playback, frame inspector, batch processing — into one vertically stacked `ScrollView`. On iPhone the cumulative height (~1370 pt) exceeds the usable height (~660 pt) by roughly 2×, so the user can never see the playback video and the per-frame analysis output (muscle activations + IK joint angles + joint torques + foot forces) at the same time.

This is purely a UI-layer problem. None of the pipeline stages are involved:

- **ARKit / IK / ID / moment-arm / OSQP stages** — unchanged. The change does not touch `NimbleEngine`, `NimbleBridge.mm`, `MuscleSolver.mm`, or `MomentArmComputer.mm`.
- **Per-frame compute budget** — unchanged. No new work is scheduled per frame; the overlays consume `OfflineSession`'s already-published `currentAnalysisSample`, `currentKeypoints`, and `videoPlayer`.
- **Threading** — unchanged. All affected views live on the main actor; `OfflineSession` already publishes on main.
- **nimblephysics patches** — none required. No `DART_IOS_BUILD` guards added.

The proposal locks the high-level direction: overlay analysis panels on the video, demote import / batch chrome to toolbar sheets, compress the inspector to a 2×3 badge grid. This document records the technical decisions needed to land that without regressing existing behavior.

## Goals / Non-Goals

**Goals:**

- All four scrubbable surfaces — video pose, IK overlay, muscle overlay, badge grid — visible simultaneously on iPhone 14 / 15 / 16 (Pro and non-Pro) without scrolling, so ◀ / ▶ updates everything in one glance.
- Reuse `MuscleActivationBar` and `IKReadoutPanel` from `ContentView.swift` unmodified — both are already styled for translucent overlay use in the live-capture screen.
- Match the visual language of the live ARKit capture screen (overlay-on-video) so users moving between live and playback do not re-learn the layout.
- Provide a one-tap escape hatch (`eye.slash`) when overlays obstruct the pose.
- Demote rarely-used controls (Import, Batch) without losing access to any current functionality.
- Graceful fallback on smaller phones (SE) without bespoke layout code.

**Non-Goals:**

- Any change to the analysis pipeline, file formats, exports, or `OfflineSession` published surface.
- iPad-optimized split layout, hardware-keyboard scrubbing shortcuts, or persisted overlay state.
- Redesign of the live capture screen.

## Decisions

### Overlay placement: top = IK, bottom = muscles, both inside the video card

Both `IKReadoutPanel` and `MuscleActivationBar` already use `.black.opacity(0.5)` backgrounds and `.foregroundStyle(.white)` — they were built for camera overlay. Stack them in a `ZStack` over `PlayerLayerView`, aligned `.topLeading` and `.bottomLeading` respectively, with `.padding(8)` from the video edges.

Rationale for top/bottom (vs. corners or sides):

- `PoseOverlayView` draws skeleton joints in the video's coordinate space; for a centered standing/walking subject the head/feet edges of the frame are the cleanest dead zones.
- IK panel is a single-row horizontal scroll of compact items — fits the top edge naturally.
- Muscle bar is also a single-row horizontal scroll with vertical activation bars — bottom edge is the natural footing for "bar charts."
- Side placement would require rotating text or re-layout of the existing components; rules out reuse.

**Alternative considered:** put the muscle bar below the video instead of overlaid. Rejected because it adds ~85 pt vertical cost and the user explicitly chose Direction B (overlay) during exploration, in part to keep the video card visually unified with the live-capture screen.

### Overlay-visibility toggle: view-local `@State`, no persistence

Single `@State private var showOverlays = true` in `OfflineAnalysisView`. Toggle button is an SF Symbol button (`eye` / `eye.slash`) anchored `.topTrailing` of the video card. Tap fades both overlays via `.opacity(showOverlays ? 1 : 0).animation(.easeInOut(duration: 0.2), value: showOverlays)`.

Rationale:

- The need to hide overlays is situational ("the IK panel is over the subject's face right now"), not a user preference. Persisting the choice would surprise users when re-opening the tab.
- View-local state means no `@AppStorage` key, no `OfflineSession` field — zero new global state.
- The pose overlay (`PoseOverlayView`) stays visible regardless; only the analysis text overlays toggle. Hiding pose would mask information that has no other surface in this view.

### Source / Batch sheets: toolbar buttons, medium+large detents, icon-only

Two `.toolbarItem(placement: .topBarTrailing)` buttons:

- `Image(systemName: "tray.and.arrow.down")` → presents `sourceSheet` (current `importSection` + `summarySection` content).
- `Image(systemName: "square.stack.3d.up")` → presents `batchSheet` (current `batchSection` content + the existing `ShareSheet`).

Both sheets use `.presentationDetents([.medium, .large])` so the sheet does not fully cover the playback area when the user wants to peek.

Rationale for icon-only labels: user reported import is rare in a session, so the toolbar real estate is better spent on accessibility-labeled SF Symbols than on `Text("Source")` / `Text("Batch")` that would compete with the navigation title. Each button gets `.accessibilityLabel("Source files")` / `.accessibilityLabel("Batch analysis")`.

**Alternative considered:** a single `Menu` button with both items inside. Rejected because it adds an extra tap to reach what are already infrequent actions, and because the export share-sheet flow inside Batch wants its own dismissal context.

### Auto-open Source on first launch only

```swift
@AppStorage("offline.hasSeenSourceSheet") private var hasSeenSourceSheet = false
```

In `.onAppear` of the root view: if `!hasSeenSourceSheet && session.modelURL == nil && session.videoURL == nil` then set `isShowingSourceSheet = true` and `hasSeenSourceSheet = true`.

Rationale:

- First-time users have no folder imported and no obvious entry point — the auto-open removes a "where do I start?" moment.
- Returning users who deliberately closed the sheet are not nagged.
- The `session.modelURL == nil && session.videoURL == nil` guard means even a fresh install does not auto-open if the user somehow already has data loaded (e.g., future state restoration). Defensive, not strictly required today.
- `@AppStorage` is already a standard SwiftUI pattern; no new dependency.

### Inspector compression: 2×3 grid of equal-column `summaryBadge`s

Replace the three `HStack { summaryBadge ... }` rows in `resultsSection` with a single 2-row, 3-column grid:

```
Row 1: [SO ✓ / FAIL]  [|τ| 14.2 Nm]   [|τ|/m 0.18 Nm/kg]
Row 2: [Root res 0.02] [L Foot 380 N] [R Foot swing]
```

Implementation: two `HStack`s each containing three `summaryBadge`s, since `summaryBadge` already uses `.frame(maxWidth: .infinity)` to size to equal columns. No `Grid` API needed.

Dropped from the inspector: the `Frame N/total`, `Time x.xxx s` badges — both are duplicated in the slider counter row directly above. Keeping them was costing a full row (~36 pt) for no information gain.

Rationale for "all six visible always" (vs. horizontal-scroll or single-row condensed): user explicitly said all six are read while scrubbing. A scrolling strip would mean some are off-screen at any given moment, defeating the purpose. Equal columns also keep value text from jittering as foot force flips between `380 N` and `swing`.

### Empty state when `currentAnalysisSample == nil`

Two distinct empty states:

- **No video / no motion loaded** (`session.videoPlayer == nil && session.frameCount == 0`): show only the import prompt centered in the playback card area. No overlays, no badge grid, no transport.
- **Loaded but not yet processed** (`session.frameCount > 0 && session.currentAnalysisSample == nil`): show the video + transport. Overlays are hidden (no analysis to display). Badge grid shows a single inline message replacing all six badges: "Tap Batch ▸ Process All Frames to compute IK→ID→SO."

Rationale:

- Showing empty overlays (`Muscles ▮▮▮▮▮▮ all 0%`) would be misleading.
- A single message in the badge grid slot keeps the layout dimensions stable so the user does not see content shift after processing finishes — only content fill in.

### Fallback for narrow heights: conditional `ScrollView`

Wrap the body in:

```swift
GeometryReader { geo in
    if geo.size.height < 600 {
        ScrollView { layout }
    } else {
        layout
    }
}
```

(Threshold tuned during implementation; 600 pt is a starting point that excludes iPhone SE 1st gen but includes mini.)

Rationale: most current iPhones do not need a scroll view. Forcing one universally re-introduces the scroll behavior that caused the original problem on devices that do not need it. The conditional keeps the no-scroll experience on all common targets and only degrades gracefully where physically necessary.

**Alternative considered:** `ViewThatFits` to switch between two prebuilt layouts (compact vs. comfortable). Rejected as over-engineering for a single screen — both branches would render the same content; only the wrapper differs.

## Risks / Trade-offs

- **[Overlay obscures pose for non-centered subjects]** → IK panel covers the top ~70 pt of the video; an upper-body-framed subject's head could be partially hidden. **Mitigation**: the `eye.slash` toggle hides overlays in one tap. The pose overlay itself stays visible regardless of the toggle.

- **[Sheet covers transport when summoned]** → Source / Batch sheets at `.medium` detent cover roughly the bottom half of the screen, which is where the slider and ◀ / ▶ live. **Mitigation**: this is acceptable because sheets are summoned to perform an action (import, process), not to scrub. The `.large` detent is available when the user wants the full sheet.

- **[Auto-open on first launch interrupts the empty Offline tab]** → A user who lands on the tab just to look at it will see a sheet pop up. **Mitigation**: the `@AppStorage` flag fires once per install, and the sheet is dismissible without action. The trade-off is judged worthwhile because a first-time user with no data and no visible import button would otherwise have nowhere to go.

- **[6-badge grid still doesn't fit on iPhone SE]** → The conditional `ScrollView` re-introduces scrolling on small screens. **Mitigation**: explicit, narrow scope (only sub-600 pt heights). On those devices the user gets the original experience back, which is no worse than today.

- **[Toolbar SF Symbols ambiguous without text]** → A new user might not know what `tray.and.arrow.down` means. **Mitigation**: `.accessibilityLabel` for VoiceOver; long-press tooltip on iPad; the auto-open of Source on first launch demonstrates the icon's role implicitly.

- **[Layout drift if `MuscleActivationBar` / `IKReadoutPanel` ever resize]** → These components are owned by `ContentView.swift` and consumed in two places now (live + playback). A future change to their internal padding could push the badge grid off-screen on tight devices. **Mitigation**: documented in `tasks.md` that any change to those components must consider both consumers; verified in implementation by measuring against the 690 pt budget on iPhone 14 baseline.

## Migration Plan

This is a UI-only change in a single file with no data, schema, or export-format implications. There is nothing to migrate.

- **Deploy:** ship in the next TestFlight build. Bump `CURRENT_PROJECT_VERSION` in `project.yml` per project convention before upload.
- **Rollback:** revert the commit. No state, files, or persisted user preferences are introduced beyond the `offline.hasSeenSourceSheet` `@AppStorage` flag, which is harmless if left behind on rollback.

## Open Questions

None — all design decisions were resolved in exploration.
