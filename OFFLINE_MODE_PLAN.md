# Offline Mode Plan

## Current State

The project is no longer at pure-planning stage.

Implemented now:

- offline `.mot` parsing and preparation in `BioMotion/Offline/ImportedMotion.swift`
- shared post-IK biomechanics path in `NimbleEngine`, so online and offline both feed the same `q -> dq/ddq -> ID -> static optimization` logic
- explicit model loading API in `NimbleEngine` so offline can load a user-selected `.osim`
- offline session in `BioMotion/Offline/OfflineSession.swift`:
  - `.osim` / `.mot` / `.mov` / `.keypoints.json + .keypoints.bin` import (sandbox-copied)
  - `AVPlayer`-based video playback with playback time as the master clock
  - prepared-motion sampling and 2D keypoint sampling driven by the same time observer
- desktop preprocessing script `scripts/build_offline_keypoints.py`:
  - reads any video, runs MediaPipe Pose by default, remaps to the OpenPose-25 ordering
  - emits the spec'd `<stem>.keypoints.json` + `<stem>.keypoints.bin` pair
  - detector is pluggable so a true OpenCap detector can drop in later
- iOS-side keypoint stack in `BioMotion/Offline/OfflineKeypointTrack.swift`:
  - `Codable` metadata types, frame-major `Float32` binary loader, `frameIndex(for:)` quantiser
  - OpenPose-25 skeleton edges + side classification
  - `AspectFitMapping` helper for image-pixel → view-space conversion
- offline UI in `BioMotion/App/OfflineAnalysisView.swift`:
  - controls-free `PlayerLayerView` (`AVPlayerLayer.resizeAspect`) under a `PoseOverlayView`
  - live 2D bones/joints drawn on top of the video, color-coded left/right, alpha-modulated by detector confidence
  - works in video-only mode (no `.mot`) so users can preview keypoints without running the biomechanics
- unit coverage in `BioMotionTests/OfflineKeypointTrackTests.swift` for binary round-tripping, fps quantisation, scalar-type rejection, and the aspect-fit mapping

Validated now:

- offline `.osim` and `.mot` import path exists in the app
- `demodata/opencap-demodata/trail2` contains a usable OpenCap offline dataset:
  - `LaiUhlrich2022_scaled.osim`
  - `run2.mot`
  - `run2.trc`
  - `run2.mov`
- `trail2` model and motion coordinates are compatible for the current offline path:
  - all `.mot` coordinates exist in the paired `.osim`
  - the only additional `.osim` coordinates are patellofemoral coupled coordinates (`knee_angle_r_beta`, `knee_angle_l_beta`)
- the keypoint binary format is round-trip tested end-to-end in the simulator

Still not implemented:

- a generated `run2.keypoints.{json,bin}` checked into `demodata/opencap-demodata/trail2` (preprocessor exists, asset has not been committed yet — gated on a Mac with `mediapipe` installed)
- offline muscle visualization (Phase 5; intentionally deferred)
- offline export workflow

So the current implementation status is:

- Phase 1 implemented
- Phase 2 implemented
- Phase 3 implemented (script ships; demo asset still to be generated locally and committed)
- Phase 4 implemented
- Phase 5 still open

## Goal

Add an offline mode that:

- keeps the current online ARKit and Nimble pipeline unchanged
- imports OpenCap-produced data
- continues downstream processing through inverse dynamics and static optimization
- presents playback in a way that is visually consistent with the current online experience
- is optimized for fast local iteration while bringing ID and static optimization online first

The target offline input set is:

- original video
- OpenCap `.mot`
- optional OpenCap `.trc`
- optional precomputed 2D keypoint file generated locally on desktop

## Front-End Data Preparation

Before the iOS app loads an offline session, prepare display-side pose data locally.

The intended preprocessing flow is:

1. Take the original source video.
2. Run OpenCap's open-source 2D pose detector pipeline locally.
3. Export the detected 2D keypoints into a file format the app can read.
4. Ship that keypoint file together with the video and `.mot`.

This preprocessing step exists specifically to support:

- app-side playback
- app-side 2D skeleton overlay
- real-time visual display during offline playback

The app should not own detector execution in the first implementation. It should only consume the generated keypoint file.

Confirmed for `demodata/opencap-demodata/trail2`:

- `run2.mot` and `run2.trc` both run at `120 Hz`
- `run2.mot` and `run2.trc` both contain `3823` frames over `31.85 s`
- `run2.mov` is `1280x720` HEVC with `3838` frames over `32.005 s`

Important interpretation:

- the video and kinematic timelines are close and start-aligned
- they are not frame-count identical
- playback synchronization must therefore be time-based, not frame-index-based

## Existing Online Pipeline

The current online path is:

1. `ARKit` body tracking produces 3D joint positions.
2. A 1-euro filter smooths the tracked joint positions.
3. `NimbleEngine.processFrame()` maps tracked joints to marker positions.
4. Nimble runs IK and produces joint coordinates `q`.
5. Savitzky-Golay filtering produces smoothed `q`, `dq`, and `ddq`.
6. Nimble runs inverse dynamics.
7. The muscle pipeline runs moment-arm computation and static optimization.
8. UI displays the camera feed with RealityKit 3D skeleton and 3D muscle entities.

Important implementation fact:

- the current online muscle display is not a 2D overlay on a 2D skeleton
- it is a 3D RealityKit overlay placed in AR world space in front of the camera feed

## Offline Product Direction

Offline mode should mirror the online user experience at a high level, but not by reusing ARKit.

The offline mode should have two parallel paths:

1. Display path
2. Analysis path

For the first implementation, the display path should be intentionally simplified:

- run 2D pose detection locally on desktop or laptop
- save a keypoint file
- ship that file with the demo data
- let the phone only do playback, overlay drawing, and downstream biomechanics

This is the preferred first milestone because it removes detector runtime complexity from the iOS app and keeps the implementation focused on getting ID and SO working quickly.

### Display Path

Use the original video as background and draw precomputed 2D pose detector keypoints on top of it.

This is the closest equivalent to the current online experience where the user sees body structure over the live camera image.

Key decisions:

- for offline display, use 2D pose detector output on the video
- compute the 2D detector output locally before loading the session on device
- store the detector result as a lightweight keypoint file
- do not use `.trc` 3D points as the primary display source
- do not require 3D-to-2D reprojection for the first version

Reason:

- 2D video plus 2D detector output is the simplest correct display path
- desktop preprocessing is much easier to debug than app-side detector integration
- offline playback on phone stays lightweight and deterministic
- `.trc` is 3D and would require camera calibration to overlay accurately on the original video
- `.mot` is for kinematics, not direct 2D drawing

### Chosen 2D Detection Source

The preferred source for offline 2D keypoints is OpenCap's open-source detector pipeline.

Relevant OpenCap components already identified:

- `utilsDetector.py`
- `main.py`
- associated OpenPose or mmpose-based video pose-detection path

Planned use:

- run the OpenCap detector stack locally outside the app
- capture per-frame 2D keypoints
- convert the output into a compact app-readable file

This keeps the offline display source consistent with the OpenCap ecosystem and avoids inventing a separate detector path before the offline biomechanics pipeline is validated.

### Analysis Path

Use the OpenCap `.mot` as the kinematic input source for downstream analysis.

This path should start after IK:

`OpenCap .mot -> q -> dq/ddq -> ID -> static optimization`

This means:

- offline mode does not run the current ARKit-to-Nimble IK stage
- offline mode reuses the current inverse dynamics and muscle pipeline logic as much as possible
- offline mode can be debugged with stable, repeatable imported kinematics

## Relationship Between Online and Offline

### Model Strategy

The project uses a split model strategy.

- online mode keeps the current in-app model and current pipeline unchanged
- offline mode should prefer the OpenCap `.osim` model that matches the imported `.mot`

This is an intentional decision.

Reason:

- offline import should minimize coordinate-name translation and model mismatch
- online stability is more valuable than forcing immediate model unification
- changing the online model now would also require revalidating ARKit marker mapping, online IK behavior, and muscle overlay assumptions

This means phase 1 does not attempt to unify the online and offline models.

Possible future work:

- once offline ID and static optimization are stable, evaluate whether the project should converge both modes onto one common model

### `trail2` Demo Dataset Implication

The `trail2` dataset is strong evidence that the split-model strategy is feasible.

Observed facts:

- `run2.mot` matches `LaiUhlrich2022_scaled.osim` coordinate naming directly
- no imported `.mot` coordinate is missing from the paired `.osim`
- the only extra `.osim` coordinates are `knee_angle_r_beta` and `knee_angle_l_beta`
- those extra coordinates are dependent patellofemoral coordinates driven by coordinate coupler constraints, not an independent kinematic source the `.mot` must provide
- in the current `NimbleBridge` load path, those dependent coordinates are not exposed as active analysis DOFs, so offline preparation sees a fully matched DOF set for `trail2`

Practical consequence:

- the current offline preparation path can treat `trail2` as the baseline validation dataset for Phase 1

### Same Parts

The following parts should be the same or as close as possible:

- model loading
- DOF handling
- inverse dynamics solve
- moment-arm computation
- static optimization
- muscle activation and force outputs
- export formats for downstream results

### Different Parts

The following parts are intentionally different:

- online input comes from ARKit 3D joints
- offline input comes from imported OpenCap `.mot`
- online uses the current in-app `.osim`
- offline should use the OpenCap-matching `.osim`
- online display uses RealityKit 3D overlay
- offline display uses video plus 2D pose overlay

### Practical Meaning

Offline mode should reuse the same post-IK biomechanics logic, but not the same pre-IK tracking path.

So the intended equivalence is:

- same downstream processing strategy
- different upstream kinematics source

Not:

- identical full pipeline end to end

## Why `.mot` Is the Correct Offline Analysis Input

OpenCap `.mot` already contains IK output, meaning joint coordinates over time.

For this project, inverse dynamics and static optimization only require:

- joint angles `q`
- joint velocities `dq`
- joint accelerations `ddq`
- DOF names aligned with the loaded model

Therefore:

- `.mot` is sufficient to continue the pipeline after IK
- `.trc` is not required for the first offline analysis implementation

## Role of `.trc`

`.trc` should be treated as optional in the first offline version.

It may later be used for:

- validation against imported `.mot`
- debugging marker behavior
- future 3D reconstruction or reprojection workflows
- future comparison UI

But it should not block the first offline implementation.

For the shipped demo dataset, however, `run2.trc` should still be kept in scope for validation work:

- it provides a stable reference that shares the same `120 Hz`, `3823`-frame timebase as `run2.mot`
- it is the best first comparison source for checking offline import correctness

## Role of Precomputed 2D Keypoints

Precomputed 2D keypoints should be treated as a first-class offline input for the initial implementation.

Recommended properties:

- one file per video
- explicit timestamp or frame index per sample
- stable keypoint ordering
- optional confidence score per keypoint

Recommended provenance:

- generated locally from the original video using OpenCap's open-source 2D detection pipeline

This file is only for display and visual debugging. It is not the source of truth for ID or static optimization.

The source of truth for offline analysis remains the imported `.mot`.

The source of truth for offline display remains the precomputed 2D keypoint file.

## Keypoint File Format

The offline keypoint file format is fixed as:

- one metadata JSON file
- one binary point-data file

This replaces the earlier all-JSON draft.

Chosen structure:

- `run2.keypoints.json`
- `run2.keypoints.bin`

Reasons:

- much smaller than verbose per-frame JSON at 120 Hz
- still easy to generate from Python preprocessing
- metadata remains easy to inspect and debug
- Swift can decode the JSON with `Codable` and load the binary blob efficiently

### File 1: Metadata JSON

Recommended extension:

- `.keypoints.json`

Schema:

```json
{
  "format": "biomotion-offline-keypoints-v1",
  "video": {
    "filename": "run2.mov",
    "width": 1280,
    "height": 720,
    "fps": 120.0,
    "frame_count": 3838
  },
  "keypoint_set": {
    "name": "opencap-openpose-25",
    "points": [
      "Nose",
      "Neck",
      "RShoulder",
      "RElbow",
      "RWrist",
      "LShoulder",
      "LElbow",
      "LWrist",
      "midHip",
      "RHip",
      "RKnee",
      "RAnkle",
      "LHip",
      "LKnee",
      "LAnkle",
      "REye",
      "LEye",
      "REar",
      "LEar",
      "LBigToe",
      "LSmallToe",
      "LHeel",
      "RBigToe",
      "RSmallToe",
      "RHeel"
    ]
  },
  "storage": {
    "binary_filename": "run2.keypoints.bin",
    "scalar_type": "float32",
    "channels_per_point": 3,
    "channel_order": ["x", "y", "confidence"],
    "layout": "frame-major"
  }
}
```

### File 2: Binary Point Data

Recommended extension:

- `.keypoints.bin`

Storage type:

- little-endian `Float32`

Per-point layout:

- `x`
- `y`
- `confidence`

Per-frame layout:

`frame0_point0[x,y,c] ... frame0_point24[x,y,c] frame1_point0[x,y,c] ...`

This is frame-major storage.

### Required Fields in Metadata JSON

Top level:

- `format`
- `video`
- `keypoint_set`
- `storage`

`video` object:

- `filename`
- `width`
- `height`
- `fps`
- `frame_count`

`keypoint_set` object:

- `name`
- `points`

`storage` object:

- `binary_filename`
- `scalar_type`
- `channels_per_point`
- `channel_order`
- `layout`

### Point Ordering Rule

The binary point order inside each frame must match `keypoint_set.points`.

This is mandatory.

The app must rely on array order, not names repeated per frame.

### Coordinate Convention

2D coordinates are stored in image pixel space:

- origin at top-left
- `x` increases to the right
- `y` increases downward

No normalization is used in phase 1.

Reason:

- direct draw to video space is simpler
- avoids ambiguity during overlay

### Confidence Convention

`confidence` is a floating-point value in `[0, 1]`.

If the preprocessing pipeline does not produce a confidence for a point, write:

- `0.0` for missing or unusable points

The app may hide points or bones below a configurable threshold.

### Missing Point Rule

Every frame must contain the same number of points as `keypoint_set.points`.

If a point is missing:

- keep the slot
- write `x = 0`
- write `y = 0`
- write `confidence = 0`

This keeps indexing and rendering logic simple and deterministic.

### Timing Rule

Phase 1 timing is derived from:

- `video.fps`
- `video.frame_count`
- frame index in the binary stream

Per-frame timestamps do not need to be stored in the binary file in phase 1.

Time at frame `i` is:

`i / fps`

This is sufficient because the video and playback keypoints are frame-aligned.

### Chosen Keypoint Set for Phase 1

Use the OpenCap/OpenPose 25-keypoint ordering for phase 1.

Reason:

- it matches OpenCap's existing detector-side conventions
- it is enough for skeleton overlay
- it avoids unnecessary mapping complexity during the first offline implementation

### Size Estimate

For 25 keypoints, 3 channels, `Float32` storage:

- `25 * 3 * 4 = 300 bytes` per frame

At 120 Hz for about 32 seconds:

- about `300 * 3838 ~= 1.15 MB`

This is much better than verbose JSON for the same data.

### Swift Loading Implication

Swift should decode:

- metadata JSON using `Codable`
- binary payload using `Data`

Suggested app-side types:

- `OfflineKeypointMetadata`
- `OfflineVideoMetadata`
- `OfflineKeypointSet`
- `OfflineKeypointStorage`

The binary payload can then be indexed by:

- frame index
- point index
- channel offset

## Offline Display Strategy

### First Version

The first offline display should show:

- video playback
- precomputed 2D keypoints
- 2D bones
- existing numeric panels for IK, ID, and muscle outputs where applicable

This display path should be independent from the imported `.mot` except for timeline synchronization.

Practical interpretation:

- the 2D overlay is a visualization layer
- the `.mot` is the analysis layer
- both share the same playback clock

### Why Not Reuse the Online 3D Muscle Overlay Directly

The current online `MuscleOverlay` depends on:

- 3D joint positions in AR world space
- RealityKit entities
- 3D segment placement between world-space points

Offline video playback does not naturally provide that same world-space context.

Without a calibrated 3D camera model and a 3D pose source in the same frame:

- the online RealityKit muscle placement cannot simply be dropped onto the offline video

### Future Muscle Visualization for Offline

The first offline version should not block on projecting full 3D muscles back to video.

Recommended stages:

1. First ship video plus 2D pose overlay and offline ID/SO results.
2. Then add an offline muscle visualization layer if needed.

For that future layer, two possible directions exist:

- 2D approximate muscle overlay based on 2D skeleton segments
- true 3D muscle reprojection, which would require camera calibration and a 3D pose source suitable for projection

The first direction is much cheaper and more aligned with the current requirements.

## Feasibility Assessment

### Overall Assessment

This simplified offline plan is highly feasible.

It is substantially more feasible than:

- running a 2D detector directly on device
- reusing the current online ARKit display pipeline for offline playback
- projecting 3D TRC or 3D muscle geometry back into the original video

### Why It Is Feasible

- the imported `.mot` already gives the kinematic state needed to start after IK
- the current codebase already contains the ID and muscle solving logic
- the video and `.mot` are already aligned
- precomputing 2D keypoints locally removes the heaviest and least app-friendly runtime dependency
- OpenCap already has an open-source 2D detector pipeline that can be reused during preprocessing

### Main Remaining Risks

- mapping imported `.mot` coordinate names into the current model DOF set
- ensuring unit conversion is correct
- making sure `dq/ddq` generation is numerically stable
- keeping keypoint file timing and video playback perfectly aligned

These are manageable engineering risks and are much smaller than the risks of integrating full detector runtime into the phone app.

### Recommendation

Proceed with the simplified plan:

- precompute 2D keypoints locally using OpenCap's open-source detector pipeline
- import video + `.mot` + 2D keypoint file into the app
- focus the app on playback, overlay, ID, and static optimization

This is the fastest path to getting the downstream biomechanics stack running and debugged.

## Time Synchronization

Given the current `trail2` dataset characteristics:

- the video player time should be the master clock
- the analysis state should sample the imported `.mot` at the current playback time

Recommended behavior:

- for each playback timestamp `t`, read or interpolate the nearest kinematic state from `.mot`
- compute or retrieve the corresponding downstream biomechanical outputs for the same `t`
- display the current 2D pose overlay on the video for that same `t`

Explicit constraint from `trail2`:

- do not assume `videoFrameIndex == motFrameIndex`
- clamp or interpolate near the end of the clip because the video contains slightly more frames than the motion data

## Kinematic Processing Requirements

Although `.mot` provides `q`, the project still needs `dq` and `ddq` for inverse dynamics.

To stay consistent with the current online behavior:

- offline mode should use the same or equivalent smoothing and derivative logic as online mode
- the existing Savitzky-Golay-based approach is the preferred baseline

Reason:

- inverse dynamics is sensitive to derivative quality
- using a different derivative strategy offline would create avoidable differences from the online path

## Model Compatibility Constraint

The main technical risk in the offline analysis path is not video synchronization. It is model-coordinate compatibility.

The imported OpenCap `.mot` should align with the OpenCap `.osim` loaded for offline mode:

- coordinate names must match or be remapped
- coordinate meanings must match
- units must be handled correctly
- rotational coordinates must be converted into the app's internal units as needed

This is the most important correctness check for offline ID and SO.

Because the offline path is expected to use the OpenCap model, this risk is significantly reduced compared with forcing the imported `.mot` onto the current online model.

## Recommended Architecture

### New Offline Data Types

Add an offline session abstraction that groups:

- video URL
- `.mot` URL
- offline `.osim` URL
- optional `.trc` URL
- precomputed 2D keypoint URL

### New Offline Playback Layer

Add a playback-oriented UI layer that owns:

- `AVPlayer` or equivalent video playback
- 2D keypoint overlay rendering from precomputed keypoint files
- current playback time

### New Offline Analysis Entry Point

Add a path that bypasses live frame-to-IK processing and begins from imported kinematics.

This should either:

- extend `NimbleEngine` with an imported-kinematics entry point

or

- introduce a dedicated offline analysis engine that reuses the same bridge and muscle components

Preferred design rule:

- do not duplicate ID or muscle solver logic
- factor shared post-IK logic into reusable methods
- keep model selection explicit so offline can load the OpenCap-specific `.osim` without affecting online mode

### Suggested Preprocessing Contract

Define a small local preprocessing step outside the iOS app:

1. Run OpenCap's open-source 2D detector on the source video locally.
2. Extract per-frame 2D keypoints from the detector result.
3. Export a compact keypoint file.
4. Place the output next to the video and `.mot`.

The iOS app should assume the keypoint file already exists and should not own detector execution in phase 1.

### Suggested Keypoint File Requirements

The preprocessing output should support app-side real-time playback display.

Minimum contents:

- video-relative frame index or timestamp
- keypoint name or fixed keypoint ordering
- `x`
- `y`
- optional confidence

Preferred behavior:

- deterministic ordering across all frames
- explicit metadata for image width and height
- easy decoding from Swift without Python dependencies

The exact serialization format can be chosen later, but JSON or a compact line-delimited format are both acceptable for phase 1.

## Implementation Phases

### Phase 1: Offline Analysis Only

Deliver:

- offline `.osim` loading support
- `.mot` import
- DOF mapping
- `q` ingestion
- `dq/ddq` generation
- inverse dynamics
- static optimization
- numeric output validation

No video overlay required for this phase.

`trail2` execution target for Phase 1:

- load `LaiUhlrich2022_scaled.osim`
- import `run2.mot`
- confirm that all source coordinates map into the loaded model
- confirm that the current runtime DOF set presented by `NimbleBridge` is fully matched for `trail2`
- use `run2.trc` as the first validation reference where comparison is needed

### Phase 2: Offline Video Playback

Deliver:

- video playback UI
- video time as master clock
- synchronization with imported kinematics

Execution note for `trail2`:

- synchronization must be implemented against playback time, not by assuming equal frame counts

### Phase 3: Local Preprocessing Pipeline — implemented

Delivered:

- `scripts/build_offline_keypoints.py` consumes any source video and writes the spec'd `<stem>.keypoints.json` + `<stem>.keypoints.bin` pair in the OpenPose-25 layout
- the detector backend is abstracted behind a small `Detector` interface; the default implementation is MediaPipe Pose remapped into OpenPose-25 (Neck and midHip are derived from shoulder/hip midpoints; LSmallToe and RSmallToe stay zeroed because MediaPipe does not produce them)
- HEVC rotation is handled via `cv2.CAP_PROP_ORIENTATION_AUTO` so written pixel dimensions match what `AVPlayer` renders
- frame counts written into metadata reflect what was actually decoded, with the container's declared count surfaced under `declared_frame_count` only when they disagree

Current gap:

- the generated keypoint pair for `trail2/run2.mov` has not been committed; running the script on a Mac with `pip install mediapipe opencv-python` produces it directly into `demodata/opencap-demodata/trail2/`
- a real OpenCap / OpenPose detector backend is not yet wired into the script (the `Detector` interface is the seam where it will plug in)

### Phase 4: Precomputed 2D Pose Overlay — implemented

Delivered:

- `OfflineKeypointTrack` decodes the metadata JSON via `Codable` (with snake_case key conversion), memory-maps the binary blob, and exposes `frame(at:)` / `frame(atTime:)` plus an O(1) `frameIndex(for:)` quantiser
- `OpenPose25Skeleton` defines the bone topology and side classification; `AspectFitMapping` performs the image-pixel → view-space transform so the overlay lines up with `AVPlayerLayer.resizeAspect`
- `PlayerLayerView` is a controls-free `AVPlayerLayer` wrapper; `PoseOverlayView` paints bones (alpha modulated by min-pair confidence) and joints on top
- `OfflineSession.importKeypoints(from:)` accepts the JSON+BIN pair (multi-select), copies them into a per-import sandbox folder, validates the format, and starts publishing `currentKeypoints` from the same time observer that drives prepared-motion sampling
- the offline tab now renders the overlay in a `ZStack` over the player and tolerates the no-`.mot` case (video + keypoints only)

This is the main visual match to the online experience.

### Phase 5: Optional Offline Muscle Visualization

Possible deliverables:

- simple 2D muscle-group approximation driven by current activations

Not recommended for the first implementation:

- full 3D muscle reprojection into the video

## Non-Goals for First Version

The first offline implementation should explicitly avoid:

- modifying the existing online ARKit pipeline
- replacing the current online `.osim`
- re-running full IK from video when OpenCap `.mot` already exists
- requiring `.trc` for baseline offline analysis
- requiring camera intrinsics or extrinsics
- full 3D muscle projection into the original video
- running the 2D pose detector directly on device

## Final Summary

The planned offline mode is:

- local preprocessing: run OpenCap's open-source 2D detector on desktop and save keypoint file
- display: original video plus precomputed 2D pose detector overlay
- analysis: imported OpenCap `.osim + .mot` feeding the existing post-IK biomechanics pipeline

In compact form:

`video + precomputed 2D keypoints -> 2D overlay`

`OpenCap .osim + .mot -> q -> dq/ddq -> ID -> static optimization`

This keeps the online workflow intact, reuses the most valuable downstream code, avoids unnecessary detector integration on device, and is the fastest path to getting offline ID and static optimization running for debugging.
