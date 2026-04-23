## Why

The real-time (online) ARKit pipeline currently always loads the bundled generic `Rajagopal2016.osim` and at calibration time runs `scaleModelWithHeight:`, which derives per-segment scales from ARKit joint distances and uses a `height/1.8` fallback. Body mass is never asked for — it is propagated from the generic 75 kg model by Nimble's `setBodyScales`, so inverse-dynamics torques and per-kilogram normalizations can be off by 5–15 kg of mass and several cm of segment length per user.

The offline pipeline already accepts an OpenCap-produced `*_scaled.osim` (the `Resources/trail2/LaiUhlrich2022_scaled.osim` is a real example: per-body anisotropic `<scale_factors>` and arbitrary-precision `<mass>` values that sum to the subject's actual weight). The body-name conventions (`pelvis`, `femur_r/l`, `tibia_r/l`, `talus_r/l`, `toes_r/l`, `torso`, `humerus_r/l`, `ulna_r/l`, `hand_r/l`) match what `NimbleBridge`'s virtual-marker table targets. The only blocker for online use is the unconditional anthropometric rescaling at calibration.

This change lets the online pipeline consume the same OpenCap-scaled models the offline pipeline already accepts, so the real-time ID/SO numbers reflect the subject's real anthropometry without requiring a second ARKit-based scaling pass.

## What Changes

- Add a **subject profile** concept: a named on-device record that owns one `.osim` file and a "model source" tag (`generic-bundled` or `prescaled-imported`). Profiles persist in `Documents/Profiles/<id>/`.
- Add a UI surface to **import a pre-scaled `.osim`** for the online pipeline, mirroring the security-scoped sandbox-copy logic already used in `OfflineSession.importModel`.
- Make the online **calibration step mode-aware**: for `prescaled-imported` profiles, skip `scaleModelWithHeight:` entirely; still run ground-plane calibration and 1-euro filter warm-up.
- `NimbleEngine` exposes the loaded model's source mode (already-scaled vs needs-scaling) so downstream code (`MomentArmComputer`, `MuscleSolver`, UI badges) can rely on a single source of truth for "is this model anthropometrically correct for this user".
- Default behavior is unchanged: with no profile selected, the app loads the bundled `Rajagopal2016.osim` and runs the existing scaling-at-calibration flow.
- **Non-goals**:
  - No change to the offline pipeline (it already does the right thing).
  - No new C++/ObjC++ bridge surface — `NimbleBridge.scaleModelWithHeight:` and `loadModel:` are reused as-is; the conditional skip happens in Swift.
  - No automatic OpenCap session orchestration, no in-app OpenSim ScaleTool. Users bring their own scaled `.osim` from any source (OpenCap web, OpenSim GUI, etc.).
  - No multi-user switching mid-session, no profile sync across devices.
  - No support for swapping muscle topology mid-run: the muscle visualization assumes the loaded model's muscle set is parsed at `loadModel` time, which already works.

## Capabilities

### New Capabilities
- `subject-profile`: managing per-subject scaled musculoskeletal models for the online pipeline — profile CRUD, `.osim` import + sandbox storage, and the contract that calibration honors the profile's "model source" mode.

### Modified Capabilities
<!-- None. openspec/specs/ is currently empty; the existing online calibration behavior is implicit in the codebase and is being formalized for the first time as part of this change. -->

## Impact

- **Swift**:
  - `BioMotion/Nimble/NimbleEngine.swift` — new `modelSource` published property; `loadModel(fromPath:source:)` overload; helper to load the bundled default.
  - `BioMotion/App/CalibrationView.swift` — branch on `modelSource` to decide whether to call `scaleModel(...)`.
  - New files: `BioMotion/App/SubjectProfile.swift` (model + persistence), `BioMotion/App/SubjectProfileStore.swift` (CRUD + active-profile state), `BioMotion/App/SubjectProfileView.swift` (UI for creating, importing, selecting profiles).
  - `BioMotion/App/ContentView.swift` — wire active profile into the engine load flow at app start; surface profile name + source badge.
- **ObjC++ / C++**: no changes. `NimbleBridge.loadModel:` and `scaleModelWithHeight:` are reused unchanged; the conditional happens in Swift.
- **Project**: `project.yml` gets the new Swift files (run `xcodegen generate`). No new dependencies, no SDK-conditional library paths, no bridging-header churn.
- **Filesystem**: profiles stored in app `Documents/Profiles/<uuid>/model.osim` (survives `tmp/` cleanup, included in iCloud backup unless explicitly excluded — out of scope for this change).
- **Docs**: update `docs/UI_PARAMETERS.md` if a "Profile" or "Model source" badge is added; update `CLAUDE.md` to note that online supports pre-scaled models.
- **Risk**: muscle set / DOF count differs between Rajagopal2016 (39 DOF, 80 muscles) and other scaled models (LaiUhlrich2022 has different muscle naming). `MuscleOverlay`'s 28-capsule list is name-keyed, so unsupported muscles render as missing. Documented as a known limitation; users importing a non-Rajagopal-family model see fewer overlay capsules.
