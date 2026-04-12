# BioMotion

Real-time musculoskeletal analysis iOS app. ARKit body tracking → Nimble IK/ID → OSQP muscle optimization → 3D visualization.

> **Setup, dependency cloning, patching, and full build/TestFlight commands live in [`README.md`](./README.md).** This file is LLM-facing context for architecture and gotchas only.

## Architecture

```
ARKit 91 joints → 1-euro filter → Nimble IK (~1ms) → Nimble ID (~0.1ms)
  → Computed moment arms (numerical diff) → OSQP muscle opt (~0.5ms)
  → 3D muscle visualization (RealityKit)
```

### Swift ↔ C++ Bridge

ObjC++ wrappers in `BioMotion/Nimble/` and `BioMotion/Muscle/`:
- `NimbleBridge.h/.mm` — loads .osim, runs IK/ID. Registers virtual markers at joint centers for ARKit compatibility.
- `MuscleSolver.h/.mm` — parses 80 muscles, runs OSQP static optimization
- `MomentArmComputer.h/.mm` — parses muscle paths, computes moment arms via FK + numerical differentiation
- Bridging header: `BioMotion/Nimble/BioMotion-Bridging-Header.h`

### Key files

| File | Purpose |
|------|---------|
| `project.yml` | XcodeGen project definition (team, signing, lib paths, build settings) |
| `BioMotion/Resources/Rajagopal2016.osim` | Musculoskeletal model (80 muscles, 39 DOFs, 66 markers) |
| `BioMotion/ARKit/BodyTrackingSession.swift` | ARKit body tracking + 1-euro filter |
| `BioMotion/ARKit/MuscleOverlay.swift` | 3D muscle capsule visualization (28 muscles) |
| `BioMotion/Nimble/NimbleEngine.swift` | Orchestrates IK → ID → muscle pipeline on background queue |
| `BioMotion/Recording/TRCExporter.swift` | OpenSim .trc export |
| `BioMotion/App/CalibrationView.swift` | T-pose calibration with live camera |
| `BioMotion/Muscle/osqp_interrupt_stub.c` | OSQP interrupt handler stub for iOS |
| `nimblephysics/CMakeLists.txt` | iOS-specific CMake (NOT the original — upstream is preserved as `CMakeLists_original.txt`) |

### Nimble iOS patches

The vendored `nimblephysics/` tree carries iOS-specific patches. Grep for `DART_IOS_BUILD` to find them. Touched areas:

- `config.hpp` — manual config with `HAVE_IPOPT=0`, `DART_IOS_BUILD=1`
- `MeshShape.hpp` / `MeshShape_ios.cpp` — Assimp stubs
- `OpenSimParser.cpp` — guarded MarkerFitter, GUIRecording, SdfParser, MJCFExporter includes
- `MarkerAspect.hpp` / `Marker.hpp` — enum `NO` → `CONSTRAINT_NONE` (ObjC macro conflict)
- `AssimpInputResourceAdaptor.hpp`, `SoftMeshShape.hpp` — Assimp guards
- `C3DLoader.hpp`, `LilypadSolver.hpp`, `Anthropometrics.hpp`, `IKErrorReport.hpp` etc — GUIWebsocketServer guards
- `DARTCollisionDetector_ios.cpp` — stub for collision detector factory
- Vendored: Eigen 3.4.0 (`third_party/eigen`), tinyxml2 (`third_party/tinyxml2`)

## Gotchas

- **Eigen version**: Nimble requires Eigen 3.x. Eigen 5.x (Homebrew default) has breaking API changes. Use vendored `third_party/eigen` (3.4.0).
- **Marker names**: ARKit joints map to virtual markers at body node origins, NOT to the model's surface markers (RASI, LASI etc). See `NimbleBridge.mm` virtual marker registration.
- **C++ exceptions**: Always use C++ `try/catch`, never ObjC `@try/@catch` — ObjC exceptions don't catch `std::exception` or SIGSEGV.
- **Build number**: Must increment `CURRENT_PROJECT_VERSION` in `project.yml` before each TestFlight upload.
- **Library search paths**: Conditional on SDK — `[sdk=iphoneos*]` for device, `[sdk=iphonesimulator*]` for simulator.
- **XcodeGen**: Always run `xcodegen generate` after editing `project.yml` — never edit `BioMotion.xcodeproj/` by hand.
