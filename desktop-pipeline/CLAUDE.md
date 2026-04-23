# BioMotion Desktop Validation Pipeline

LLM-facing context for the `desktop-pipeline/` sub-project. Setup instructions
and user-facing usage live in [`README.md`](./README.md). This file is the
architecture cheat sheet.

## Purpose

Produce a **defensible OpenSim ground truth** for any OpenCap trial the iOS
app processes, then diff it against the iOS app's exported `.mot` / `.sto` /
`.json` files. Every file written by this pipeline mirrors the names emitted
by `BioMotion/Offline/OfflineAnalysisExporter.swift` so comparison is just
column-aligned arithmetic.

## Data flow

```
Trial config (configs/*.yaml)
        │
        ▼
biomotion_desktop.trial.Trial            ← dataclass with absolute paths
        │
        ├─► opensim_ik.run_ik(...)        → <stem>_kinematics.mot
        ├─► opensim_id.run_id(...)        → <stem>_inverse_dynamics.sto
        └─► opensim_so.run_so(...)        → <stem>_activations.sto
                                          → <stem>_muscle_forces.sto

iOS export folder (zipped from phone via ShareLink)
        │
        ▼
biomotion_desktop.ios_export.IOSExport   ← parses identical filenames
        │
        ▼
biomotion_desktop.compare.compare_pair(...) → JSON report (per column metrics)
```

## Why the file names match the iOS exporter exactly

`OfflineAnalysisExporter.export(...)` produces these filenames:

| File                              | Content                          | Units |
|-----------------------------------|----------------------------------|-------|
| `<stem>_kinematics.mot`           | joint angles                     | deg   |
| `<stem>_inverse_dynamics.sto`     | joint torques                    | Nm    |
| `<stem>_activations.sto`          | muscle activations               | 0–1   |
| `<stem>_muscle_forces.sto`        | muscle forces                    | N     |
| `<stem>_summary.json`             | metadata (fps, mass, etc.)       | —     |

Desktop `pipeline.run_pipeline(...)` writes the same five filenames into the
output directory. `compare.compare_folders(desktop, ios, stem)` looks them up
by name — no hand-mapping required.

## OpenSim driver gotchas

- **`InverseKinematicsTool`**: requires an **`<IKTaskSet>`** with a task per
  marker present in the model. We auto-build it from the model's marker set
  (default weight 1.0) instead of expecting the user to supply a setup XML.
- **`InverseDynamicsTool`** with no GRF will produce huge residuals at the
  pelvis — that is expected for OpenCap upper-body trials. Use the residual
  magnitudes only as a sanity diff against the iOS app, not as physics truth.
- **`StaticOptimization`** runs through `AnalyzeTool`. The activation file it
  writes is named `<results_prefix>_StaticOptimization_activation.sto` by
  OpenSim itself; we **rename** it on disk to `<stem>_activations.sto` to
  match the iOS exporter.
- **Filtering**: kinematics are low-pass filtered (Butterworth, default 6 Hz)
  before ID and SO to avoid amplified acceleration noise. Override via the
  trial config's `lowpass_hz`.

## Comparison semantics

Two columns are considered the "same" channel only if their names match
**exactly** (case-sensitive). When the iOS Nimble model and the OpenSim model
disagree on naming (e.g. `ankle_angle_r` vs `ankle_r`), the channel lands in
`unmatched_columns` rather than silently being skipped.

Time alignment: linear interpolation of the iOS series onto the desktop time
grid (desktop is treated as authoritative because it always uses the source
TRC framerate). Out-of-range samples are dropped, not extrapolated.

## Where this hooks back into the iOS repo

- **Inputs**: `../demodata/opencap-demodata/**` — the same trials the iOS app
  consumes. The default `configs/trail2.yaml` points there with relative paths.
- **iOS export folder**: produced by the **Export** button in the Offline tab
  (`OfflineAnalysisExporter.Output.folder`). Share via AirDrop / Files / etc.,
  unzip on the Mac, point `--ios <path>` at the resulting folder.

## Non-goals

- This sub-project does **not** rebuild the iOS app, run XcodeGen, or touch
  `BioMotion/`. If a task requires editing Swift code, switch to the parent
  workspace.
- This is **not** a real-time pipeline. OpenSim IK on a long trial takes
  seconds to minutes; we trade speed for accuracy on purpose.
- Marker reconstruction from raw video is **out of scope** for the initial
  cut. We assume the trial already has a `.trc` (the OpenCap demo data does).
  A later pass can wrap `opencap-processing` for end-to-end video → markers.
