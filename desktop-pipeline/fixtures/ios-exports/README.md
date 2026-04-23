# iOS export fixtures

Each subdirectory here is one untouched export from the iOS app's **Offline →
Export** button (`OfflineAnalysisExporter.export(...)`). Drop the unzipped
`OfflineExport-*` folder in, rename it to the trial stem (e.g. `run2`), and
the comparator picks it up.

## Why files are named with a UUID, not the trial name

The iOS app names every export file after a per-run UUID, e.g.

```
46A017D9-568C-409D-B384-22DDFFE0592A_kinematics.mot
46A017D9-568C-409D-B384-22DDFFE0592A_inverse_dynamics.sto
46A017D9-568C-409D-B384-22DDFFE0592A_activations.sto
46A017D9-568C-409D-B384-22DDFFE0592A_muscle_forces.sto
46A017D9-568C-409D-B384-22DDFFE0592A_summary.json
```

The desktop pipeline names files after the trial config (`run2_kinematics.mot`,
…). The CLI handles the mismatch via `--ios-stem`:

```bash
python -m biomotion_desktop.cli compare \
  --desktop desktop-pipeline/results/run2 \
  --ios     desktop-pipeline/fixtures/ios-exports/run2 \
  --stem    run2 \
  --ios-stem 46A017D9-568C-409D-B384-22DDFFE0592A
```

If you omit `--stem` / `--ios-stem`, both are autodetected from the matching
folder's `*_summary.json`.

## Layout

```
fixtures/ios-exports/
├── README.md                 ← this file
└── <trial>/                  ← one folder per trial; folder name = desktop stem
    ├── <uuid>_kinematics.mot
    ├── <uuid>_inverse_dynamics.sto
    ├── <uuid>_activations.sto
    ├── <uuid>_muscle_forces.sto
    └── <uuid>_summary.json
```

These fixtures *are* checked into git so the comparator has reproducible
ground-truth iOS data without depending on a phone being plugged in.
