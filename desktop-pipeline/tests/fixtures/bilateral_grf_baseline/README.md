# `bilateral_grf_baseline` — negative-result historical snapshot

This fixture is **not a passing baseline**. It is a frozen record of the
post-implementation state of the `fix-bilateral-grf-bias` change
(2026-04-30) and the evidence that the change's central hypothesis was
**falsified** by its own diagnostic tooling.

See `openspec/changes/fix-bilateral-grf-bias/proposal.md` → "Findings" for
the full write-up. Short version:


| run        | source UUID                            | `idMode`  | FAIL pairs (±10%) |
| ---------- | -------------------------------------- | --------- | ----------------- |
| `withGRF/` | `D57C2A63-6B0C-4705-A704-B991B1518DE0` | `withGRF` | 3 / 40            |
| `noGRF/`   | `357FDEF1-F363-46F2-97EB-C0FA266869A8` | `noGRF`   | **19 / 40**       |


Both runs come from the same OpenCap `run2` walking trial (~31.7 s, 120 fps,
3807 frames, mass 56.07 kg), processed back-to-back on the same iPhone with
the per-foot ground tracking + hysteresis fix already applied. The only
difference is the offline `IDMode` toggle.

Reproduce the asymmetry tables:

```bash
cd desktop-pipeline
PYTHONPATH=src python3 -c "
from pathlib import Path
from biomotion_desktop.compare import asymmetry_report_for_export
for label, folder, stem in [
    ('withGRF', Path('tests/fixtures/bilateral_grf_baseline/withGRF'),
     'D57C2A63-6B0C-4705-A704-B991B1518DE0'),
    ('noGRF',   Path('tests/fixtures/bilateral_grf_baseline/noGRF'),
     '357FDEF1-F363-46F2-97EB-C0FA266869A8'),
]:
    text, _ = asymmetry_report_for_export(folder, stem, threshold_pct=10.0)
    print(f'\n========== {label} ==========')
    print(text)
"
```

## Why this is kept in git

1. **Falsification evidence.** `noGRF` (which bypasses the entire GRF
  estimator the change was built to fix) is *more* asymmetric than
   `withGRF`, not less. This rules out "single-scalar ground ratchet" as
   the dominant source of bilateral asymmetry — future work needs to look
   upstream of `solveIDGRF`.
2. **Saturation-blindness exhibit.** `withGRF` looks deceptively symmetric
  (~30 pairs at `peak_l = peak_r = 1.000`, `asym_idx_pct = +0.0%`) while
   `rms_diff` of 0.3–0.6 reveals large temporal mismatches. This is the
   motivating example for why `bilateral-asymmetry-report` needs an
   `rms_diff` / cross-correlation gate in addition to its peak metric.
3. **Regression anchor.** Any future change that claims to reduce bilateral
  asymmetry must demonstrate improvement against these numbers. Without a
   committed baseline, "did it actually get better?" is unanswerable.

## What's missing (and why)

- The full `_processed.json` (~39 MB each) and `_inverse_dynamics.sto`,
`_kinematics.mot`, `_muscle_forces.sto` are intentionally excluded.
`summary.json` + `activations.sto` is the minimum to reproduce the
asymmetry table.
- A clean bilaterally-symmetric reference trial. The `run2` 31.7 s clip is
long enough to contain non-steady segments (turns, foot pivots) and so
cannot itself act as a "should-be-symmetric" baseline. Picking a clean
3-cycle steady-walk trial is a prerequisite for the follow-up change.

