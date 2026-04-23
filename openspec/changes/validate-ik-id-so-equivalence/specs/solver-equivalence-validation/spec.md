## ADDED Requirements

### Requirement: Desktop pipeline SHALL provide an `equivalence` CLI command

The `desktop-pipeline` package SHALL expose a new CLI subcommand `biomotion-desktop equivalence` that diffs an iOS export folder against a desktop output folder across three solver stages — IK, ID, and SO — using stage-appropriate metrics. The existing `compare` and `run` subcommands SHALL be unchanged.

#### Scenario: Run all stages with defaults

- **WHEN** the user runs `biomotion-desktop equivalence --desktop <dir> --ios <dir>`
- **THEN** the command computes IK, ID, and SO equivalence metrics, writes them to `<desktop>/equivalence.json`, prints a one-line summary per stage with pass/fail per the documented thresholds, and exits 0 if all stages pass

#### Scenario: Restrict to a single stage

- **WHEN** the user passes `--stages ik`
- **THEN** only the IK stage runs and only IK metrics appear in `equivalence.json`; the ID and SO sections are absent

#### Scenario: Existing `compare` is untouched

- **WHEN** the user runs `biomotion-desktop compare --desktop <dir> --ios <dir>`
- **THEN** the command produces the same `compare.json` it produced before this change, with no new fields and no new behavior

### Requirement: IK equivalence SHALL use phase-compensated per-DOF kinematics metrics

The IK stage SHALL load the iOS-side joint angles (from `kinematics.mot` or the equivalent series in the iOS export) and the desktop-side joint angles, resample both onto the desktop time grid, and for each model DOF compute: RMSE in degrees, max\_abs in degrees, R² **after best-integer-sample shift search** in `[-PHASE_SEARCH_MAX_MS, +PHASE_SEARCH_MAX_MS]`, and the recovered `phase_lag_ms`. The stage SHALL pass when, for every actuated DOF whose desktop range exceeds 5°, RMSE ≤ 2° and R² ≥ 0.95 after compensation.

#### Scenario: Phase-aligned DOF passes

- **WHEN** an actuated DOF has range > 5° and post-shift RMSE ≤ 2° with R² ≥ 0.95
- **THEN** that DOF's per-channel record sets `pass = true`

#### Scenario: Low-range DOF is reported but not gated

- **WHEN** a DOF has desktop range ≤ 5°
- **THEN** the per-channel record sets `pass = null` and `note = "low_range"`, and the DOF is excluded from the stage-level pass aggregation

#### Scenario: Phase lag is recovered and reported

- **WHEN** the iOS series is uniformly delayed relative to desktop within ±100 ms (e.g. SG group delay)
- **THEN** the per-channel `phase_lag_ms` reports the recovered offset, and the stage prints a one-line "median phase lag = X ms" alongside the pass/fail summary

### Requirement: ID equivalence SHALL refuse to run unless both sides are no-GRF

The ID stage SHALL read the iOS export's `idMode` field and the desktop run's metadata. If either side ran with GRF, the stage SHALL emit a warning, mark itself as `skipped`, and `equivalence.json` SHALL contain `id.status == "skipped_grf_mismatch"`. The stage SHALL run only when both sides are no-GRF.

#### Scenario: iOS export with `idMode = "withGRF"` against any desktop

- **WHEN** the iOS summary says `idMode == "withGRF"`
- **THEN** the ID stage is skipped, `equivalence.json` records `id.status == "skipped_grf_mismatch"`, and the CLI exits non-zero only if the user explicitly passed `--require-id`

#### Scenario: Both sides no-GRF

- **WHEN** the iOS summary says `idMode == "noGRF"` and the desktop run is no-GRF (the default for `opensim_id`)
- **THEN** the ID stage runs and produces per-DOF metrics

### Requirement: ID equivalence SHALL mask the floating-root DOFs and gate on actuated DOFs

When the ID stage runs, it SHALL exclude the 6 floating-root DOFs from the stage-level pass criteria, reporting them in a separate `id.root_residual` block (RMSE in N or Nm, depending on translational vs rotational). For actuated DOFs, the stage SHALL compute per-DOF RMSE in Nm/kg-of-body-mass and R². The stage SHALL pass when, for every actuated DOF whose desktop peak |τ| exceeds 0.5 Nm/kg, RMSE ≤ 0.3 Nm/kg and R² ≥ 0.90.

#### Scenario: Root DOF reported but not gated

- **WHEN** the comparison includes a floating-root DOF (`pelvis_tx`, `pelvis_ty`, `pelvis_tz`, `pelvis_tilt`, `pelvis_list`, `pelvis_rotation`)
- **THEN** that DOF appears under `id.root_residual` and not under `id.actuated`, and contributes nothing to the stage pass result

#### Scenario: Low-effort DOF excluded from gating

- **WHEN** an actuated DOF has desktop peak |τ| ≤ 0.5 Nm/kg
- **THEN** the per-channel record sets `pass = null` and `note = "low_effort"`, and the DOF is excluded from the stage-level pass aggregation

### Requirement: SO equivalence SHALL replace single-number R² with biomechanical metrics

The SO stage SHALL NOT use per-frame Pearson R² as a pass criterion. Instead, for each muscle present in both sides' activation files, it SHALL report:
- onset\_diff\_ms, peak\_time\_diff\_ms, offset\_diff\_ms (existing Tier-3 metrics from `compare.py`)
- saturation\_rate (fraction of frames where activation ≥ 0.95) on each side
- agonist/antagonist co-contraction Pearson coefficient between configured muscle pairs
- `Σa²` integrated over each detected gait cycle on each side (effort proxy)

Pass criteria SHALL be reported per metric (no single roll-up boolean):
- timing: |onset/peak/offset diff| ≤ 50 ms on muscles with at least one onset event in both sides
- saturation\_rate delta ≤ 0.10 (absolute)
- co-contraction Pearson ≥ 0.7 on configured pairs
- `Σa²` per cycle within ±25% (relative)

#### Scenario: Per-muscle timing within tolerance

- **WHEN** a muscle has a clear onset on both sides
- **THEN** the muscle record reports onset\_diff\_ms, peak\_time\_diff\_ms, offset\_diff\_ms, and a per-metric `pass` boolean against the 50 ms threshold

#### Scenario: Saturation-rate divergence

- **WHEN** iOS shows a saturation rate of 0.40 on a muscle and desktop shows 0.05
- **THEN** the muscle record reports `saturation_rate_delta = 0.35` and `saturation_pass = false`

#### Scenario: Configured antagonist pair

- **WHEN** the user provides `--antagonist-pairs "tibant_r:gasmed_r,tibant_l:gasmed_l"`
- **THEN** for each pair the stage reports a Pearson coefficient on each side and the absolute difference, with `co_contraction_pass = (|delta| ≤ 0.3) and (min_pearson ≥ 0.7)`

#### Scenario: SO stage prints per-metric pass counts, not a single number

- **WHEN** the SO stage finishes
- **THEN** the CLI prints "SO: timing N/M pass · saturation N/M pass · co-contraction N/M pass · effort N/M pass" and writes the same counts to `equivalence.json`

### Requirement: Equivalence run SHALL be invocable from a regression script

The repository SHALL provide a script `desktop-pipeline/scripts/validate_demo.sh` that:
- runs the desktop pipeline on a checked-in OpenCap fixture
- consumes a checked-in iOS export fixture committed under `desktop-pipeline/fixtures/ios/<trial>-trc/` and `<trial>-mot/`
- invokes `biomotion-desktop equivalence` with documented flags
- exits non-zero if any of the per-stage pass criteria fail

#### Scenario: All stages pass on the fixture

- **WHEN** the script is run with the unmodified fixture and unmodified solver code
- **THEN** the script exits 0 and `equivalence.json` reports `ik.status == "passed"`, `id.status == "passed"`, and SO per-metric pass counts as expected

#### Scenario: A regression breaks IK equivalence

- **WHEN** an iOS-side change shifts a DOF's RMSE above 2° or R² below 0.95 on the fixture
- **THEN** the script exits non-zero and prints which DOFs caused the failure

### Requirement: Comparison semantics SHALL be documented

The repository SHALL include `desktop-pipeline/docs/comparison_semantics.md` describing, for each of the three stages: what is being compared, why the metrics are appropriate, what each pass threshold means, and the known sources of legitimate disagreement (filter group delay, GRF estimation, muscle-model differences). The document SHALL link to the existing `OPEN_ISSUES.md`.

#### Scenario: Doc is present and linked from desktop README

- **WHEN** a reader navigates to `desktop-pipeline/`
- **THEN** the README links to `docs/comparison_semantics.md`, and the document covers IK, ID, and SO sections in that order
