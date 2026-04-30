## ADDED Requirements

### Requirement: L/R muscle pair detection

`desktop-pipeline` SHALL identify bilateral muscle pairs in any iOS-exported `<stem>_activations.sto` by matching column-name suffixes `_l` and `_r` (case-insensitive, after stripping the trailing `.activation` token if present). Pairs SHALL be reported in stable alphabetical order by their stem (e.g., `tibant`, `gaslat`, `recfem`). Columns without a matching counterpart SHALL be skipped silently.

#### Scenario: Standard Rajagopal2016 pairs detected

- **GIVEN** an `<stem>_activations.sto` whose columns include `tibant_l`, `tibant_r`, `recfem_l`, `recfem_r`, `bflh_l`
- **WHEN** the bilateral-asymmetry report is run
- **THEN** the pairs `(tibant_l, tibant_r)` and `(recfem_l, recfem_r)` SHALL be reported
- **AND** `bflh_l` SHALL be skipped (no `bflh_r` counterpart)

### Requirement: Per-pair asymmetry metrics

For each detected L/R pair the report SHALL emit, computed across the full time window of the trial:
- `peak_diff` = `|max(left) − max(right)|`
- `rms_diff` = `sqrt(mean((left − right)^2))`
- `asymmetry_index` = `100 * (max(left) − max(right)) / (0.5 * (max(left) + max(right)))` (signed, percent; 0 = symmetric, positive = left-dominant)

Each metric SHALL be reported as a finite floating-point number; if either side has all-zero activations the asymmetry_index SHALL be reported as `NaN` and the row flagged.

#### Scenario: Metric computation on a hand-built fixture

- **GIVEN** `tibant_l(t) = max 0.40` and `tibant_r(t) = max 0.20` over a 1-second trial sampled at 60 Hz
- **WHEN** the report is generated
- **THEN** `peak_diff` SHALL equal `0.20`
- **AND** `asymmetry_index` SHALL be approximately `66.7` (positive, left-dominant)

### Requirement: Pass/fail summary

The report SHALL emit a single `PASS` or `FAIL` summary line based on a configurable peak-asymmetry threshold (default 10 percent). A trial SHALL be marked `FAIL` if any reported pair has `|asymmetry_index| > threshold`, otherwise `PASS`. The threshold SHALL be overridable via a CLI flag `--asymmetry-threshold-pct` on the comparator entry point.

#### Scenario: Trial fails on a single asymmetric pair

- **GIVEN** the default threshold of 10 percent and a trial where `tibant` shows `asymmetry_index = 35` and all other pairs show `|asymmetry_index| < 5`
- **WHEN** the report is generated
- **THEN** the summary line SHALL be `FAIL` and SHALL name `tibant` as a failing pair

#### Scenario: Trial passes when all pairs symmetric

- **GIVEN** the default threshold and a trial where every pair has `|asymmetry_index| <= 8`
- **WHEN** the report is generated
- **THEN** the summary line SHALL be `PASS`

### Requirement: CLI integration

The bilateral-asymmetry report SHALL be opt-in via a `--asymmetry-report` flag on the existing `desktop-pipeline` comparator command. When the flag is absent, the comparator output SHALL be byte-identical to its current behavior. When the flag is present, the asymmetry report SHALL be emitted in addition to (not instead of) the existing tier output.

#### Scenario: Flag absent preserves existing output

- **GIVEN** a comparator invocation that previously emitted tier-1/2/3 output
- **WHEN** the same invocation is run after this change without `--asymmetry-report`
- **THEN** stdout SHALL be byte-identical to the pre-change output

#### Scenario: Flag present adds asymmetry section

- **WHEN** the same invocation is run with `--asymmetry-report`
- **THEN** stdout SHALL contain all the previous tier output
- **AND** SHALL also contain a clearly delimited bilateral-asymmetry section ending with the `PASS` / `FAIL` summary line
