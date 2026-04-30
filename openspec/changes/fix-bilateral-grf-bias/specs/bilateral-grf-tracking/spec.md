## ADDED Requirements

### Requirement: Per-foot ground height tracking

`NimbleBridge.solveIDGRF` SHALL maintain two independent ground-height estimates `_groundHeightLY` and `_groundHeightRY`, one per foot. Each SHALL be initialized to `(corresponding heel y) − 0.01 m` on the first frame the foot is observed, and SHALL ratchet down (only) to `(heel y) − 0.01 m` on any subsequent frame where the heel goes lower than the current estimate. Neither foot's ground estimate SHALL be influenced by the other foot's heel position.

#### Scenario: Independent ratcheting under asymmetric heel descent

- **GIVEN** a sequence of frames where `calcn_l.y` reaches a minimum of 0.02 m and `calcn_r.y` reaches a minimum of 0.06 m
- **WHEN** `solveIDGRF` is called for each frame in order
- **THEN** `_groundHeightLY` SHALL converge to 0.01 m and `_groundHeightRY` SHALL converge to 0.05 m
- **AND** the right foot's contact detection SHALL use 0.05 m as its ground reference, not 0.01 m

#### Scenario: First-frame initialization

- **GIVEN** a fresh model load with `_groundHeightCalibrated == NO`
- **WHEN** the first `solveIDGRF` call observes `calcn_l.y = 0.10 m` and `calcn_r.y = 0.12 m`
- **THEN** `_groundHeightLY` SHALL be set to 0.09 m and `_groundHeightRY` SHALL be set to 0.11 m on that single call
- **AND** `_groundHeightCalibrated` SHALL transition to `YES`

#### Scenario: Backward compatibility for the UI badge

- **GIVEN** `_groundHeightLY = 0.02 m` and `_groundHeightRY = 0.05 m`
- **WHEN** Swift code reads `bridge.groundHeightY`
- **THEN** the accessor SHALL return `min(_groundHeightLY, _groundHeightRY)` = 0.02 m so the existing UI ground-height display remains meaningful

### Requirement: Hysteresis on per-foot contact detection

Per-foot contact detection in `solveIDGRF` SHALL use hysteresis: a foot transitions from swing to stance when `(calcn_y − ground_y) < CONTACT_ATTACH` (default 0.06 m) and only transitions back from stance to swing when `(calcn_y − ground_y) > CONTACT_RELEASE` (default 0.08 m). The contact state of each foot SHALL persist across frames and SHALL be tracked independently per foot.

#### Scenario: Foot stays in contact through small lift

- **GIVEN** the left foot is currently in stance and `_groundHeightLY = 0.00 m`
- **WHEN** `calcn_l.y` rises to 0.07 m on the next frame
- **THEN** the left foot SHALL remain in stance (hysteresis: 0.07 < 0.08 release threshold)

#### Scenario: Foot transitions to swing when fully clearing release threshold

- **GIVEN** the left foot is currently in stance and `_groundHeightLY = 0.00 m`
- **WHEN** `calcn_l.y` rises to 0.09 m on the next frame
- **THEN** the left foot SHALL transition to swing (0.09 > 0.08 release threshold)

#### Scenario: Independent contact transitions

- **GIVEN** the left foot is in stance and the right foot is in swing
- **WHEN** the right foot's heel drops below `_groundHeightRY + 0.06 m`
- **THEN** the right foot SHALL transition to stance independently of the left foot's state
- **AND** the left foot's contact state SHALL not be affected by the right foot's transition

### Requirement: Symmetric weight distribution under bilateral stance

When both feet are detected in contact on the same frame, `solveIDGRF` SHALL distribute the body-weight wrench guess equally between the two feet (`perFootForce = mass * 9.81 / 2` along world +y). When only one foot is in contact, the full body weight SHALL be assigned to that foot's wrench guess.

#### Scenario: Bilateral stance equal split

- **GIVEN** both feet are in contact on a given frame and the model mass is 70 kg
- **WHEN** `solveIDGRF` builds the wrench guesses
- **THEN** the left and right wrench guesses SHALL each contain a vertical force of `70 * 9.81 / 2 = 343.35 N`

#### Scenario: Single-leg stance full weight

- **GIVEN** only the right foot is in contact on a given frame and the model mass is 70 kg
- **WHEN** `solveIDGRF` builds the wrench guesses
- **THEN** the right foot's wrench guess SHALL contain a vertical force of `70 * 9.81 = 686.7 N` and no left wrench guess SHALL be added
