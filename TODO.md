# TODO

## High Priority
- [ ] Auto (Owner: AADIT)
  - [ ] Make path planning reliable.
  - [ ] Choose and standardize on `Choreo` or `PathPlanner`.
  - [ ] Define autonomous goals:
    - [ ] Target number of pieces.
    - [ ] Target cycle time / scoring window.
- [ ] Auto-aim (Owner: PRANESH)
  - [ ] Implement button-triggered hub aim and shoot when flywheel speed is at setpoint.
  - [ ] Finalize shot parameter tuning.
- [ ] Simulation (Owner: TYCHO)
  - [ ] Add autonomous simulation coverage.
  - [ ] Integrate AdvantageKit logging.
- [ ] Swerve bring-up

## Medium Priority
- [ ] Telemetry improvements
  - [ ] Publish key auto/shot metrics to dashboard.
  - [ ] Add alerts for flywheel not-at-speed and heading error.
- [ ] Auto routine library
  - [ ] Build 2-piece and 3-piece baseline routines.
  - [ ] Add starting-position variants for common field setups.

## Low Priority
- [ ] Shoot on the move.
- [ ] Codebase cleanup
  - [ ] Remove stale TODOs/comments in drive + shooter code.
  - [ ] Consolidate constants naming for auto and shot tuning.
- [ ] Documentation
  - [ ] Add/update setup notes for simulation and logging workflow.

## Completed
- [x] Track complete tasks here.
