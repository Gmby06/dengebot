# Changelog

## 1.7.1 (development preview)
- Fixed: `setAlpha()` could be set above the adaptive high alpha (inverted adaptive mode); now rejected while adaptive is enabled. `setAdaptiveEnabled()` returns bool.
- Changed: default adaptive thresholds 0.15/0.10 g -> 0.03/0.02 g (adaptive remains opt-in).
- Changed: extended CSV accepts NaN reference (empty field); added `EDATelemetryEvent` constants.
- Added: `EDAGyroBiasCalibrator`, `GyroBiasCalibrationDemo`, tests, CI target.

## 1.7.0 (development preview)
- Added platform-independent complementary pitch filter.
- Added portable, no-allocation telemetry snapshots and Serial Plotter/CSV formatting.
- Added synthetic sensor example and host regression tests.
- Retained v1.6.1 PID and motor conversion implementation.
- Expanded board compile CI to cover new example.

## 1.6.1 (community preview)
- Improved finite-value portability and CI configuration; motor command conversions and host tests.

### v1.7.0 development revision (adaptive fusion)
- Optional acceleration-magnitude-based adaptive alpha with hysteresis; legacy update remains fixed alpha.
- Optional calibrated gyro bias subtraction; explicit validity on rejected samples.
- Shared finite float helper; optional extended reference/event/sensor timestamp/alpha CSV.
- Reduced synthetic example reporting frequency. PID gains and core algorithm untouched.

## v1.7.2 — Dead-zone Compensation (test candidate)
- Added optional `EDADeadZoneCompensation.h` with `toDCWithDeadZone()`.
- Configurable minimum effective PWM, maximum PWM and zero-command threshold.
- Invalid inputs/configuration fail safe to zero; monotonic and sign-symmetric mapping.
- Existing `toDC()`, PID gains and control behavior remain unchanged by default.
- Added dedicated host regression tests and a non-driving Arduino example.
- Physical robot validation pending; do not treat as a verified release.

- v1.7.2 test: unified `<dengebot.h>` public entry point, preserving separate headers for compatibility.
