## v1.7.6 — DEVELOPMENT / NOT RELEASED (STEP/DIR demos)

- Added separate classic-ESP32 motor-only test sketches: `ESP32_A4988_Demo` and `ESP32_DRV8825_Demo`, plus informational driver timing profiles. Neither sketch implements balance control.
- FIX4: resolved strict `-Werror=reorder` build warning in `EDAStepDirLEDC_ESP32.h` by aligning member initialization with declaration order (already present in this input archive).
- Removed the redundant nested `PwmCalibrationDemo` example directory (already clean in this input archive).
- Release preparation: aligned README STEP/DIR GPIO map with sketches (18/19/23/5, EN 33), updated driver demo instructions, and added both new demos to board-compile CI.
- No change to PID algorithms, DC motor control, legacy safety behavior, or STEP/DIR runtime backend in this preparation pass.
- Host tests are local software tests, not hardware validation. Arduino board compilation and GitHub Actions are pending independent confirmation.

## v1.7.6 — DEVELOPMENT / NOT RELEASED (phase 1)

- Added an optional, architecture-neutral two-channel STEP/DIR pulse scheduler (`EDAStepDirController.h`).
- Added ESP32-WROOM motor-only bench test (`ESP32_StepDirFirstTest`) using hardware LEDC (`EDAStepDirLEDC_ESP32.h`). No IMU or PID changes.
- FIX3: LEDC adapter no longer toggles ENABLE on a pure frequency change (only on start / direction reversal); a single motor's change no longer restarts the other.
- FIX3: example ramps at 20000 STEP/s^2 from 200 STEP/s (was 5000 STEP/s jumps every 50 ms); rate table gains 200/500/1000; hold time counts after the ramp.
- FIX3: `StepDirController::setSpeeds()` clamps over-limit speeds instead of rejecting them (a saturated PID output no longer ends in a command timeout). Non-finite input is still rejected.
- FIX3: `EDASafetyGuard` header documents timestamp order (take `sampleMs` before `nowMs`).
- FIX3: host test `ledc_adapter` (fake ESP32 core, 2.x and 3.x API paths); CI compiles the ESP32 example on core 2.0.17 and 3.x.
- Added synthetic frequency/direction/watchdog tests; updated README stale CI statements.
- Not validated on real motors or oscilloscopes; remaining v1.7.6 scope is not implemented.

# Changelog

## v1.7.5 — Safe PWM Motor Calibration (Community Preview)
### Review fixes and release adjustments
- `PwmCalibration::begin()` now invalidates the stored result of the selected wheel/direction (new `PwmCalibrationResults::clear()`); an aborted or timed-out re-run no longer leaves the old value confirmed. Refused `begin()` calls change nothing; other slots are kept.
- `PwmCalibration::valid()` rejects `idleTimeoutMs` / `maxSessionMs` above `0x7fffffff` (the wrap-safe comparison treats larger differences as errors). `EDASafetyConfig::maxSampleAgeMs` has the same upper bound in `EDASafetyGuard::validConfig()`.
- `PwmCalibrationResults::has()/get()` return `false`/`0` for out-of-range wheel or direction values instead of indexing out of bounds.
- `EDABalancePID::reset(initialRate)` replaces a non-finite seed with 0, so `reset(NAN)` can no longer poison the filter state.
- `EDASafetyGuard` reports an invalid configuration as the new `EDASafetyFault::InvalidConfig` (appended; existing values unchanged) instead of `InvalidSensor`. Behavior is unchanged: the guard stays faulted and never arms.
- `EDAGyroBiasCalibrator::stdDev()` cannot return NaN from a tiny negative rounding residue of the running variance.
- `PwmCalibrationDemo` is now active only on Nano / Uno / Mega; every other board (other AVR, Gemma/ATtiny85, ESP) compiles an inert sketch.
- `GyroBiasCalibrationDemo` calibrates once at startup (retries only after a rejection) instead of looping forever.
- Tests: re-run invalidation, config upper bounds, invalid enums, `reset(NAN/Inf)`, `InvalidConfig`, per-board active/inert demo checks (host stubs; no real board compile).
- CI: Arduino core versions pinned (`arduino:avr@1.8.6`, `esp8266:esp8266@3.1.2`, `esp32:esp32@2.0.17`) and a Gemma (`adafruit:avr:gemma`) compile job for the inert PWM demo. **These workflow changes could not be run here; verify them on the first CI run.**
- README: documented the symmetric dead-zone caveat and the direct-PID `zeroOutputOnFault` note.
- Not changed on purpose: the software STOP still cannot cut power if the main loop hangs (documented hardware requirement); `zeroOutputOnFault` default stays OFF for compatibility.

- Added optional `EDAPwmCalibration.h`: operator-confirmed, encoder-free PWM pulses per wheel and direction. Existing PID, safety guard and dead-zone code are not modified.
- Bounded operation: `command(nowMs)` refuses to drive past the pulse time even if `tick()` is late (the loop must still run); the operator has an idle timeout (default 15 s, counted only while READY) plus a hard session cap (default 300 s). New fault codes `IdleTimeout` and `Aborted`.
- `suggestDeadZoneMinPwm()` returns a conservative starting value (lower confirmed direction minus a margin). It is never applied automatically.
- `PwmCalibrationDemo`: Nano / Uno / Mega only (an inert example on other boards); writes to the driver only when the command changes; a key pressed during a pulse cuts the output and aborts that pulse instead of resuming it.
- Tests: `check_example.sh` stub extended and run for the AVR and non-AVR paths; `pwm_calibration` unit tests (deterministic cases, random operation sequences, `millis()` wrap); `demo_harness` runs the demo sketch itself against a fake Arduino (pulse length, mid-pulse interruptions, write-on-change, idle timeout, ramp, other wheel stays off); `demo_inert_nonavr` checks that no pin is touched on non-AVR; `pwm_demo_safety` is a library-level abort check only.
- `library.properties` is set to 1.7.5 for this release package. GitHub Actions runs must be verified after upload.
- Existing PID control mathematics unchanged; additive SafetyGuard fault enum and defensive checks retain previous API behavior. On-device Nano/L298N breakaway testing was reported for four motor/direction combinations; comprehensive safety, PWM waveform and MCU-stall hardware checks remain unverified.

- Release demo pulse changed from 120 ms to **150 ms**; the library configuration default remains **120 ms**. The 11-scenario demo harness was adjusted accordingly.

## v1.7.4 candidate refinement — target-aware safe re-arming
- Added source-compatible START/requestStart overloads accepting target pitch and mutable PID.
- Explicit START checks optional target-relative fall limit before arming; an unsafe request leaves a latched FAULT untouched.
- Successful target-aware START resets PID immediately; original signatures retain their deferred-reset behavior for compatibility.
- Added host regression tests and updated SafetyGuardDemo/README; no PID algorithm changes.
- GitHub Actions board compilation and real-robot validation of this refinement remain pending.
- Legacy 5-argument START APIs now fail closed if target-relative limit is enabled (cannot validate a missing target); use the 6-argument API.
- Target-aware START now validates before changing FAULT state and resets PID immediately.

## v1.7.4 — Safety Guard Fixes (Community Preview)
- `EDASafetyGuard`: the first `update()` after every successful `start()`/`requestStart()` now calls `pid.reset()`, so stale integral/filter/recovery state is not carried across STOP or FAULT.
- `EDASafetyGuard::setMaxAbsErrorDeg()` / `maxAbsErrorDeg()`: optional fall limit relative to the balance target; disabled by default, absolute limit unchanged.
- `<dengebot.h>` now includes every module (filter, telemetry, gyro-bias calibrator added); its include guard no longer shares a name with `EDADengeBot.h`.
- CI compiles all six examples; added regression tests (stale-state re-arm, target-relative fall limit, unified header).
- Removed the stale v1.7.1 `dengebot/` subfolder from the repository.
- PWM threshold calibration moved from v1.7.4 to v1.7.5. PID mathematics unchanged. Physical validation of this release is pending.

## v1.7.3 — STOP/FAULT Safety Update (Community Preview)
- Added optional `EDASafetyGuard` for initial STOP, explicit START/arming, latched FAULT handling, sensor freshness/validity checks, and fall-angle protection.
- Added `requestStart()` for explicit operator-initiated recovery from a latched FAULT when safe-start conditions are satisfied, without rebooting the controller. Recovery is not automatic.
- Added safety demonstration and host regression tests. Existing PID control mathematics and APIs remain unchanged.
- Compilation was reported successful across multiple Arduino AVR, ESP8266, and ESP32 board profiles.
- On one Arduino Nano balancing-robot integration, START/STOP, fall-angle-triggered motor shutdown, and FAULT recovery after returning upright were physically tested successfully.
- Safety behavior depends on correct application integration: the application must enforce motor-driver EN/PWM shutdown. Physical validation on one robot does not establish safety on all boards, sensors, or motor drivers.
- PWM threshold calibration is planned for v1.7.4 and is not part of this release.

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


## v1.7.6 development FIX2 (unreleased)

- Added experimental classic-ESP32 LEDC STEP/DIR hardware adapter (two independent channels, 0 and 2), keeping the generic v1.7.5 PID and motor mapping unchanged.
- Replaced software-ISR STEP output in the ESP32-only bench example. Raised example test stages to 2000 / 5000 / 10000 / 20000 / 40000 STEP/s without changing the portable scheduler's rate contract.
- Kept 2-second user-triggered bench pulse test, per-direction manual selection, hardware ENABLE pin, bounded command refresh, and optional speed steps for the higher tiers.
- Physical rotation/frequency verification is still pending; no claim of A4988, NEMA17 or industrial-driver compatibility in real hardware.
