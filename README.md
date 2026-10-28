# dengebot v1.7.1 — Adaptive Sensor Fusion & Stability Improvements

**Open-source balancing robot controller library for Arduino-compatible platforms.**

**Current release:** [v1.7.1 — Community Preview](https://github.com/Gmby06/dengebot/releases/tag/v1.7.1)

## Version History

### v1.7.1 — Adaptive Sensor Fusion & Stability Improvements
- Adaptive complementary filtering with configurable alpha values.
- Acceleration-based filtering with hysteresis.
- Gyroscope bias calibration helper.
- Improved sensor input validation and fault handling.
- Expanded diagnostics and telemetry support.
- Additional regression tests and Arduino examples.
- Existing PID and motor-control behavior preserved.

### v1.7.0 — Complementary Filter & Telemetry
- Introduced complementary filtering for accelerometer and gyroscope fusion.
- Added pitch-angle estimation support.
- Added telemetry capabilities for tuning and analysis.
- Extended test coverage.

### v1.6.1 — Community Preview
- Improved motor command safety and validation.
- Added DC and stepper motor command conversion.
- Expanded Arduino platform compilation tests.
- Added host regression tests and sanitizer checks.
- Preserved the existing PID mathematics.

## Supported Platforms

Arduino Uno, Nano, Mega 2560, ESP8266, ESP32 and ESP32-S3.

## Important Note

Adaptive sensor fusion is optional. Simulation results are encouraging, but comprehensive physical robot validation is ongoing. No universal improvement in balancing performance is guaranteed.

For complete technical details, examples, installation instructions and safety guidance, see the sections below.

--------------------------------------------------------------------------------------------------------------

# dengebot v1.6.1 Technical Details

## Changes since 1.6.0
- `EDAMotorCommand.h`: any non-finite command or invalid/non-finite limit now returns a **zero** command (1.6.0 saturated +-Inf commands and returned infinite steps/s for an infinite `maxStepsPerSecond`). `direction==0` now always means zero output; a command that rounds to PWM 0 no longer reports a direction. Ratio-first arithmetic avoids overflow for huge finite values.
- Motor tests expanded (NaN/Inf, invalid limits, `maxPwm`, symmetry, monotonicity, overflow).
- New `examples/MotorCommandDemo` (serial print only, drives no pins). `tests/check_example.sh` checks every example.
- CI: Mega added to the board matrix; both examples compiled; `--library .` replaces the parent-directory path assumption; host tests also run with sanitizers (`EDA_SANITIZE=1`). **Board CI has not been run yet** — pin core versions after the first green run.
- `library.properties` now lists `EDADengeBot.h`; stale source comment cleaned. PID mathematics unchanged.
- MIT license added; GitHub repository URL will be added once the repository exists.

Two-wheel self-balancing robot controller from E.D.A CNC. The **1.5.2 PID mathematics is retained** in this release. The new 1.6 additions are motor-command conversion helpers, host tests, draft board-build CI and community feedback templates. **No physical balance performance improvement is claimed.**

## Install / Kurulum

Arduino IDE → Sketch → Include Library → Add .ZIP Library. Prefer `<EDADengeBot.h>` (`<dengebot.h>` still works for existing sketches). Use `edaDefaultConfig()`, customize values for your physical robot, construct `EDABalancePID`, and call `update(target,pitch,verifiedRate,pitchRate,dtSeconds)` with actual seconds. **Do not drive motors until direction, arming and fault handling have been tested.**

## Platform targets

Uno, Nano, Mega, ESP8266, ESP32 and ESP32-S3. Core is standard C++; the GitHub workflow attempts board compilation on six targets. Board-specific build success must be checked in CI before marking each platform verified.

## Motor support scope

`EDAMotorCommand.h` offers pure conversions from normalized controller output to **DC PWM+direction** and **stepper speed+direction** commands. These are **not motor drivers**: they do not generate STEP pulses, implement acceleration limits, detect lost steps, or provide torque/velocity feedback. NEMA 17 balancing remains **experimental and unvalidated**. Do not treat an open-loop stepper speed mapping as proven stable control.

## Safety

Check `configValid()`, `lastUpdateValid()`, `faulted()` and `firstFault()`. Use `setZeroOutputOnFault(true)` where appropriate, and implement a separate hardware motor disable and explicit re-arm. Zero output is not guaranteed to prevent a falling robot. `clearFault()` does not re-arm a machine. No default PID gains are universally safe.

## Tests

Run `sh tests/run_host_tests.sh`. Host tests cover basic math, saturation, configuration, fault latching, and command conversion. `.github/workflows` includes host and Arduino build checks. Physical testing, motor dead-zone characterization and comparative control benchmarks remain to be completed.

## Community testing

Please open a GitHub issue using the **Robot test report** template with your board, motor, driver, IMU, mass, wheel size, controller settings and measured behavior. Attach logs without credentials. We welcome negative results and reproducible bugs.

## Licensing

Licensed under the **MIT License**; see [LICENSE](LICENSE). For background and attribution to Brett Beauregard’s Arduino PID Library (PID_v1), see [ACKNOWLEDGMENTS.md](ACKNOWLEDGMENTS.md). PID_v1 is not a dependency of dengebot.
