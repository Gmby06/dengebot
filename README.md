# dengebot v1.7.2 — Dead-Zone Compensation

**Open-source, hardware-independent, Arduino-compatible two-wheel balancing robot control library.**

Developed by **Gmby06**.

dengebot provides reusable balancing control components for Arduino-based robotics projects, including PID control, motor command conversion, sensor fusion, telemetry, gyro bias calibration, and optional dead-zone compensation.

The library is designed for experimentation, education, community testing, and further development.

**Current Release:** [v1.7.2 — Dead-Zone Compensation (Community Preview)](https://github.com/Gmby06/dengebot/releases/tag/v1.7.2)

**License:** MIT

---

## What's New in v1.7.2

### Optional Dead-Zone Compensation

Version 1.7.2 introduces an optional DC motor dead-zone compensation helper.

Some DC motors do not respond to very low PWM values because of mechanical friction, gearbox resistance, and motor-driver characteristics.

The new helper allows developers to configure a minimum effective PWM value while preserving the controller's existing PID behavior.

Features:

- Configurable minimum effective motor PWM.
- Configurable maximum motor PWM.
- Adjustable zero-command threshold.
- Separate compensation settings for left and right motors.
- Monotonic PWM mapping above the threshold.
- Invalid configuration handling.
- Fully optional integration.

**Important:** Dead-zone compensation is not enabled automatically. Existing motor conversion behavior remains unchanged.

### Unified Library Header

Version 1.7.2 introduces a convenient unified header:

```cpp
#include <dengebot.h>
```

This provides access to the core balancing controller, motor command helpers, and optional dead-zone compensation.

Existing individual headers remain available for projects that prefer modular integration.

### New Example

The `DeadZoneDemo` example demonstrates how to configure and use dead-zone compensation.

The example does not drive physical motor GPIO pins.

---

## Compatibility and Testing

Version 1.7.2 was successfully compiled for **18 Arduino board profiles**.

### AVR — 8 Profiles

- Arduino Uno
- Arduino Nano
- Arduino Mega 2560
- Arduino Leonardo
- Arduino Pro Mini
- Arduino Uno WiFi
- Arduino Mega ADK
- Arduino Micro

### ESP32 — 7 Profiles

- ESP32 Dev Module
- ESP32-S2 Dev Module
- ESP32-S3 Dev Module
- ESP32-C3 Dev Module
- ESP32 WROVER Module
- ESP32-S3 Octal WROOM-2
- ESP32 PICO-D4

### ESP8266 — 3 Profiles

- ESP8266
- ESP8285
- ESPDuino ESP-13

**Compilation result: 18/18 PASS**

GitHub Actions automated tests also passed for the v1.7.2 development update.

Dead-zone compensation has additionally been tested on an Arduino Nano-based two-wheel balancing robot.

**Note:** Compilation success does not mean that every board and motor configuration has been physically validated. Hardware testing remains an important part of this community preview.

---

## Installation

### Arduino IDE

1. Download the latest ZIP package from [GitHub Releases](https://github.com/Gmby06/dengebot/releases/tag/v1.7.2).
2. Open Arduino IDE.
3. Select **Sketch > Include Library > Add .ZIP Library**.
4. Select the downloaded library ZIP.
5. Include the library in your sketch.

```cpp
#include <dengebot.h>
```

Alternatively, include individual modules when needed.

### Repository

Source code, examples, tests, and development history are available at:

https://github.com/Gmby06/dengebot

---

## Dead-Zone Compensation Example

```cpp
#include <dengebot.h>

// Example configuration only.
// Measure the actual effective PWM range of your motor.
EDA::DeadZoneConfig leftMotor(48, 170, 0.5f);

const float commandLimit = 100.0f;

void setup() {
  Serial.begin(115200);
}

void loop() {
  const float pidCommand = 12.0f;

  EDA::DCCommand output =
      EDA::toDCWithDeadZone(
          pidCommand,
          commandLimit,
          leftMotor
      );

  Serial.print("Direction: ");
  Serial.print(output.direction);

  Serial.print(" PWM: ");
  Serial.println(output.pwm);

  delay(1000);
}
```

### How It Works

The compensation helper uses three configuration parameters:

- `minEffectivePwm`: Minimum effective PWM above the zero-command threshold.
- `maxPwm`: Maximum allowed PWM output.
- `zeroCommandThreshold`: Command magnitude at or below which motor output remains zero.

The zero-command threshold uses the same units as the controller command, not PWM units.

Above this threshold, the PWM output begins at the configured minimum effective value and increases monotonically toward the maximum.

This creates a discontinuity at the threshold. Depending on the motor, mechanical system, and controller tuning, that discontinuity can introduce unwanted movement or oscillation.

Different motors may require different settings.

**Safety:** Always begin testing with the robot's wheels raised. Use a hardware motor-disable mechanism and appropriate current protection.

This helper does not provide slew-rate limiting, hysteresis, current limiting, or closed-loop stall detection.

---

## Core Library Features

### Balancing PID Controller

The `EDABalancePID` controller includes:

- Configurable proportional, integral, and derivative control.
- Gyro-based derivative support.
- Conditional integral behavior.
- Integral anti-windup.
- Fault handling.
- Optional advanced recovery behavior.
- Configurable controller limits.

### Motor Command Conversion

`EDAMotorCommand.h` provides motor-command conversion utilities for:

- DC motor PWM and direction.
- Stepper motor velocity and direction.
- Optional DC dead-zone compensation in v1.7.2.

These utilities generate commands. They do not directly implement physical motor drivers or STEP pulse generation.

### Complementary Filter

`EDAComplementaryFilter.h` provides lightweight pitch estimation by combining accelerometer angle and gyroscope angular velocity.

Features include:

- Configurable complementary-filter alpha.
- Explicit initialization and reset.
- Finite-input validation.
- Time-step validation.
- Optional adaptive alpha.
- Optional gyroscope bias correction.

The caller must calculate accelerometer pitch and gyroscope pitch rate using consistent axes and units.

This module is not an MPU6050 or MPU6500 hardware driver.

### Telemetry

`EDATelemetry.h` provides structured diagnostic output.

Supported information includes:

- Timestamp.
- Target pitch.
- Measured pitch.
- Gyroscope rate.
- PID output.
- Motor command.
- Loop duration.
- Controller state.
- Fault code.

Output formats include Arduino Serial Plotter and CSV.

Extended CSV output supports optional reference pitch, event codes, sensor timestamps, and the alpha value used by the filter.

### Gyroscope Bias Calibration

`EDAGyroBiasCalibrator.h` supports startup gyro-bias estimation from stationary sensor samples.

Stationarity checks help reject calibration attempts made while the robot is moving.

Calibration should be performed with the robot held still.

---

## Complementary Filter Example

```cpp
#include <EDAComplementaryFilter.h>

EDAComplementaryFilter filter(0.98f);

// After reading calibrated IMU measurements:
//
// if (filter.update(
//       accelPitchDegrees,
//       gyroPitchDegPerSecond,
//       dtSeconds)) {
//
//   float pitch = filter.pitchDegrees();
// }
```

The first valid reading initializes the estimate from accelerometer pitch.

Subsequent readings combine the integrated gyroscope rate with accelerometer pitch:

`angle = alpha * (previousAngle + gyroRate * dt) + (1 - alpha) * accelAngle`

The filter accepts a time step between 0.0001 and 0.25 seconds.

The PID controller has its own update-time requirements of 0.004 to 0.050 seconds.

Invalid sensor readings must be handled safely by the calling application.

---

## Optional Adaptive Complementary Filter

The adaptive filter is experimental and disabled by default.

It can adjust the gyroscope weighting when acceleration magnitude suggests additional non-gravitational acceleration.

Example:

```cpp
EDAComplementaryFilter filter(0.98f);

filter.configureAdaptive(
    0.998f,
    0.03f,
    0.02f
);

filter.setAdaptiveEnabled(true);

filter.setGyroBias(
    calibratedGyroBiasDegPerSec
);

// With fresh IMU measurements:
//
// if (!filter.update(
//       accelPitchDeg,
//       gyroPitchRateDps,
//       dtSeconds,
//       accelMagnitudeG)) {
//
//   // Disable motor commands.
// }
```

The thresholds shown are experimental simulation candidates, not universal hardware defaults.

Motor vibration, IMU noise, mounting orientation, and sensor calibration can significantly affect filter performance.

The three-argument `update()` method continues to use fixed alpha for backward compatibility.

The adaptive acceleration-magnitude heuristic cannot detect every accelerometer angle error.

---

## Telemetry and Diagnostics

The telemetry module can be used to monitor controller behavior during development.

```cpp
EDATelemetry::printPlotter(Serial, frame);
```

or:

```cpp
EDATelemetry::printCSV(Serial, frame);
```

The caller determines the reporting frequency.

Serial printing may block execution, so telemetry should be scheduled at a lower rate than the main balancing control loop.

### Extended CSV

```cpp
EDATelemetry::printCSVExtended(
    out,
    frame,
    pitchRefDeg,
    eventCode,
    sensorTimeMs,
    filter.usedAlpha()
);
```

Extended telemetry can help analyze simulation and physical experiments.

Only supply a reference pitch when a valid reference is available.

---

## Gyroscope Bias Calibration Example

```cpp
EDAGyroBiasCalibrator cal(100);

// Collect samples while the robot is stationary.
//
// cal.addSample(rawGyroPitchRateDps);
//
// if (cal.ready() && cal.stationary()) {
//   filter.setGyroBias(cal.bias());
// }
```

Gyroscope bias can create steady-state angle error, especially when the complementary filter uses a high alpha.

Recalibration after sensor warm-up may improve results.

---

## Version History

### v1.7.2 — Dead-Zone Compensation

- Optional DC motor dead-zone compensation.
- Unified `dengebot.h` public header.
- New `DeadZoneDemo` example.
- Backward-compatible motor command behavior.
- Successful compilation across 18 Arduino board profiles.
- Automated GitHub Actions tests passed.

### v1.7.1 — Sensor Fusion & Telemetry Improvements

- Adaptive-filter configuration fixes.
- Updated experimental adaptive acceleration thresholds.
- Extended CSV handling for unavailable reference pitch.
- Suggested telemetry event codes.
- Added gyroscope bias calibration helper.
- Added `GyroBiasCalibrationDemo`.
- Existing PID and fixed-alpha filter behavior preserved.

### v1.7.0 — Complementary Filter & Telemetry

- Introduced `EDAComplementaryFilter.h`.
- Introduced `EDATelemetry.h`.
- Added synthetic-input filter and telemetry examples.
- Expanded host tests and Arduino CI compilation coverage.
- Preserved existing PID control behavior.

### v1.6.x — Controller and Platform Compatibility

- Core balancing PID improvements.
- Motor command conversion utilities.
- Fault-handling improvements.
- Host regression testing.
- Arduino board compilation workflows.
- Community Preview releases.

---

## Safety and Hardware Limitations

dengebot is an experimental robotics control library.

A balancing robot can move unexpectedly, fall, or damage connected hardware if improperly configured.

Developers should implement:

- Hardware motor disable.
- Manual arming and emergency stop.
- Sensor freshness checks.
- Correct motor direction and sign conventions.
- Fall-angle shutdown.
- Fault handling.
- Appropriate power and current protection.

Motor outputs remain disabled in the library's demonstration examples unless an application explicitly implements motor control.

**Successful compilation or simulation does not guarantee stable physical balancing.**

Stepper motor behavior and additional hardware configurations require further physical validation.

---

## Testing and Development

Host regression tests are included in the repository.

Run:

```bash
sh tests/run_host_tests.sh
```

Where supported, sanitizer testing can be enabled with:

```bash
EDA_SANITIZE=1 sh tests/run_host_tests.sh
```

The repository also includes GitHub Actions workflows for automated testing and Arduino compilation.

Testing should cover both software behavior and real hardware conditions.

---

## Community Testing and Contributions

We welcome feedback from Arduino developers, robotics enthusiasts, students, and researchers.

If you test dengebot, please share:

- Arduino or ESP board model.
- IMU sensor model.
- Motor driver.
- DC or stepper motor specifications.
- Controller configuration.
- Complementary-filter settings.
- Dead-zone compensation settings, if used.
- Observed results and any problems.

Feedback helps identify compatibility issues, improve documentation, and guide future development.

**Report bugs, test results, and suggestions:**

[GitHub Issues](https://github.com/Gmby06/dengebot/issues)

**Download the latest release:**

[dengebot v1.7.2](https://github.com/Gmby06/dengebot/releases/tag/v1.7.2)

---

## License and Attribution

dengebot is distributed under the **MIT License**.

See [LICENSE](LICENSE) for details.

See [ACKNOWLEDGMENTS.md](ACKNOWLEDGMENTS.md) for PID_v1 community attribution.

---

**Gmby06 — dengebot**

*Open-source balancing robot control development.*

*Community testing and contributions are welcome.*

-----------------------------------------------------------


# dengebot v1.7.1 — Sensor Fusion & Telemetry (development preview)

Hardware-independent, experimental Arduino-compatible two-wheel balancing control library.

## What's new in v1.7.1
- **Adaptive-filter fixes:** `setAlpha()` is rejected while adaptive mode is on if it would exceed the high-acceleration alpha (previously the "high" mode could end up trusting the gyro *less*); `setAdaptiveEnabled(true)` returns `false` in that situation.
- **Default adaptive thresholds are now 0.03 g / 0.02 g** (were 0.15 g / 0.10 g). With the old defaults the mode never triggered in 50-run simulations: horizontal acceleration is perpendicular to gravity, so a 0.1 g acceleration changes `|a|` by only ~0.005 g. Adaptive mode is still **off by default**; the 3-argument `update()` is unchanged. The new thresholds are **simulation candidates** assuming ~0.01 g magnitude noise; measure your own IMU (motor vibration can need higher thresholds).
- `EDATelemetry::printCSVExtended()` accepts `NaN` as "no reference" and prints an empty `pitch_ref_deg` field, so the extended format is usable on hardware without a reference. Suggested `EDATelemetryEvent` codes added (>=100 reserved for the application).
- New `EDAGyroBiasCalibrator.h`: startup gyro-bias estimate from held-still samples with stationarity checks; `examples/GyroBiasCalibrationDemo` (synthetic input only).
- PID, motor conversion and fixed-alpha filter behaviour are unchanged (bit-identical in the 50-run simulation).

## What's new in v1.7.0
- `EDAComplementaryFilter.h`: lightweight pitch complementary filter with configurable alpha (0..1), explicit reset/initialization, finite-input and time-step checks. **Caller must calculate accelerometer pitch and gyro pitch rate in consistent axes/units**; this is not an MPU driver.
- `EDATelemetry.h`: no-allocation telemetry frame with Arduino Serial Plotter labeled output and CSV output for future tools. User controls reporting frequency; printing can block, so avoid excessive reporting in time-critical loops.
- `examples/FilterTelemetryDemo`: **synthetic inputs only**, no motor control or physical balancing.
- Host tests (including sanitizers) and Arduino CI example compile targets expanded. PID control behavior and motor command logic are unchanged; the PID source previously received a finite-value portability adjustment.

## Install
Arduino IDE > Sketch > Include Library > Add .ZIP Library. Include `EDADengeBot.h`, `EDAComplementaryFilter.h` or `EDATelemetry.h` as needed. Existing v1.6.1 sketches should remain source-compatible.

## Complementary filter example
```cpp
#include <EDAComplementaryFilter.h>
EDAComplementaryFilter filter(0.98f);
// In loop, after reading a calibrated IMU:
// if (filter.update(accelPitchDegrees, gyroPitchDegPerSecond, dtSeconds)) {
//   float pitch = filter.pitchDegrees();
// }
```
First valid reading initializes the estimate from accelerometer pitch. Subsequent readings use `alpha*(previousAngle + gyroRate*dt) + (1-alpha)*accelPitch`. The filter accepts dt between 0.0001 and 0.25 seconds; this is **not** the PID update time range (PID requires 0.004..0.050 s). Reject invalid sensor readings and disable motor drive through the caller's safety logic.

## Telemetry
`EDATelemetryFrame` stores timestamp, target pitch, measured pitch, gyro rate, PID output, motor command, loop duration, state and fault code. Call `EDATelemetry::printPlotter(Serial, frame)` or `EDATelemetry::printCSV(Serial, frame)`; CSV header is `EDATelemetry::csvHeader()`. Plotter and CSV are separate formats. For performance, print at a lower rate than the PID loop. The caller assigns state (0 stopped, 1 running, 2 fault) and fault code.

## Existing capabilities
`EDABalancePID` exposes configurable gyro-based PID, optional advanced recovery, anti-windup and fault status. `EDAMotorCommand.h` provides pure DC PWM/direction and stepper velocity/direction conversions; **not physical motor drivers**. No STEP pulse generator is provided. Physical balancing and NEMA17 stepper behavior remain unvalidated.

## Supported compilation targets
GitHub Actions is configured to compile examples on Arduino Uno, Nano, Mega, ESP8266 NodeMCU, ESP32 and ESP32-S3. The new v1.7.0 CI workflow must run successfully on GitHub before claiming verified board compilation.

## Safety
Motor outputs remain disabled in examples. Users must implement hardware motor disable, manual arming, sensor freshness checks, correct sign conventions, fall-angle shutdown and fault handling. A successful software test does not prove physical safety or stable balancing.

## Testing
Run `sh tests/run_host_tests.sh` (or `EDA_SANITIZE=1 sh tests/run_host_tests.sh` with supported sanitizers). Report hardware results via GitHub Issues.

## License and attribution
MIT License (see `LICENSE`). See `ACKNOWLEDGMENTS.md` for PID_v1 community attribution.

## Optional adaptive filter (experimental)
```cpp
EDAComplementaryFilter filter(0.98f);
filter.configureAdaptive(0.998f, 0.15f, 0.10f); // enter/exit |magnitude - 1g|
filter.setAdaptiveEnabled(true);
filter.setGyroBias(calibratedGyroBiasDegPerSec); // measure while stationary
// after reading the IMU, with a fresh sample and measured acceleration magnitude:
if (!filter.update(accelPitchDeg, gyroPitchRateDps, dtSeconds, accelMagnitudeG)) {
  // Stop motor commands; last angle is stale until next valid update.
}
```
The 3-argument `update()` always uses fixed alpha for backward compatibility.
Acceleration magnitude is in g; compute it from calibrated X/Y/Z acceleration,
not from the pitch angle. The 1g deviation heuristic cannot detect every
accelerometer angle error. Hysteresis thresholds and alpha values are
**simulation candidates**, not universal hardware defaults. `usedAlpha()`
reports the last successful fusion weight. The first valid sample initializes
from the accelerometer. `valid()` becomes false after invalid inputs; never
use the retained angle to drive motors while invalid. `reset()` retains bias
calibration and filter configuration but clears angle/validity.

### Optional extended CSV for simulations
`EDATelemetry::printCSVExtended(out, frame, pitchRefDeg, eventCode,
sensorTimeMs, filter.usedAlpha())` adds reference pitch, integer event code,
sensor sample timestamp and actual alpha. Header: `csvExtendedHeader()`.
The existing `printCSV()` and `csvHeader()` remain unchanged. Only supply a
reference pitch when you actually have one; do not invent ground truth on a
physical robot. Printing can block and must be scheduled away from critical
control timing.

### Gyro bias calibration (recommended with high alpha)
```cpp
EDAGyroBiasCalibrator cal(100);            // ~1 s at 100 Hz
// robot held perfectly still: cal.addSample(rawGyroPitchRateDps) every sample
// if (cal.ready() && cal.stationary()) filter.setGyroBias(cal.bias());
// else: robot moved or sensor fault -> do not apply, retry
```
Steady-state angle error from an uncorrected gyro bias `b` is about `alpha*dt/(1-alpha)*b`
(0.49 s*b at alpha 0.98, ~2 s*b at 0.995, dt = 10 ms). Re-calibrate after warm-up.

### Extended CSV event codes
`EDA_EVENT_NONE=0, EDA_EVENT_DISTURBANCE=1, EDA_EVENT_FAULT=2, EDA_EVENT_ARMED=3,
EDA_EVENT_DISARMED=4`; values >= 100 (`EDA_EVENT_USER_BASE`) are free for the application.

### Evidence limitations
The evidence for adaptive alpha comes from simulated inverted-pendulum runs with an assumed
accelerometer-contamination model; a trigger driven by the *true* motor acceleration (not
measurable) was strongest, while a magnitude-based trigger helps only with low thresholds.
Physical validation is still required. The simulation's Kp/Kd values are **not** universal defaults.
Simulation also showed that Kp=40/Kd=6 is near the stability limit (very little tolerance to
sensor latency or gain error); Kp~20/Kd~3 was far more tolerant. Re-tune on your robot.

## v1.7.2 candidate: optional DC dead-zone compensation

Include `<EDADeadZoneCompensation.h>` and call:

```cpp
EDA::DeadZoneConfig left(48, 170, 0.5f); // example only, measure your motor
EDA::DCCommand output = EDA::toDCWithDeadZone(pidCommand, 100.0f, left);
```

`zeroCommandThreshold` is expressed in the same units as `pidCommand`, **not PWM**.
The mapping is off at and below that threshold; above it, PWM begins at
`minEffectivePwm` and rises monotonically to `maxPwm`. This introduces a
**discontinuity** at the threshold. It is intended as an opt-in baseline for
physical tests, not a cure for motor oscillation. Use separate left/right
configurations when necessary. Ensure `minEffectivePwm <= maxPwm` and
`zeroCommandThreshold < commandLimit`. Invalid values produce a zero command.

**Safety:** test with wheels raised first; use a hardware motor-disable path.
This helper does not provide slew-rate limiting, hysteresis, current limiting,
or closed-loop stall detection. Existing `toDC()` behavior is unchanged.


### v1.7.2 unified header
Include only `#include <dengebot.h>` for the core, motor command helpers and optional dead-zone mapping. Including the header does **not** enable optional behavior; use `EDA::toDCWithDeadZone(...)` explicitly.
