# dengebot

**Arduino-compatible control library for two-wheel self-balancing robots.**

**Current release: v1.7.4 — Safety Guard Fixes (Community Preview)**

[Download v1.7.4](https://github.com/Gmby06/dengebot/releases/tag/v1.7.4) · [Examples](examples) · [Changelog](CHANGELOG.md) · [Report an issue](https://github.com/Gmby06/dengebot/issues) · [MIT License](LICENSE)

`dengebot` provides reusable, hardware-independent balancing control, sensor-fusion helpers, motor-command mapping, telemetry, and optional software STOP/FAULT safety checks. **It is a library, not a complete robot firmware, IMU driver, or motor driver.** Your application supplies the sensor readings, timing, physical motor outputs, and hardware protections.

## Highlights

- **Balance control:** `EDABalancePID`, gyro-rate derivative, configurable PID, integral management, fault reporting, and optional recovery behavior.
- **Sensor fusion:** `EDAComplementaryFilter`, optional adaptive-alpha mode, gyro-bias correction, and input-validity checks.
- **Motor command helpers:** `EDAMotorCommand.h` for DC PWM/direction and stepper speed/direction conversions (no STEP pulse generation).
- **Optional DC dead-zone mapping:** `EDADeadZoneCompensation.h` with per-motor effective minimum PWM and command threshold.
- **Telemetry and calibration:** `EDATelemetry.h` and `EDAGyroBiasCalibrator.h`.
- **Optional safety interlock:** `EDASafetyGuard.h` for operator-controlled START/STOP, sample freshness, fall limits, controller faults, and guarded re-arming.

All optional features require explicit integration by the calling sketch; simply including a header does not activate motors or protections.

## What's new in v1.7.4

- Fixed the include-guard collision between the public `dengebot.h` and internal headers.
- Added **target-aware** `start()` and `requestStart()` overloads: safe arming validates the current target and resets PID state immediately on success.
- Added optional `setMaxAbsErrorDeg()` protection against excessive pitch error relative to the balance target, in addition to the absolute pitch limit.
- Improved sensor validation and START/FAULT recovery handling; extended safety regression coverage.
- Updated Arduino CI to compile all six examples for six board profiles.
- **No changes to the core PID control mathematics.** PWM threshold calibration is planned for v1.7.5, not included here.

See [CHANGELOG.md](CHANGELOG.md) for previous releases and detailed changes.

## Installation

1. Download the **v1.7.4 ZIP** from [Releases](https://github.com/Gmby06/dengebot/releases/tag/v1.7.4).
2. In Arduino IDE, choose **Sketch → Include Library → Add .ZIP Library**, then select the library ZIP.
3. Include the unified public header:

```cpp
#include <dengebot.h>
```

Individual headers remain available for projects that prefer them (`EDADengeBot.h`, `EDAComplementaryFilter.h`, `EDATelemetry.h`, etc.). Use **one** installed copy of the library to avoid duplicate-library confusion.

## Supported compilation targets

GitHub Actions compiles the six examples on:

| AVR | Wi-Fi microcontrollers |
| --- | --- |
| Arduino Uno | ESP8266 (NodeMCU) |
| Arduino Nano | ESP32 |
| Arduino Mega 2560 | ESP32-S3 |

The six-board CI matrix passed for v1.7.4. **Compilation is not a claim of physical balancing or safety validation for every board and motor driver.** Physical safety integration has been tested on one Arduino Nano robot (see [Validation](#validation-and-limitations)).

## Examples

| Sketch | Purpose |
| --- | --- |
| [`BasicBalance`](examples/BasicBalance) | Basic controller demonstration |
| [`MotorCommandDemo`](examples/MotorCommandDemo) | Motor-command conversions |
| [`FilterTelemetryDemo`](examples/FilterTelemetryDemo) | Synthetic sensor fusion and telemetry; no physical balancing |
| [`GyroBiasCalibrationDemo`](examples/GyroBiasCalibrationDemo) | Synthetic gyro-bias calibration flow |
| [`DeadZoneDemo`](examples/DeadZoneDemo) | Optional DC dead-zone mapping |
| [`SafetyGuardDemo`](examples/SafetyGuardDemo) | Software START/STOP and FAULT interlock |

These sketches are examples, not preconfigured firmware for a particular robot, pinout, IMU, or motor driver.

## Sensor-to-controller input contract

```cpp
float command = pid.update(targetDeg, pitchDeg,
                           verifiedRateDps, pitchRateDps, dtSeconds);
```

| Input | Unit | Requirement |
| --- | --- | --- |
| `targetDeg` | degrees | Target angle, same axis and sign convention as measured pitch |
| `pitchDeg` | degrees | Fresh measured/fused pitch, not radians |
| `verifiedRateDps` | degrees/second | Fresh, validated pitch-axis angular rate, normally gyro bias-corrected |
| `pitchRateDps` | degrees/second | Pitch-axis rate used for optional assist; may be the same fresh gyro value |
| `dtSeconds` | seconds | Time between fresh control samples; accepted PID range: **0.004–0.050 s** |

Positive pitch rate must correspond to increasing pitch. Verify the sensor axis and sign, then verify physical motor direction **separately with wheels raised**. Do not use wheel RPM in place of pitch angular rate. Check `lastUpdateValid()` and `faulted()` before permitting output.

## Optional complementary filter

```cpp
#include <EDAComplementaryFilter.h>

EDAComplementaryFilter filter(0.98f);

// After a valid, fresh IMU reading:
if (filter.update(accelPitchDeg, gyroPitchRateDps, dtSeconds)) {
  float pitchDeg = filter.pitchDegrees();
  // Validate freshness and safety before driving motors.
}
```

The first valid sample initializes the pitch from the accelerometer. Subsequent samples fuse predicted gyro angle and accelerometer pitch. Filter `dt` is **0.0001–0.25 s** (different from the PID's range). The filter is not an IMU driver.

Optional adaptive fusion uses `configureAdaptive(highAlpha, enterDeviationG, exitDeviationG)` and `setAdaptiveEnabled(true)`, followed by the four-argument `update(accelPitch, gyroRate, dt, accelMagnitudeG)`. The three-argument overload always uses fixed alpha. Default adaptive thresholds of **0.03 g / 0.02 g** are simulation-informed candidates, **not universal hardware settings**. Motor vibration and accelerometer errors vary by machine. `setGyroBias()` accepts a bias measured while stationary; do not subtract the same bias twice.

## Optional DC motor dead-zone compensation

```cpp
#include <EDADeadZoneCompensation.h>

EDA::DeadZoneConfig motor(48, 170, 0.5f); // Example values only
EDA::DCCommand output = EDA::toDCWithDeadZone(pidCommand, 100.0f, motor);
```

`zeroCommandThreshold` is in **PID-command units, not PWM units**. Commands at or below that threshold map to zero; larger commands map to `minEffectivePwm` through `maxPwm`, which creates a discontinuity at activation. Measure each motor/driver independently; this helper does not provide hysteresis, current limiting, stall detection, or physical motor shutdown. Existing `toDC()` behavior is unchanged.

## v1.7.4 software safety guard

`EDASafetyGuard` starts in **STOP**, requires an explicit new operator START event, and checks real sensor timestamps, validity, absolute body tilt, and controller health. On STOP or FAULT, guarded `update()` returns a zero command. FAULT recovery requires another explicit START request under safe conditions—never automatic re-arming.

**Recommended target-aware API (v1.7.4):**

```cpp
#include <dengebot.h>

EDABalancePID pid(edaDefaultConfig());
EDASafetyGuard safety({30.0f, 50}); // Example: absolute tilt deg, max sample age ms

// At setup, if target-relative protection is desired:
safety.setMaxAbsErrorDeg(25.0f);   // 0 disables target-relative protection

// On a NEW operator START event, after validating a real IMU sample:
bool accepted = safety.requestStart(nowMs, imuSampleTimestampMs,
                                    pitchDeg, targetDeg, imuValid, pid);
// Keep motor-driver EN/PWM disabled until accepted AND a fresh guarded
// control update has returned a permitted command.

// On EVERY fresh control tick:
float command = safety.update(pid, nowMs, imuSampleTimestampMs,
                              imuValid, targetDeg, pitchDeg,
                              gyroRateDps, gyroRateDps, dtSeconds);

// Your application must physically disable driver EN/PWM whenever
// !safety.armed() or output is otherwise unsafe.

// On STOP: disable EN/PWM immediately, then call safety.stop().
```

**Important API details:**

- The six-argument `start(nowMs, sampleMs, pitch, target, sensorValid, pid)` and `requestStart(...)` overloads need a **mutable** `EDABalancePID&` and reset PID state **immediately when START succeeds**.
- The legacy five-argument overloads remain supported **only when the target-relative limit is disabled**. With a positive `setMaxAbsErrorDeg()`, they reject arming because the target angle is unknown. With that limit disabled, their PID reset is deferred until the first guarded `update()`; do not use stale PID output in the interval.
- A configured absolute pitch bound remains active even when target-relative protection is enabled. Sensor timestamps must represent **actual new IMU acquisitions**, not an updated `millis()` value every loop. Use the same monotonic clock for `nowMs` and the sample timestamp.
- `requestStart()` must be called for a **new** operator event, not continuously while a button is held or a network command is latched. Check its returned `bool` before considering motor enable.
- A fault in the underlying PID controller must be resolved separately before re-arming. Unsafe START requests are rejected.

### Critical hardware safety boundary

**The library does not control motor-driver EN/PWM pins, a physical emergency stop, or power removal.** Your application must physically disable the driver on STOP, invalid/stale sensor data, FAULT, missed updates, and other unsafe conditions. A blocked control loop or I2C hang cannot be mitigated by a guard that is no longer being called. Use an independent watchdog and motor-disable hardware appropriate for your design. Begin testing with wheels raised and a reliable way to disconnect motor power.

## Telemetry and calibration

`EDATelemetryFrame` carries sensor, controller, timing, state, and fault data. Use `EDATelemetry::printPlotter(Serial, frame)` or `EDATelemetry::printCSV(Serial, frame)`; `EDATelemetry::csvHeader()` supplies the CSV columns. `printCSVExtended(...)` optionally records a known reference pitch, event, sample timestamp, and actual filter alpha. **Do not invent reference measurements on physical hardware.** Serial printing can block, so report less frequently than the control loop.

`EDAGyroBiasCalibrator` estimates gyro bias from a stationary sequence; only apply the result when its stationary validation succeeds. Re-calibrate after warm-up if necessary. See the example sketches for usage.

## Validation and limitations

- Local host tests and ASan/UBSan regression checks passed for the v1.7.4 candidate.
- GitHub Actions passed the six example-compilation targets on Uno, Nano, Mega 2560, ESP8266, ESP32, and ESP32-S3.
- An Arduino Nano-based two-wheel robot was tested for operator START/STOP, fall-angle shutdown, and FAULT recovery without rebooting.
- These tests are **not** a hardware safety certification. Other boards, drivers, IMU orientations, and motor configurations require independent physical testing. Stepper/NEMA17 physical balancing behavior remains unvalidated.

To run the host test suite in a compatible development environment:

```sh
sh tests/run_host_tests.sh
# Where supported:
EDA_SANITIZE=1 sh tests/run_host_tests.sh
```

When reporting issues, include your board, IMU, motor driver, motor type, library version, PID values, power supply, and the behavior observed. See [CONTRIBUTING.md](CONTRIBUTING.md).

## Roadmap

**v1.7.5 (planned):** optional motor/PWM movement-threshold calibration. No v1.7.5 functionality is included in v1.7.4.

## License and attribution

Distributed under the [MIT License](LICENSE). See [ACKNOWLEDGMENTS.md](ACKNOWLEDGMENTS.md) for PID_v1 attribution and other credits.
