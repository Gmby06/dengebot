# dengebot

**Arduino-compatible control library for two-wheel self-balancing robots.**

**Latest pre-release: [v1.7.6 — A4988 & DRV8825 STEP/DIR Support](https://github.com/Gmby06/dengebot/releases/tag/v1.7.6)**

**Previous pre-release: [v1.7.5 — Safe PWM Motor Calibration](https://github.com/Gmby06/dengebot/releases/tag/v1.7.5)**

[Latest pre-release](https://github.com/Gmby06/dengebot/releases/tag/v1.7.6) · [Examples](examples) · [Changelog](CHANGELOG.md) · [Report an issue](https://github.com/Gmby06/dengebot/issues) · [MIT License](LICENSE)

`dengebot` provides reusable, hardware-independent balancing control, sensor-fusion helpers, motor-command mapping, telemetry, and optional software STOP/FAULT safety checks. **It is a library, not a complete robot firmware or IMU driver; the v1.7.6 STEP/DIR module also requires a hardware-specific timing and GPIO backend.** Your application supplies the sensor readings, timing, physical motor outputs, and hardware protections.

## Highlights

- **Balance control:** `EDABalancePID`, gyro-rate derivative, configurable PID, integral management, fault reporting, and optional recovery behavior.
- **Sensor fusion:** `EDAComplementaryFilter`, optional adaptive-alpha mode, gyro-bias correction, and input-validity checks.
- **Motor command helpers:** `EDAMotorCommand.h` for DC PWM/direction and stepper speed/direction conversions. **Development-only:** `EDAStepDirController.h` emits logical STEP/DIR pulse levels when driven by a precise periodic timer.
- **Optional PWM breakaway calibration (v1.7.5):** manual, encoder-free four-direction threshold discovery with bounded pulses and no automatic application.
- **Optional DC dead-zone mapping:** `EDADeadZoneCompensation.h` with per-motor effective minimum PWM and command threshold.
- **Telemetry and calibration:** `EDATelemetry.h` and `EDAGyroBiasCalibrator.h`.
- **Optional safety interlock:** `EDASafetyGuard.h` for operator-controlled START/STOP, sample freshness, fall limits, controller faults, and guarded re-arming.

All optional features require explicit integration by the calling sketch; simply including a header does not activate motors or protections.

## What's new in v1.7.6

- Separate [A4988](examples/ESP32_A4988_Demo) and [DRV8825](examples/ESP32_DRV8825_Demo) motor-only demos for classic ESP32, plus the [STEP/DIR first test](examples/ESP32_StepDirFirstTest).
- FIX4 LEDC member-initialization order correction and removal of a duplicate PWM calibration example directory.
- Updated STEP/DIR documentation, pin mapping, and CI matrix for Arduino-ESP32 2.0.17 and 3.0.7.
- GitHub Actions host tests and all configured Arduino board compilation jobs passed on commit `66611f6`. These are software checks, **not** real A4988/DRV8825 motor or balancing validation.
- No changes to existing PID, sensor fusion, DC motor runtime control, or safety behavior in the final release-preparation pass.

## Previously added in v1.7.5

- Operator-confirmed PWM breakaway calibration for each motor and direction, with bounded test pulses, STOP and fault handling.
- The **PwmCalibrationDemo** uses a **150 ms** test pulse (example-specific setting). The library's default `PwmCalibrationConfig` pulse remains **120 ms** for backward compatibility.
- Recalibration invalidates only the selected result, preventing stale confirmed thresholds after abort or timeout.
- Added configuration/enum validation and PID reset / SafetyGuard fault diagnostic fixes while preserving legacy API values and default fault policy.
- Motor-driving demo limited to Nano, Uno and Mega; unsupported boards use a pin-inert example.
- Host regression and fake-Arduino demo tests cover safety paths. Physical PWM cutoff on MCU lockup still requires **independent hardware protection**.

See [CHANGELOG.md](CHANGELOG.md) for full changes and previous releases.


## v1.7.6 STEP/DIR — A4988 and DRV8825 demos (pre-release)

This **published pre-release** builds on v1.7.5. GitHub Actions software checks passed, while electrical and real-motor testing remain pending.

**Added:** `EDAStepDirController.h` — a portable two-channel integer-only
STEP/DIR pulse scheduler. It accepts signed steps/second, handles separate
left/right phase accumulation, direction setup, explicit arming, STOP and
150-ms command refresh watchdog. It owns **no pins, interrupts or power**.
`EDABalancePID` and `toStepperVelocity()` are unchanged.

**Motor-only test sketches:** `examples/ESP32_StepDirFirstTest`, `examples/ESP32_A4988_Demo`, and `examples/ESP32_DRV8825_Demo` (FIX3/FIX4).
ESP32-WROOM-32 **motor-only** bench test: no IMU, no PID. STEP pulses come from
the ESP32 **LEDC hardware** (`EDAStepDirLEDC_ESP32.h`), not from a timer ISR.
Serial Monitor 115200, No line ending: `1..6` wheel/direction, `+`/`-` select
200/500/1000/2000/5000/10000/20000/40000 STEP/s, `p` = ramp (20000 STEP/s^2 from
200 STEP/s) then 2-second hold, `s` = stop. GPIO18/19 left STEP/DIR, GPIO23/5
right STEP/DIR, GPIO33 **active-low common ENABLE** with a **physical 10-kOhm
pull-up** so resets keep drivers disabled. Classic ESP32 only; other targets
compile an inert example. These pins are NOT portable defaults.

**Driver-independent is not electrically universal:** the driver must accept
3.3-V logic and the chosen common-enable arrangement; optocoupler STEP/DIR
inputs (e.g. some TB6600/DM-series) may require level shifting and different
pulse polarity/width. Microstep and motor current must be set on the driver.
Do not connect motor power until you have verified driver requirements.

**Limitations:** experimental STEP/DIR feature in a pre-release. Informational A4988/DRV8825 driver profiles are
available, but they do not configure hardware or enforce individual pulse timings.
There is no AVR interrupt backend, no measured STEP output, no motor-load
verification and no physical hardware STOP certification. The command watchdog
only works while the MCU and main loop still run; use an independent hardware
power/enable cutoff.
Provide an independent hardware stop/power cutoff. Wheel-off-ground testing
and an immediately accessible physical power disconnect are required.

`tests/step_dir.cpp` covers output frequency simulation, directions,
reversal, STOP, arming, invalid inputs and command timeout. Run
`sh tests/run_host_tests.sh`. GitHub Actions board compilation passed for the current pre-release; physical bench validation remains pending.

## Installation

1. Download the **v1.7.6 pre-release** from [Releases](https://github.com/Gmby06/dengebot/releases/tag/v1.7.6), or choose [v1.7.5](https://github.com/Gmby06/dengebot/releases/tag/v1.7.5) if you only need the earlier PWM-calibration preview. Use the source-code ZIP or a library ZIP with `library.properties` at the library root.
2. In Arduino IDE, choose **Sketch → Include Library → Add .ZIP Library**, then select the library ZIP.
3. Include the unified public header:

```cpp
#include <dengebot.h>
```

Individual headers remain available for projects that prefer them (`EDADengeBot.h`, `EDAComplementaryFilter.h`, `EDATelemetry.h`, etc.). Use **one** installed copy of the library to avoid duplicate-library confusion.

## Supported compilation targets

The release CI builds the library examples for these boards:

| AVR | Wi-Fi microcontrollers |
| --- | --- |
| Arduino Uno | ESP8266 (NodeMCU) |
| Arduino Nano | ESP32 |
| Arduino Mega 2560 | ESP32-S3 |

The v1.7.6 pre-release CI passed all configured example-compilation jobs (including both ESP32 core versions and Gemma); physical STEP/DIR balancing is not yet validated. **Compilation is not a claim of physical balancing or safety validation for every board and motor driver.** Physical safety integration has been tested on one Arduino Nano robot (see [Validation](#validation-and-limitations)).

## Examples

| Sketch | Purpose |
| --- | --- |
| [`BasicBalance`](examples/BasicBalance) | Basic controller demonstration |
| [`MotorCommandDemo`](examples/MotorCommandDemo) | Motor-command conversions |
| [`FilterTelemetryDemo`](examples/FilterTelemetryDemo) | Synthetic sensor fusion and telemetry; no physical balancing |
| [`GyroBiasCalibrationDemo`](examples/GyroBiasCalibrationDemo) | Synthetic gyro-bias calibration flow |
| [`DeadZoneDemo`](examples/DeadZoneDemo) | Optional DC dead-zone mapping |
| [`SafetyGuardDemo`](examples/SafetyGuardDemo) | Software START/STOP and FAULT interlock |
| [`PwmCalibrationDemo`](examples/PwmCalibrationDemo) | Manual DC-motor PWM calibration on Nano/Uno/Mega |
| [`ESP32_StepDirFirstTest`](examples/ESP32_StepDirFirstTest) | Classic ESP32 STEP/DIR motor bench test (experimental) |
| [`ESP32_A4988_Demo`](examples/ESP32_A4988_Demo) | Classic ESP32 A4988 motor-only STEP/DIR demo (experimental) |
| [`ESP32_DRV8825_Demo`](examples/ESP32_DRV8825_Demo) | Classic ESP32 DRV8825 motor-only STEP/DIR demo (experimental) |

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

**Direct PID use.** `EDABalancePID` keeps `zeroOutputOnFault` OFF by default (backward compatible): after one invalid sample a later good sample can produce output again while `faulted()` stays latched. If you wire the PID output straight to the motors, call `setZeroOutputOnFault(true)` and check `faulted()`; `EDASafetyGuard` already trips on `pid.faulted()`.

**The library does not control motor-driver EN/PWM pins, a physical emergency stop, or power removal.** Your application must physically disable the driver on STOP, invalid/stale sensor data, FAULT, missed updates, and other unsafe conditions. A blocked control loop or I2C hang cannot be mitigated by a guard that is no longer being called. Use an independent watchdog and motor-disable hardware appropriate for your design. Begin testing with wheels raised and a reliable way to disconnect motor power.

## Telemetry and calibration

`EDATelemetryFrame` carries sensor, controller, timing, state, and fault data. Use `EDATelemetry::printPlotter(Serial, frame)` or `EDATelemetry::printCSV(Serial, frame)`; `EDATelemetry::csvHeader()` supplies the CSV columns. `printCSVExtended(...)` optionally records a known reference pitch, event, sample timestamp, and actual filter alpha. **Do not invent reference measurements on physical hardware.** Serial printing can block, so report less frequently than the control loop.

`EDAGyroBiasCalibrator` estimates gyro bias from a stationary sequence; only apply the result when its stationary validation succeeds. Re-calibrate after warm-up if necessary. See the example sketches for usage.

## Validation and limitations

- v1.7.6 host regression tests and GitHub Actions host workflow passed on commit `66611f6`; sanitizer outcomes depend on the execution environment.
- v1.7.6 GitHub Actions passed example-compilation jobs for Uno, Nano, Mega 2560, ESP8266, classic ESP32 (cores 2.0.17 and 3.0.7), and ESP32-S3 (cores 2.0.17 and 3.0.7). Gemma passed its separate inert-demo build.
- A4988, DRV8825 and first STEP/DIR demo sketches are compiled in the ESP32 jobs. On ESP32-S3 they compile as inert stubs, so this does not validate STEP pulses on ESP32-S3.
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

**v1.7.6 pre-release:** A4988/DRV8825 STEP/DIR demos and CI compatibility checks are published. Next steps are physical pulse waveform, motor rotation, load/current/thermal, and hardware STOP verification before making stronger hardware-support claims. **v1.7.5** remains available as the earlier PWM-calibration preview.

## License and attribution

Distributed under the [MIT License](LICENSE). See [ACKNOWLEDGMENTS.md](ACKNOWLEDGMENTS.md) for PID_v1 attribution and other credits.

## v1.7.5: PWM breakaway calibration

`EDAPwmCalibration.h` is an **optional**, encoder-free state machine for finding, by eye, the lowest PWM at which each wheel starts to turn, per wheel and direction. It owns no pins and cannot stop a motor by itself: call `tick(millis())` every loop, drive the motor only from `command(millis())`, and keep an independent hardware motor disable. **No software rule here can cut power if the microcontroller or the main loop hangs; use a physical motor disconnect and an independent hardware enable cutoff.**

**Operator flow.** `begin` (wheel and direction), `requestPulse` (one bounded pulse; every pulse needs its own request, there is no automatic motion), then `advance` (no movement seen, PWM rises by `stepPwm`) or `confirmMovement` (movement seen, value saved). The library cannot detect motion without a feedback sensor; confirmation is always the operator's.

**Timing.** A pulse lasts `pulseMs` (library default 120 ms; **PwmCalibrationDemo uses 150 ms**) and is followed by `cooldownMs`, during which `advance()` is refused. `command(nowMs)` returns zero after the pulse deadline even if `tick()` has not run yet; while the loop runs, the output can overshoot by at most one loop period. The operator has `idleTimeoutMs` (default 15 s) to act while READY; that clock does not run during a pulse or cooldown. `maxSessionMs` (default 300 s) is a hard cap. `idleTimeoutMs` and `maxSessionMs` must be at most `0x7fffffff` ms (about 24.8 days); larger values make the config invalid. Faults: `Timeout` (session cap), `IdleTimeout`, `LimitReached`, `InvalidConfig`, `Aborted`.

**Results.** `begin()` invalidates the stored result of the selected wheel/direction (other slots are kept), so an aborted re-run never leaves an old value looking confirmed. `results().has()/get()` return `false`/`0` for invalid wheel or direction values.

**Suggestion.** `suggestDeadZoneMinPwm(results, wheel, marginPercent=15)` returns a conservative starting value for `DeadZoneConfig::minEffectivePwm`: the lower confirmed direction minus the margin. `DeadZoneConfig` is symmetric, so this single value is a policy choice, not a measured fact: in the direction with the HIGHER measured threshold it can sit below breakaway. Look at both raw per-direction results (`results().get(wheel, direction)`) before choosing. It does **not** produce per-direction `DeadZoneConfig` values. What is measured is a short-pulse **breakaway** threshold of an unloaded wheel, rounded up by at most `stepPwm-1`; it does not establish loaded or continuous motion. Verify it on the robot and never apply it automatically. In this project's synthetic simulation an overestimated minimum caused strong chatter around the center while an underestimate did not; that simulation is not a hardware result.

**Demo.** `examples/PwmCalibrationDemo` is active ONLY on Arduino Nano, Uno and Mega boards (the boards in the CI matrix) and drives an L298N with the example pin map: ENA D5, ENB D6, IN1 D4, IN2 D7, IN3 D8, IN4 D12. Confirm YOUR wiring (your robot may use D9 for IN4), remove the ENA/ENB jumpers, raise and secure the wheels, and keep motor power disconnected during upload. On every other board (other AVR boards, ESP8266/ESP32) the demo is intentionally inert and touches no pin; the header stays usable with your own driver code. The demo writes to the driver pins only when the command changes.

**Arduino Gemma / ATtiny85 and other AVR boards:** The calibration demo intentionally compiles as an inert sketch with empty `setup()` and `loop()` because standard `Serial` and Nano motor pin assignments are unavailable. This does **not** indicate physical calibration support for Gemma.

Serial controls (115200 baud, "No line ending"): `1` left forward, `2` left reverse, `3` right forward, `4` right reverse, `p` one bounded pulse, `n` no movement seen (PWM up after cooldown), `y` movement seen (save), `s` stop. Pressing `s`, `n`, `y` or `1`-`4` during a pulse cuts the output and aborts that pulse; a deliberate new selection and `p` are needed to continue. Pressing `p` during a pulse is ignored (the pulse is neither extended nor restarted); CR/LF and other characters are ignored.

**Status.** Host tests cover the state machine and, through a fake Arduino, the demo sketch itself. The v1.7.5 release CI passed; these tests do not verify real PWM waveforms, physical cutoff or other hardware.

### v1.7.6 pre-release: ESP32 STEP/DIR FIX3/FIX4 (experimental)

A dedicated classic ESP32 LEDC adapter, `EDAStepDirLEDC_ESP32.h`, supplies **hardware-generated** STEP waveforms for two STEP/DIR motors. The example `ESP32_StepDirFirstTest` uses a user-invoked ramp + two-second hold at 200–40000 requested STEP/s; it is not a balancing firmware. Arduino-ESP32 2.x and 3.x LEDC API differences are handled conditionally. The separate portable `EDAStepDirController.h` scheduler is unchanged (its default rate cap remains 4000; over-limit requests are now clamped, not rejected), and the v1.7.5 PID is unchanged. This feature is pending physical rotation and pulse-frequency validation; electrical levels and ENABLE polarity depend on the external driver. See `V1_7_6_STEPDIR_FIX3_NOTES.md` and `DRV8825_UYUMLULUK_TR.md`.
