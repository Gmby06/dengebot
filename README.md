# dengebot v1.7.2 — Sensor Fusion, Telemetry & Dead-Zone Compensation (test candidate)

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

## Balance controller `update()` input contract (v1.7.2 documentation)

```cpp
float command = balance.update(targetDeg, pitchDeg,
                               verifiedRateDps, pitchRateDps, dtSeconds);
```

| Parameter | Units | Meaning |
|---|---|---|
| `targetDeg` | degrees | Desired pitch; same axis and sign as `pitchDeg`. |
| `pitchDeg` | degrees | Current measured/fused pitch, **not radians**. |
| `verifiedRateDps` | degrees/second | Fresh, validated pitch-axis angular velocity (normally bias-corrected gyro); used for derivative and recovery logic. |
| `pitchRateDps` | degrees/second | Pitch-axis angular velocity for optional assist processing; can be the **same** fresh gyro value as `verifiedRateDps`. It is not wheel RPM. |
| `dtSeconds` | seconds | Time between **fresh** samples; controller accepts 0.004–0.050 s. |

**Sign convention:** Increasing pitch angle must correspond to **positive** angular rate for both rate inputs. Select the IMU axis and invert its sign in the application if needed. `target` and `pitch` must use the same positive direction. The controller uses `error = target - pitch` and a derivative contribution based on negative filtered `verifiedRate`. Motor polarity must be checked separately with wheels raised; it is not inferred from the IMU convention.

**Example sensor-to-controller flow (pseudocode):** read a new IMU sample → check read success, age and finite values → calibrate/correct gyro bias → estimate pitch in degrees → choose gyro pitch-axis rate in deg/s with matching sign → compute `dt` from sample timestamps → call `update(target, pitch, gyroRate, gyroRate, dt)` → check `lastUpdateValid()`, `faulted()`, sample freshness, fall-angle and arming state → only then apply motor command through the application driver. The same rate is deliberately passed twice when no separate pitch-rate estimator is available. Avoid subtracting gyro bias twice.

**Safety evidence boundary:** Host tests verify software behavior, not a physical emergency-stop or a motor driver. The application must implement independent hardware motor disable, stale-sample timeout, fall-angle shutdown and manual re-arming. Test motor polarity with wheels off the ground; then verify unplugged/stale IMU, extreme tilt, STOP button, driver enable disable and restart prevention under a guarded test rig. **No new physical validation is claimed by this documentation change.**

**CI coverage:** The Arduino workflow now compiles `DeadZoneDemo` alongside the other examples for its configured AVR, ESP8266 and ESP32 targets. CI results must be confirmed after pushing this revision; local host-test success is not evidence of a GitHub Actions run.


## v1.7.3 candidate: optional STOP/FAULT safety guard

`EDASafetyGuard.h` adds an **opt-in**, hardware-independent command interlock. It starts
**STOPPED**, requires an explicit `start()` call with a fresh valid IMU sample,
and latches faults for stale data, invalid readings, excessive pitch, or PID faults.
On STOP/FAULT, `update()` returns **zero command** and requires an explicit
`clearFault()` followed by `start()` to re-arm. `stop()` never clears a latched fault.

```cpp
#include <dengebot.h>
EDABalancePID pid(edaDefaultConfig());
EDASafetyConfig safetyConfig = {30.0f, 50}; // example only: deg, milliseconds
EDASafetyGuard safety(safetyConfig);

// After IMU calibration and operator action, with real fresh IMU data:
// bool armed = safety.start(millis(), imuSampleTimestampMs, pitchDeg, imuValid, pid);
// In every control cycle (using a NEW sample and measured dt):
// float cmd = safety.update(pid, millis(), imuSampleTimestampMs, imuValid,
//                           targetDeg, pitchDeg, gyroDps, gyroDps, dtSeconds);
// if (!safety.armed()) { /* hardware motor-enable OFF, PWM=0 */ }
// To STOP: safety.stop(); /* hardware motor-enable OFF immediately */
// To recover: stop motors, investigate fault, pid.clearFault(),
//             safety.clearFault(), then explicitly safety.start(...) with fresh data.
```

**Critical safety boundary:** This class returns a software command only. It does
**not** switch L298N ENA/ENB, PWM pins, power relays, or any physical motor output.
Your application must disable motor outputs on STOP, sensor timeout, and fault,
and must not use the most recent nonzero motor command when an update is skipped.
A missed control-loop deadline must also be handled by the application.

The sample timestamp must come from the **last actual fresh IMU acquisition**,
using the same monotonic millisecond clock as `nowMs`. Do not set `sampleMs = millis()`
on every iteration unless a new IMU sample was actually acquired. `dt` remains in
**seconds** and is validated separately by the PID. The pitch limit is measured
absolute pitch (not pitch error); select it for the physical robot. `sensorValid`
must be false for failed IMU reads or failed sensor checks.

**Validation status:** Host tests cover STOP, stale sample, invalid sensor, fall
angle, controller fault, manual reset and millisecond timer wraparound. These are
software tests only; physical motor cut-off and real robot fall tests are **not
validated** by these tests. Test with wheels raised and motor power isolated first.

### v1.7.3 candidate: operator-controlled FAULT recovery

`EDASafetyGuard::requestStart(nowMs, sampleMs, pitch, sensorValid, pid)` handles a **new** START event from a button, Wi-Fi or other input. When stopped or faulted, it checks a fresh, valid, upright IMU sample and a healthy PID controller; only then does it clear the latched safety fault and arm. Unsafe attempts retain the fault. Do **not** call continuously while a button is held or a network command remains set. The older `start()` and `clearFault()` APIs remain available. An underlying PID fault must be resolved/reset separately before requesting START. Motor PWM and driver enable must be physically shut off by the application on STOP/FAULT; this class alone cannot enforce that.
