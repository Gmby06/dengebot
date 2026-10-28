#ifndef EDA_TELEMETRY_H
#define EDA_TELEMETRY_H
#include "EDANumeric.h"
#include <stdint.h>
// Transport-independent snapshot. No heap, Serial or Wi-Fi dependency.
// Suggested event codes for the extended CSV (caller-defined; values >= 100 are reserved for the application).
enum EDATelemetryEvent {
  EDA_EVENT_NONE = 0, EDA_EVENT_DISTURBANCE = 1, EDA_EVENT_FAULT = 2,
  EDA_EVENT_ARMED = 3, EDA_EVENT_DISARMED = 4, EDA_EVENT_USER_BASE = 100
};
struct EDATelemetryFrame {
  uint32_t timeMs;
  float targetDeg;
  float pitchDeg;
  float gyroRateDegPerSec;
  float pidOutput;
  float motorCommand;
  uint32_t loopTimeUs;
  uint8_t state;  // 0=stopped, 1=running, 2=fault (caller-defined convention)
  uint8_t fault;
};
class EDATelemetry {
public:
  static bool valid(const EDATelemetryFrame &f) {
    return eda_detail::finiteFloat(f.targetDeg) && eda_detail::finiteFloat(f.pitchDeg) &&
      eda_detail::finiteFloat(f.gyroRateDegPerSec) && eda_detail::finiteFloat(f.pidOutput) &&
      eda_detail::finiteFloat(f.motorCommand);
  }
  // Caller supplies any Arduino Print-compatible sink (Serial, network, etc.).
  // Plotter uses tab-separated labeled values; labels are stable for tooling.
  template <typename PrintLike>
  static bool printPlotter(PrintLike &out, const EDATelemetryFrame &f) {
    if (!valid(f)) return false;
    out.print("Target:"); out.print(f.targetDeg, 3);
    out.print('\t'); out.print("Pitch:"); out.print(f.pitchDeg, 3);
    out.print('\t'); out.print("Gyro:"); out.print(f.gyroRateDegPerSec, 3);
    out.print('\t'); out.print("PID:"); out.print(f.pidOutput, 3);
    out.print('\t'); out.print("Motor:"); out.print(f.motorCommand, 3);
    out.print('\t'); out.print("LoopUs:"); out.print(f.loopTimeUs);
    out.print('\t'); out.print("State:"); out.print((unsigned int)f.state);
    out.print('\t'); out.print("Fault:"); out.println((unsigned int)f.fault);
    return true;
  }
  // CSV is suitable for logging and future external tooling; includes no header.
  template <typename PrintLike>
  static bool printCSV(PrintLike &out, const EDATelemetryFrame &f) {
    if (!valid(f)) return false;
    out.print(f.timeMs); out.print(',');
    out.print(f.targetDeg, 3); out.print(',');
    out.print(f.pitchDeg, 3); out.print(',');
    out.print(f.gyroRateDegPerSec, 3); out.print(',');
    out.print(f.pidOutput, 3); out.print(',');
    out.print(f.motorCommand, 3); out.print(',');
    out.print(f.loopTimeUs); out.print(',');
    out.print((unsigned int)f.state); out.print(',');
    out.println((unsigned int)f.fault);
    return true;
  }
  // Optional reference fields for simulation / instrumented hardware.
  // Keep legacy CSV and EDATelemetryFrame ABI intact.
  template <typename PrintLike>
  static bool printCSVExtended(PrintLike &out, const EDATelemetryFrame &f,
                               float pitchRefDeg, unsigned int eventCode,
                               uint32_t sensorTimeMs, float usedAlpha) {
    // V1.7.1: pitchRefDeg may be NaN = "no reference available" (empty field). +-Inf is rejected.
    const bool refMissing = (pitchRefDeg != pitchRefDeg);
    if (!valid(f) || (!refMissing && !eda_detail::finiteFloat(pitchRefDeg)) ||
        !eda_detail::finiteFloat(usedAlpha) || usedAlpha < 0 || usedAlpha > 1)
      return false;
    out.print(f.timeMs); out.print(',');
    out.print(f.targetDeg, 3); out.print(',');
    out.print(f.pitchDeg, 3); out.print(',');
    out.print(f.gyroRateDegPerSec, 3); out.print(',');
    out.print(f.pidOutput, 3); out.print(',');
    out.print(f.motorCommand, 3); out.print(',');
    out.print(f.loopTimeUs); out.print(',');
    out.print((unsigned int)f.state); out.print(',');
    out.print((unsigned int)f.fault); out.print(',');
    if (!refMissing) out.print(pitchRefDeg, 3);
    out.print(',');
    out.print(eventCode); out.print(',');
    out.print(sensorTimeMs); out.print(',');
    out.println(usedAlpha, 3);
    return true;
  }
  static const char *csvExtendedHeader() {
    return "time_ms,target_deg,pitch_deg,gyro_dps,pid_output,motor_command,loop_us,state,fault,pitch_ref_deg,event,sensor_time_ms,alpha_used";
  }
  static const char *csvHeader() {
    return "time_ms,target_deg,pitch_deg,gyro_dps,pid_output,motor_command,loop_us,state,fault";
  }
};
#endif
