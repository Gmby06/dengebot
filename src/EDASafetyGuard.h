#ifndef EDA_SAFETY_GUARD_H
#define EDA_SAFETY_GUARD_H
#include <stdint.h>
#include <math.h>
#include "EDANumeric.h"
#include "EDADengeBot.h"

// Optional, hardware-independent STOP/FAULT interlock.
// Time units: milliseconds from the SAME monotonic clock (e.g. millis()).
// Caller MUST supply a real IMU sample timestamp, not simply 'now' each loop.
// Caller MUST still disable motor-driver EN/PWM in hardware/application on STOP/FAULT.
enum class EDASafetyFault : uint8_t {
  None, Stopped, StaleSensor, InvalidSensor, Fallen, ControllerFault
};

struct EDASafetyConfig {
  float maxAbsPitchDeg;        // absolute measured pitch limit; > 0
  uint32_t maxSampleAgeMs;     // > 0; wrap-safe unsigned subtraction
};

class EDASafetyGuard {
public:
  explicit EDASafetyGuard(const EDASafetyConfig& cfg)
    : cfg_(cfg), armed_(false), fault_(EDASafetyFault::Stopped) {
    if (!validConfig(cfg_)) fault_ = EDASafetyFault::InvalidSensor;
  }
  static bool validConfig(const EDASafetyConfig& c) {
    return eda_detail::finiteFloat(c.maxAbsPitchDeg) && c.maxAbsPitchDeg > 0.0f && c.maxSampleAgeMs > 0;
  }
  bool armed() const { return armed_; }
  bool faulted() const { return fault_ != EDASafetyFault::None && fault_ != EDASafetyFault::Stopped; }
  EDASafetyFault fault() const { return fault_; }
  void stop() { armed_ = false; if (!faulted()) fault_ = EDASafetyFault::Stopped; }
  void trip(EDASafetyFault reason) {
    armed_ = false;
    if (reason != EDASafetyFault::None && reason != EDASafetyFault::Stopped && !faulted()) fault_ = reason;
    else if (reason == EDASafetyFault::Stopped && !faulted()) fault_ = reason;
  }
  // Explicit operator reset only while stopped. Does not arm the motors.
  bool clearFault() {
    if (armed_ || !validConfig(cfg_)) return false;
    fault_ = EDASafetyFault::Stopped;
    return true;
  }
  // Arm requires fresh, valid, upright IMU data and a healthy PID controller.
  bool start(uint32_t nowMs, uint32_t sampleMs, float pitch, bool sensorValid, const EDABalancePID& pid) {
    if (fault_ != EDASafetyFault::Stopped || pid.faulted() || !pid.configValid()) return false;
    if (!checkSample(nowMs, sampleMs, pitch, sensorValid)) return false;
    armed_ = true; fault_ = EDASafetyFault::None;
    return true;
  }
  // One explicit START event (button edge, Wi-Fi command, etc.).
  // If a FAULT is latched, recover ONLY when the IMU is fresh, valid and
  // upright and the controller is healthy. Failed attempts keep FAULT latched.
  // Never call repeatedly for a held button or retained network command.
  bool requestStart(uint32_t nowMs, uint32_t sampleMs, float pitch,
                    bool sensorValid, const EDABalancePID& pid) {
    if (armed_) return false; // Do not reinterpret START as STOP.
    if (!validConfig(cfg_) || pid.faulted() || !pid.configValid()) return false;
    // Evaluate without mutating the original latched FAULT on failure.
    if (!sensorValid || !eda_detail::finiteFloat(pitch)) return false;
    const uint32_t age = nowMs - sampleMs;
    if (age > cfg_.maxSampleAgeMs || age > 0x7ffffffful) return false;
    if (fabsf(pitch) > cfg_.maxAbsPitchDeg) return false;
    // Preconditions passed: explicit operator request authorizes recovery.
    fault_ = EDASafetyFault::Stopped;
    return start(nowMs, sampleMs, pitch, sensorValid, pid);
  }
  // Call EVERY motor-control cycle; zero is returned whenever stopped/faulted.
  float update(EDABalancePID& pid, uint32_t nowMs, uint32_t sampleMs,
               bool sensorValid, float target, float pitch,
               float verifiedRate, float pitchRate, float dt) {
    if (!armed_) return 0.0f;
    if (!checkSample(nowMs, sampleMs, pitch, sensorValid)) return 0.0f;
    if (pid.faulted() || !pid.configValid()) { trip(EDASafetyFault::ControllerFault); return 0.0f; }
    const float output = pid.update(target, pitch, verifiedRate, pitchRate, dt);
    if (!pid.lastUpdateValid() || pid.faulted() || !eda_detail::finiteFloat(output)) {
      trip(EDASafetyFault::ControllerFault); return 0.0f;
    }
    return output;
  }
private:
  bool checkSample(uint32_t nowMs, uint32_t sampleMs, float pitch, bool sensorValid) {
    if (!sensorValid || !eda_detail::finiteFloat(pitch)) { trip(EDASafetyFault::InvalidSensor); return false; }
    // Reject timestamps in the future (including erroneous timebase mismatch).
    const uint32_t age = nowMs - sampleMs;
    if (age > cfg_.maxSampleAgeMs || age > 0x7ffffffful) {
      trip(EDASafetyFault::StaleSensor); return false;
    }
    if (fabsf(pitch) > cfg_.maxAbsPitchDeg) { trip(EDASafetyFault::Fallen); return false; }
    return true;
  }
  EDASafetyConfig cfg_;
  bool armed_;
  EDASafetyFault fault_;
};
#endif
