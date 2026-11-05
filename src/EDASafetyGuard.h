#ifndef EDA_SAFETY_GUARD_H
#define EDA_SAFETY_GUARD_H
#include <stdint.h>
#include <math.h>
#include "EDANumeric.h"
#include "EDADengeBot.h"

// Optional, hardware-independent STOP/FAULT interlock.
// Time units: milliseconds from the SAME monotonic clock (e.g. millis()).
// Caller MUST supply a real IMU sample timestamp, not simply 'now' each loop.
// ORDER MATTERS: take sampleMs FIRST, then nowMs = millis(). A sampleMs newer than
// nowMs (e.g. now=millis(); readIMU(); sampleMs=millis();) is rejected as a
// future timestamp and latches StaleSensor.
// Caller MUST still disable motor-driver EN/PWM in hardware/application on STOP/FAULT.
enum class EDASafetyFault : uint8_t {
  None, Stopped, StaleSensor, InvalidSensor, Fallen, ControllerFault,
  InvalidConfig   // appended in v1.7.5; earlier values are unchanged
};

struct EDASafetyConfig {
  float maxAbsPitchDeg;        // absolute measured pitch limit; > 0
  uint32_t maxSampleAgeMs;     // 1..0x7fffffff; wrap-safe unsigned subtraction
};

class EDASafetyGuard {
public:
  explicit EDASafetyGuard(const EDASafetyConfig& cfg)
    : cfg_(cfg), maxAbsErrorDeg_(0.0f), armed_(false), resetPending_(false),
      fault_(EDASafetyFault::Stopped) {
    if (!validConfig(cfg_)) fault_ = EDASafetyFault::InvalidConfig;
  }
  static bool validConfig(const EDASafetyConfig& c) {
    return eda_detail::finiteFloat(c.maxAbsPitchDeg) && c.maxAbsPitchDeg > 0.0f &&
           c.maxSampleAgeMs > 0 && c.maxSampleAgeMs <= 0x7ffffffful;   // wrap-safe age check rejects larger values
  }
  // Optional (v1.7.4): ALSO trip Fallen when |pitch - target| exceeds this many
  // degrees. Use it when the balance target is not 0 deg (e.g. -3.85) so the fall
  // limit is symmetric around the target. 0 disables (default). Returns false and
  // leaves the previous value if v is not finite or negative.
  bool setMaxAbsErrorDeg(float v) {
    if (!eda_detail::finiteFloat(v) || v < 0.0f) return false;
    maxAbsErrorDeg_ = v;
    return true;
  }
  float maxAbsErrorDeg() const { return maxAbsErrorDeg_; }
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
  // Legacy 5-argument API has no target parameter. With a target-relative
  // limit enabled it cannot verify safe arming, so it FAILS CLOSED.
  // Use the 6-argument overload instead; with target limit disabled the legacy
  // API remains source- and behavior-compatible.
  bool start(uint32_t nowMs, uint32_t sampleMs, float pitch, bool sensorValid, const EDABalancePID& pid) {
    if (maxAbsErrorDeg_ > 0.0f || fault_ != EDASafetyFault::Stopped || pid.faulted() || !pid.configValid()) return false;
    if (!checkSample(nowMs, sampleMs, pitch, sensorValid)) return false;
    armed_ = true; fault_ = EDASafetyFault::None;
    resetPending_ = true; // v1.7.4: the first update() after arming resets the PID state.
    return true;
  }
  // One explicit START event (button edge, Wi-Fi command, etc.).
  // If a FAULT is latched, recover ONLY when the IMU is fresh, valid and
  // upright and the controller is healthy. Failed attempts keep FAULT latched.
  // Never call repeatedly for a held button or retained network command.
  bool requestStart(uint32_t nowMs, uint32_t sampleMs, float pitch,
                    bool sensorValid, const EDABalancePID& pid) {
    if (armed_ || maxAbsErrorDeg_ > 0.0f) return false; // Legacy path cannot validate target.
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
  // Target-aware API: both absolute pitch and target-relative error checked
  // BEFORE arming. Mutable PID is required for an immediate reset.
  bool start(uint32_t nowMs, uint32_t sampleMs, float pitch, float target,
             bool sensorValid, EDABalancePID& pid) {
    if (fault_ != EDASafetyFault::Stopped || pid.faulted() || !pid.configValid() ||
        !validConfig(cfg_) || !startTargetSafe(pitch, target) ||
        !sampleSafe(nowMs, sampleMs, pitch, sensorValid)) return false;
    pid.reset();
    resetPending_ = false;
    armed_ = true;
    fault_ = EDASafetyFault::None;
    return true;
  }
  bool requestStart(uint32_t nowMs, uint32_t sampleMs, float pitch, float target,
                    bool sensorValid, EDABalancePID& pid) {
    if (armed_ || pid.faulted() || !pid.configValid() || !validConfig(cfg_) ||
        !startTargetSafe(pitch, target) ||
        !sampleSafe(nowMs, sampleMs, pitch, sensorValid)) return false;
    // Validation passed: recover only on this explicit operator event.
    pid.reset();
    resetPending_ = false;
    armed_ = true;
    fault_ = EDASafetyFault::None;
    return true;
  }
  // Call EVERY motor-control cycle; zero is returned whenever stopped/faulted.
  float update(EDABalancePID& pid, uint32_t nowMs, uint32_t sampleMs,
               bool sensorValid, float target, float pitch,
               float verifiedRate, float pitchRate, float dt) {
    if (!armed_) return 0.0f;
    if (!checkSample(nowMs, sampleMs, pitch, sensorValid)) return 0.0f;
    if (pid.faulted() || !pid.configValid()) { trip(EDASafetyFault::ControllerFault); return 0.0f; }
    if (maxAbsErrorDeg_ > 0.0f) {
      if (!eda_detail::finiteFloat(target)) { trip(EDASafetyFault::ControllerFault); return 0.0f; }
      if (fabsf(pitch - target) > maxAbsErrorDeg_) { trip(EDASafetyFault::Fallen); return 0.0f; }
    }
    // Drop integral/filter/recovery state left over from before the STOP or FAULT.
    if (resetPending_) { pid.reset(); resetPending_ = false; }
    const float output = pid.update(target, pitch, verifiedRate, pitchRate, dt);
    if (!pid.lastUpdateValid() || pid.faulted() || !eda_detail::finiteFloat(output)) {
      trip(EDASafetyFault::ControllerFault); return 0.0f;
    }
    return output;
  }
private:
  bool startTargetSafe(float pitch, float target) const {
    if (!eda_detail::finiteFloat(target) || !eda_detail::finiteFloat(pitch)) return false;
    // Both independent safety limits apply when enabled; never substitute one for the other.
    if (maxAbsErrorDeg_ > 0.0f &&
        fabsf(pitch - target) > maxAbsErrorDeg_) return false;
    return true;
  }
  bool sampleSafe(uint32_t nowMs, uint32_t sampleMs, float pitch, bool sensorValid) const {
    if (!sensorValid || !eda_detail::finiteFloat(pitch)) return false;
    const uint32_t age = nowMs - sampleMs;
    return age <= cfg_.maxSampleAgeMs && age <= 0x7ffffffful &&
           fabsf(pitch) <= cfg_.maxAbsPitchDeg;
  }
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
  float maxAbsErrorDeg_;
  bool armed_;
  bool resetPending_;
  EDASafetyFault fault_;
};
#endif
