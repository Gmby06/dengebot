#ifndef EDA_COMPLEMENTARY_FILTER_H
#define EDA_COMPLEMENTARY_FILTER_H
#include "EDANumeric.h"
// Pitch in degrees, gyro in degrees/second, dt in seconds, acceleration in g.
// No sensor driver, no motor output. Caller must handle stale/invalid readings.
class EDAComplementaryFilter {
public:
  explicit EDAComplementaryFilter(float alpha = 0.98f)
    : alpha_(0.98f), angle_(0), usedAlpha_(0.98f), gyroBias_(0),
      initialized_(false), valid_(false), adaptive_(false), highAccel_(false),
      highAlpha_(0.998f), enterDeviationG_(0.03f), exitDeviationG_(0.02f) {
    setAlpha(alpha);
  }
  // V1.7.1: while adaptive mode is enabled, alpha must not exceed the high-acceleration alpha
  // (otherwise the "high" mode would trust the gyro LESS than the normal mode).
  bool setAlpha(float a) {
    if (!eda_detail::finiteFloat(a) || a < 0 || a > 1) return false;
    if (adaptive_ && a > highAlpha_) return false;
    alpha_ = a; return true;
  }
  float alpha() const { return alpha_; }
  float usedAlpha() const { return usedAlpha_; }
  bool setGyroBias(float biasDegPerSec) {
    if (!eda_detail::finiteFloat(biasDegPerSec)) return false;
    gyroBias_ = biasDegPerSec; return true;
  }
  float gyroBias() const { return gyroBias_; }
  // enter > exit prevents mode chatter around acceleration threshold.
  // V1.7.1 defaults: enter 0.03 g / exit 0.02 g of |accelMagnitude - 1 g|. These are SIMULATION
  // candidates. A purely horizontal acceleration of 0.1 g changes |a| by only ~0.005 g, so thresholds
  // near 0.15 g effectively never trigger. Measure your IMU's magnitude noise/vibration first.
  bool configureAdaptive(float highAlpha, float enterDeviationG = 0.03f,
                         float exitDeviationG = 0.02f) {
    if (!eda_detail::finiteFloat(highAlpha) || highAlpha < alpha_ || highAlpha > 1 ||
        !eda_detail::finiteFloat(enterDeviationG) ||
        !eda_detail::finiteFloat(exitDeviationG) ||
        exitDeviationG < 0 || enterDeviationG <= exitDeviationG) return false;
    highAlpha_ = highAlpha; enterDeviationG_ = enterDeviationG;
    exitDeviationG_ = exitDeviationG; highAccel_ = false; return true;
  }
  // Returns false (and leaves adaptive mode unchanged) if enabling would make alpha() > the high alpha.
  bool setAdaptiveEnabled(bool enabled) {
    if (enabled && alpha_ > highAlpha_) return false;
    adaptive_ = enabled; highAccel_ = false; return true;
  }
  bool adaptiveEnabled() const { return adaptive_; }
  bool highAcceleration() const { return adaptive_ && highAccel_; }
  void reset() { angle_ = 0; initialized_ = false; valid_ = false; highAccel_ = false; usedAlpha_ = alpha_; }
  bool initialize(float pitch) {
    if (!eda_detail::finiteFloat(pitch)) { valid_ = false; return false; }
    angle_ = pitch; initialized_ = true; valid_ = true; usedAlpha_ = alpha_; return true;
  }
  // Legacy fixed-alpha API is unchanged. Even when adaptive is enabled, this
  // three-argument overload always uses fixed alpha.
  bool update(float accelPitch, float gyroRate, float dt) {
    return fuse(accelPitch, gyroRate, dt, alpha_);
  }
  // Acceleration magnitude is an external measurement in g (not computed here).
  // Adaptive selection only applies when explicitly enabled.
  bool update(float accelPitch, float gyroRate, float dt, float accelMagnitudeG) {
    if (!eda_detail::finiteFloat(accelMagnitudeG) || accelMagnitudeG < 0) {
      valid_ = false; return false;
    }
    // Do not change mode on an invalid sample.
    if (!inputsValid(accelPitch, gyroRate, dt)) { valid_ = false; return false; }
    bool nextHigh = highAccel_;
    if (adaptive_) {
      const float deviation = fabsf(accelMagnitudeG - 1.0f);
      if (nextHigh ? (deviation <= exitDeviationG_) : (deviation >= enterDeviationG_))
        nextHigh = !nextHigh;
    }
    const float a = (adaptive_ && nextHigh) ? highAlpha_ : alpha_;
    if (!fuse(accelPitch, gyroRate, dt, a)) return false;
    highAccel_ = adaptive_ && nextHigh;
    return true;
  }
  float pitchDegrees() const { return angle_; }
  bool initialized() const { return initialized_; }
  bool valid() const { return valid_; }
private:
  static bool inputsValid(float accelPitch, float gyroRate, float dt) {
    return eda_detail::finiteFloat(accelPitch) && eda_detail::finiteFloat(gyroRate) &&
      eda_detail::finiteFloat(dt) && dt >= 0.0001f && dt <= 0.25f;
  }
  bool fuse(float accelPitch, float gyroRate, float dt, float a) {
    if (!inputsValid(accelPitch, gyroRate, dt)) { valid_ = false; return false; }
    if (!initialized_) return initialize(accelPitch);
    const float predicted = angle_ + (gyroRate - gyroBias_) * dt;
    const float fused = a * predicted + (1.0f - a) * accelPitch;
    if (!eda_detail::finiteFloat(predicted) || !eda_detail::finiteFloat(fused)) {
      valid_ = false; return false;
    }
    angle_ = fused; usedAlpha_ = a; valid_ = true; return true;
  }
  float alpha_, angle_, usedAlpha_, gyroBias_;
  bool initialized_, valid_, adaptive_, highAccel_;
  float highAlpha_, enterDeviationG_, exitDeviationG_;
};
#endif
