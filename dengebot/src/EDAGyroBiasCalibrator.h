#ifndef EDA_GYRO_BIAS_CALIBRATOR_H
#define EDA_GYRO_BIAS_CALIBRATOR_H
#include "EDANumeric.h"
#include <stdint.h>
// Startup gyro-bias estimate. Feed samples ONLY while the robot is held perfectly still.
// Running mean/variance (Welford), no allocation. This is not a drift tracker: re-calibrate
// after warm-up or temperature changes. Units: degrees/second.
class EDAGyroBiasCalibrator {
public:
  explicit EDAGyroBiasCalibrator(uint32_t minSamples = 100, float maxStdDevDps = 1.0f,
                                 float maxAbsBiasDps = 5.0f)
    : minSamples_(100), maxStd_(1.0f), maxAbs_(5.0f) { configure(minSamples, maxStdDevDps, maxAbsBiasDps); reset(); }
  bool configure(uint32_t minSamples, float maxStdDevDps, float maxAbsBiasDps) {
    if (minSamples < 10 || !eda_detail::finiteFloat(maxStdDevDps) || maxStdDevDps <= 0 ||
        !eda_detail::finiteFloat(maxAbsBiasDps) || maxAbsBiasDps <= 0) return false;
    minSamples_ = minSamples; maxStd_ = maxStdDevDps; maxAbs_ = maxAbsBiasDps; return true;
  }
  void reset() { n_ = 0; mean_ = 0; m2_ = 0; invalid_ = false; }
  // A non-finite sample poisons the run (failed() == true) until reset(); it is not accumulated.
  bool addSample(float gyroDps) {
    if (!eda_detail::finiteFloat(gyroDps)) { invalid_ = true; return false; }
    if (invalid_) return false;
    if (n_ == 0xFFFFFFFFu) return true;           // saturate; estimate is long since settled
    n_++;
    const float d = gyroDps - mean_;
    mean_ += d / (float)n_;
    m2_ += d * (gyroDps - mean_);
    return true;
  }
  uint32_t samples() const { return n_; }
  bool failed() const { return invalid_; }
  bool ready() const { return !invalid_ && n_ >= minSamples_; }
  float bias() const { return mean_; }
  float stdDev() const { return n_ > 1 ? sqrtf(m2_ / (float)(n_ - 1)) : 0.0f; }
  // true only if enough samples AND the readings look like a still robot with a plausible bias.
  // If false after ready(): the robot moved or the sensor is faulty -> do NOT apply the bias.
  bool stationary() const { return ready() && stdDev() <= maxStd_ && fabsf(mean_) <= maxAbs_; }
private:
  uint32_t minSamples_; float maxStd_, maxAbs_;
  uint32_t n_; float mean_, m2_; bool invalid_;
};
#endif
