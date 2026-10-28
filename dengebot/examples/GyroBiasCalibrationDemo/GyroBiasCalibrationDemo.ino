#include <EDAComplementaryFilter.h>
#include <EDAGyroBiasCalibrator.h>
// Demonstration ONLY: synthetic gyro samples, NO motor output and NO pin driving.
// On a real robot: hold it perfectly still, feed raw gyro pitch-rate samples (deg/s) for ~1-2 s,
// apply the bias only if stationary() is true, then start balancing.
EDAComplementaryFilter pitchFilter(0.98f);
EDAGyroBiasCalibrator calibrator(100);
unsigned long lastMs = 0;
void setup() {
  Serial.begin(115200);
  lastMs = millis();
}
void loop() {
  const unsigned long now = millis();
  if ((unsigned long)(now - lastMs) < 10UL) return;
  lastMs = now;
  if (!calibrator.ready()) {
    const float syntheticStillGyroDps = 0.45f; // replace with the real, held-still gyro reading
    calibrator.addSample(syntheticStillGyroDps);
    return;
  }
  if (calibrator.stationary() && pitchFilter.setGyroBias(calibrator.bias())) {
    Serial.println("bias applied");
  } else {
    Serial.println("calibration rejected: robot moved or sensor fault");
  }
  calibrator.reset();
  delay(1000);
}
