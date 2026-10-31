#include <EDADengeBot.h>
EDAPIDConfig cfg = edaDefaultConfig();
EDABalancePID balance(cfg);
void setup() {
  Serial.begin(115200);
  balance.reset();
  // Motor enable remains OFF until a deliberate arming sequence.
}
void loop() {
  // EXAMPLE ONLY: connect a real IMU and a suitable motor driver before operating.
  // Example IMU flow (pseudocode; no IMU driver included in this example):
  // 1) Read a NEW IMU sample and verify timestamp, validity and calibration.
  // 2) Compute pitchDeg in degrees; gyroPitchDps in degrees/second.
  //    Choose axis/sign so POSITIVE gyroPitchDps means pitchDeg increases.
  // 3) Apply gyro bias correction once (in the driver OR upstream filter).
  // 4) Compute dtSeconds from successive fresh IMU timestamps (seconds).
  // 5) float cmd = balance.update(targetDeg, pitchDeg,
  //                               gyroPitchDps, gyroPitchDps, dtSeconds);
  //    The two rates may be identical: pitchRate is an optional assist input.
  // 6) Only after freshness, angle and fault checks, map cmd to motor output.
  // Do NOT enable motors from this demonstration sketch.
  // if (!balance.lastUpdateValid() || balance.faulted()) { /* disable motor driver; require manual re-arm */ }
  delay(1000);
}
