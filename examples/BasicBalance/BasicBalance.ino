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
  // float cmd = balance.update(targetDeg, pitchDeg, gyroDegPerSec, gyroDegPerSec, dtSeconds);
  // if (!balance.lastUpdateValid() || balance.faulted()) { /* disable motor driver; require manual re-arm */ }
  delay(1000);
}
