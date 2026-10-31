#include <dengebot.h>

// Compile-only API demonstration. No IMU, no motor pins, and NO motor enable.
EDABalancePID balance(edaDefaultConfig());
EDASafetyConfig safetyConfig = {30.0f, 50};
EDASafetyGuard safety(safetyConfig);

void setup() {
  Serial.begin(115200);
  balance.setZeroOutputOnFault(true);
  safety.stop(); // Default STOP; deliberately never arms without a real IMU.
}
void loop() {
  // Integration outline:
  // 1) Obtain a NEW IMU sample, record its real millis() timestamp.
  // 2) Validate sensor and convert pitch/rate to deg and deg/s.
  // 3) On a NEW operator START event: safety.requestStart(now, sampleMs, pitch, valid, balance);
  //    This can recover a latched FAULT without reboot, but only when safe.
  //    On a NEW STOP event: safety.stop() and disable motors immediately.
  // 4) Every cycle: safety.update(balance, now, sampleMs, valid,
  //                              target, pitch, rate, rate, dtSeconds);
  // 5) If !safety.armed(): set motor PWM=0 AND disable driver EN.
  // 6) On STOP: safety.stop() AND immediately disable driver EN.
  // This demo intentionally does not drive motors.
  delay(1000);
}
