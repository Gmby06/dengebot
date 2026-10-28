#include <EDAComplementaryFilter.h>
#include <EDATelemetry.h>
#include <EDADengeBot.h>

// Demonstration ONLY: synthetic sensor samples, NO motor output and NO pin driving.
// Replace synthetic values with a calibrated IMU before any physical experiment.
EDAComplementaryFilter pitchFilter(0.98f);
EDABalancePID balance(edaDefaultConfig());
unsigned long previousMs = 0;
unsigned long lastTelemetryMs = 0;
void setup() {
  Serial.begin(115200);
  balance.setZeroOutputOnFault(true);
  balance.reset();
  previousMs = millis();
}
void loop() {
  const unsigned long now = millis();
  const unsigned long elapsed = now - previousMs;
  if (elapsed < 10) return;
  previousMs = now;
  const float dt = elapsed * 0.001f;
  const float syntheticAccelPitch = 0.0f;
  const float syntheticGyroRate = 0.0f;
  if (!pitchFilter.update(syntheticAccelPitch, syntheticGyroRate, dt)) return;
  const float output = balance.update(0.0f, pitchFilter.pitchDegrees(),
                                      syntheticGyroRate, syntheticGyroRate, dt);
  EDATelemetryFrame frame = {(uint32_t)now, 0.0f, pitchFilter.pitchDegrees(),
                             syntheticGyroRate, output, 0.0f,
                             (uint32_t)(elapsed * 1000UL), 0, (uint8_t)balance.fault()};
  if ((unsigned long)(now - lastTelemetryMs) >= 100UL) {
    lastTelemetryMs = now;
    EDATelemetry::printPlotter(Serial, frame); // synthetic demo only; can block
  }
}
