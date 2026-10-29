#include <dengebot.h>
// Mapping demo only: no motor GPIO is driven.
// Start with wheels raised and choose values measured for YOUR motor.
const EDA::DeadZoneConfig leftMotor(48, 170, 0.5f);
const float commandLimit = 100.0f;
void setup() { Serial.begin(115200); }
void loop() {
  const float sampleCommand = 12.0f;
  const EDA::DCCommand result = EDA::toDCWithDeadZone(sampleCommand, commandLimit, leftMotor);
  Serial.print("direction="); Serial.print(result.direction);
  Serial.print(" pwm="); Serial.println(result.pwm);
  delay(1000);
}
