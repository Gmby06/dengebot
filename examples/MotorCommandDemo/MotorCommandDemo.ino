#include <EDADengeBot.h>
#include <EDAMotorCommand.h>
// DEMO ONLY: prints how controller output maps to DC / stepper commands.
// It drives NO pins and NO motors. Direction/PWM semantics must still be verified
// on a real robot with the wheels lifted.
EDAPIDConfig cfg = edaDefaultConfig();
EDABalancePID balance(cfg);
void setup() {
  Serial.begin(115200);
  balance.reset();
  const float cmd = balance.update(0.0f, 2.0f, 0.0f, 0.0f, 0.01f);
  const EDA::DCCommand dc = EDA::toDC(cmd, cfg.outputLimit);
  const EDA::StepperCommand st = EDA::toStepperVelocity(cmd, cfg.outputLimit, 2000.0f);
  Serial.print("cmd=");  Serial.println(cmd);
  Serial.print("dc dir="); Serial.print((int)dc.direction); Serial.print(" pwm="); Serial.println((int)dc.pwm);
  Serial.print("step dir="); Serial.print((int)st.direction); Serial.print(" sps="); Serial.println(st.stepsPerSecond);
  if (!balance.configValid() || balance.faulted()) { Serial.println("FAULT: keep motors disabled"); }
}
void loop() {
  delay(1000);
}
