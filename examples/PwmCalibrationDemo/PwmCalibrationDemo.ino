#include <Arduino.h>
// This hardware-driving demonstration is active ONLY on the boards whose pin map it
// was written for: Arduino Nano / Uno / Mega (L298N example wiring, see below).
// Every other board - other AVR boards (Gemma/ATtiny85, Leonardo, ...), ESP8266,
// ESP32 - compiles an inert sketch that never touches a pin.
#if defined(ARDUINO_ARCH_AVR) && (defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO) || \
    defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560))
#include <EDAPwmCalibration.h>

// Hardware-specific EXAMPLE: Arduino Nano + L298N, wheels RAISED.
// Wiring verified for this robot: LEFT ENA=D5 IN1=D4 IN2=D7; RIGHT ENB=D6 IN3=D8 IN4=D9.
// VERIFY wiring, motor polarity, power and a physical motor disconnect first.
// Remove ENA/ENB jumpers. Motor pins below are EXAMPLES, not universal pinouts.
// Serial Monitor: 115200 baud, line ending = "No line ending" (CR/LF are ignored anyway).
const uint8_t ENA=5, IN1=4, IN2=7;   // Left motor, PWM-capable D5
const uint8_t ENB=6, IN3=8, IN4=9;  // Right motor, PWM-capable D6
// start 30, max 130, step 2, pulse 150 ms, cooldown 500 ms,
// idle timeout 15 s (waiting for a key), hard session cap 300 s.
EDA::PwmCalibration cal(EDA::PwmCalibrationConfig(30, 130, 2, 150, 500, 15000, 300000));

// What is currently written to the driver. Pins are touched ONLY when this changes,
// so PWM is not cut and re-armed every loop (that would distort the duty cycle).
struct Applied { int8_t wheel; int8_t dir; uint8_t pwm; };
Applied applied={-1,0,0};
EDA::CalibrationState lastState=EDA::CalibrationState::Idle;

void disableMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);          // EN first, then direction pins
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  applied.wheel=-1; applied.dir=0; applied.pwm=0;
}
void applyCommand(const EDA::DCCommand& c, EDA::CalibrationWheel wheel) {
  const bool off=(c.pwm==0 || c.direction==0);
  const int8_t w=off?-1:static_cast<int8_t>(wheel);
  const int8_t dir=off?0:c.direction;
  const uint8_t pwm=off?0:c.pwm;
  if (w==applied.wheel && dir==applied.dir && pwm==applied.pwm) return;  // unchanged: leave pins alone
  disableMotors();
  if (off) return;
  const bool forward=(dir>0);
  if (w==0) {
    digitalWrite(IN1,forward?HIGH:LOW); digitalWrite(IN2,forward?LOW:HIGH);   // direction before EN
    analogWrite(ENA,pwm);
  } else {
    digitalWrite(IN3,forward?HIGH:LOW); digitalWrite(IN4,forward?LOW:HIGH);
    analogWrite(ENB,pwm);
  }
  applied.wheel=w; applied.dir=dir; applied.pwm=pwm;
}
void help() {
  Serial.println(F("wheels UP, confirm wiring and disconnect power if unsafe"));
  Serial.println(F("1 left forward | 2 left reverse | 3 right forward | 4 right reverse"));
  Serial.println(F("p pulse | n no movement / increase PWM | y observed movement / save | s STOP"));
  Serial.println(F("No automatic motor movement on boot. EVERY pulse requires p."));
  Serial.println(F("15 s without a key press while READY ends the session (fault)."));
}
void reportFault() {
  switch (cal.fault()) {
    case EDA::CalibrationFault::IdleTimeout:  Serial.println(F("FAULT: idle timeout, motors off. Select a direction again.")); break;
    case EDA::CalibrationFault::Timeout:      Serial.println(F("FAULT: session cap reached, motors off.")); break;
    case EDA::CalibrationFault::LimitReached: Serial.println(F("FAULT: max PWM reached without confirmed movement.")); break;
    case EDA::CalibrationFault::Aborted:      Serial.println(F("FAULT: aborted by application.")); break;
    case EDA::CalibrationFault::InvalidConfig:Serial.println(F("FAULT: invalid calibration config.")); break;
    default: break;
  }
}
void setup() {
  // First put the motor-driver control pins into their inactive state.
  digitalWrite(ENA, LOW); digitalWrite(ENB, LOW);
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  disableMotors();
  Serial.begin(115200); help();
}
void loop() {
  // tick() advances the state machine; command(now) additionally refuses to drive
  // past the pulse time on its own. Neither replaces an independent hardware
  // disable if the MCU or this loop hangs.
  cal.tick(millis());
  if (Serial.available()) {
    const char c=static_cast<char>(Serial.read());
    const uint32_t now=millis();
    const bool known=(c=='s'||c=='n'||c=='y'||(c>='1'&&c<='4'));
    if (known) {
      disableMotors();                                // cut output before handling a command
      // If interrupted mid-pulse, NEVER resume the remainder automatically.
      if (cal.state()==EDA::CalibrationState::Pulsing) cal.abort();
    }
    if (c=='s') {cal.stop(); Serial.println(F("STOP"));}
    else if (c>='1' && c<='4') {
      EDA::CalibrationWheel w=(c=='1'||c=='2')?EDA::CalibrationWheel::Left:EDA::CalibrationWheel::Right;
      EDA::CalibrationDirection d=(c=='1'||c=='3')?EDA::CalibrationDirection::Forward:EDA::CalibrationDirection::Reverse;
      if(cal.begin(w,d,now)) Serial.println(F("READY: use p to request a bounded pulse"));
      else Serial.println(F("BUSY/INVALID; s to stop, then select direction"));
    } else if (c=='p') {
      Serial.println(cal.requestPulse(now)?F("PULSE START"):F("PULSE NOT ALLOWED"));
    } else if (c=='n') {
      Serial.println(cal.advance(now)?F("PWM INCREASED: press p"):F("NOT READY / LIMIT REACHED"));
    } else if (c=='y') {
      if(cal.confirmMovement(now)) {
        const EDA::PwmCalibrationResults& r=cal.results();
        Serial.print(F("CONFIRMED PWM=")); Serial.print(r.get(cal.wheel(),cal.direction()));
        Serial.print(F("  suggested dead-zone min PWM (lower confirmed direction, -15%)="));
        Serial.println(EDA::suggestDeadZoneMinPwm(r,cal.wheel()));
        Serial.println(F("This is a starting point, not a verified setting. Do not apply it automatically."));
      } else Serial.println(F("NO FINISHED PULSE TO CONFIRM"));
    }
    if (c=='1'||c=='2'||c=='3'||c=='4'||c=='p'||c=='n') {
      Serial.print(F("candidate PWM="));Serial.println(cal.candidatePwm());
    }
  }
  if (cal.state()!=lastState) {                      // report fault reasons once
    lastState=cal.state();
    if (lastState==EDA::CalibrationState::Fault) reportFault();
  }
  applyCommand(cal.command(millis()), cal.wheel());
}

#elif defined(ARDUINO_ARCH_AVR)
// Other AVR boards (e.g. Gemma / ATtiny85): the Nano pin map is NOT valid here and some
// cores (Gemma) have no 'Serial' object, so this sketch is intentionally empty.
void setup() {}
void loop() {}

#else
// Non-AVR boards: deliberately no motor control.
void setup() { Serial.begin(115200); Serial.println(F("PWM calibration hardware demo requires an AVR Nano/Uno/Mega; use a board-specific driver adapter.")); }
void loop() {}
#endif
