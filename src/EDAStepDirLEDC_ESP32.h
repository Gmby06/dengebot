#ifndef EDA_STEP_DIR_LEDC_ESP32_H
#define EDA_STEP_DIR_LEDC_ESP32_H

// Experimental v1.7.6 ESP32 (classic) STEP/DIR hardware backend.
// Hardware LEDC creates 50% duty STEP pulses, without a 20-us ISR.
// This is NOT a hardware independent watchdog or a universal stepper driver.
// NOTE: ledcWriteTone() uses 10-bit resolution; on classic ESP32 (80 MHz APB)
// the core may reject frequencies below ~77 Hz (command() then disables and
// returns false). Verify on hardware before using very low STEP rates.
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32)
#include <Arduino.h>
#include <esp_arduino_version.h>

namespace EDA {
class StepDirLEDC_ESP32 {
public:
  StepDirLEDC_ESP32(uint8_t ls, uint8_t ld, uint8_t rs, uint8_t rd,
                   uint8_t enablePin, bool leftForwardHigh, bool rightForwardHigh)
    : lStep_(ls), lDir_(ld), rStep_(rs), rDir_(rd), enable_(enablePin),
      lForwardHigh_(leftForwardHigh), rForwardHigh_(rightForwardHigh),
      ready_(false), running_(false), lastCommandMs_(0), leftRate_(0), rightRate_(0),
      leftSigned_(0), rightSigned_(0) {}

  // Requires an EXTERNAL pull-up to 3.3V on ENABLE for a reset-safe default.
  bool begin() {
    digitalWrite(enable_, HIGH);
    pinMode(enable_, OUTPUT);
    digitalWrite(lStep_, LOW); digitalWrite(rStep_, LOW);
    pinMode(lStep_, OUTPUT); pinMode(rStep_, OUTPUT);
    digitalWrite(lDir_, lForwardHigh_ ? HIGH : LOW);
    digitalWrite(rDir_, rForwardHigh_ ? HIGH : LOW);
    pinMode(lDir_, OUTPUT); pinMode(rDir_, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    const bool okL=ledcAttachChannel(lStep_, 2000, 8, 0);
    const bool okR=ledcAttachChannel(rStep_, 2000, 8, 2);
    ready_=okL && okR;
#else
    // Channel 0 and channel 2 use distinct LEDC timers on classic ESP32.
    const double fL=ledcSetup(0, 2000, 8);
    const double fR=ledcSetup(2, 2000, 8);
    ready_= fL>0 && fR>0;
    if (ready_) { ledcAttachPin(lStep_, 0); ledcAttachPin(rStep_, 2); }
#endif
    disable();
    return ready_;
  }

  bool ready() const { return ready_; }
  bool running() const { return running_; }
  uint32_t leftRate() const { return leftRate_; }
  uint32_t rightRate() const { return rightRate_; }

  // NOTE: A hardware safety circuit is required for MCU lockups / reset.
  void disable() {
    digitalWrite(enable_, HIGH);
    stopPulses();
    running_=false; leftRate_=0; rightRate_=0;
    leftSigned_=rightSigned_=0;
  }

  // Signed STEP/s. Zero on one side means no pulses on that side.
  // FIX3: ENABLE is NOT toggled for a pure frequency change, so the driver
  // keeps its coil current while the rate ramps. Only a direction REVERSAL
  // (sign flip while running) uses the full disable -> DIR -> enable path.
  // A side that starts from standstill sets its own DIR (its STEP is idle)
  // and waits the DIR setup time before the first pulse.
  bool command(int32_t left, int32_t right, uint32_t nowMs) {
    if (!ready_ || left < -40000 || left > 40000 ||
        right < -40000 || right > 40000) { disable(); return false; }
    // A refresh with unchanged speeds must NOT restart LEDC or blank STEP.
    if (running_ && left==leftSigned_ && right==rightSigned_) {
      lastCommandMs_=nowMs; return true;
    }
    const uint32_t l=(left<0)?(uint32_t)(-left):(uint32_t)left;
    const uint32_t r=(right<0)?(uint32_t)(-right):(uint32_t)right;
    const bool flip=(leftSigned_>0 && left<0) || (leftSigned_<0 && left>0) ||
                    (rightSigned_>0 && right<0) || (rightSigned_<0 && right>0);
    if (flip) {
      // Reversal: driver off, pulses off, then DIR, then enable and restart both.
      digitalWrite(enable_, HIGH);
      stopPulse(true); stopPulse(false);
      digitalWrite(lDir_, left<0 ? !lForwardHigh_ : lForwardHigh_);
      digitalWrite(rDir_, right<0 ? !rForwardHigh_ : rForwardHigh_);
      delayMicroseconds(50);
      if (l || r) {
        digitalWrite(enable_, LOW);
        delayMicroseconds(20);
        if ((l && !startTone(true,l)) || (r && !startTone(false,r))) {
          disable(); return false;
        }
      }
    } else {
      // Sides that start from standstill: set DIR while their STEP is idle.
      const bool startL=(l && leftSigned_==0), startR=(r && rightSigned_==0);
      if (startL) digitalWrite(lDir_, left<0 ? !lForwardHigh_ : lForwardHigh_);
      if (startR) digitalWrite(rDir_, right<0 ? !rForwardHigh_ : rForwardHigh_);
      if (startL || startR) delayMicroseconds(50);
      if ((l || r) && !running_) {   // driver was off: enable before first STEP
        digitalWrite(enable_, LOW);
        delayMicroseconds(20);
      }
      if (left!=leftSigned_) {
        if (!l) stopPulse(true);
        else if (!startTone(true,l)) { disable(); return false; }
      }
      if (right!=rightSigned_) {
        if (!r) stopPulse(false);
        else if (!startTone(false,r)) { disable(); return false; }
      }
      if (!l && !r) digitalWrite(enable_, HIGH);
    }
    leftRate_=l; rightRate_=r; running_=(l || r);
    leftSigned_=left; rightSigned_=right;
    lastCommandMs_=nowMs;
    return true;
  }

  // Application calls this frequently. Not sufficient if main loop hangs.
  bool watchdog(uint32_t nowMs, uint32_t maxAgeMs=150) {
    if (running_ && (uint32_t)(nowMs-lastCommandMs_)>maxAgeMs) {
      disable(); return false;
    }
    return true;
  }

private:
  uint8_t lStep_,lDir_,rStep_,rDir_,enable_;
  bool lForwardHigh_,rForwardHigh_,ready_,running_;
  uint32_t lastCommandMs_,leftRate_,rightRate_;
  int32_t leftSigned_,rightSigned_;
  void stopPulse(bool left) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(left?lStep_:rStep_,0);
#else
    ledcWrite(left?0:2,0);
#endif
  }
  void stopPulses() { stopPulse(true); stopPulse(false); }
  bool startTone(bool left,uint32_t hz) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    const uint8_t pin=left?lStep_:rStep_;
    return ledcWriteTone(pin,hz)>0;
#else
    const uint8_t ch=left?0:2;
    return ledcWriteTone(ch,hz)>0;
#endif
  }
};
}
#endif
#endif
