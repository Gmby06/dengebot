#ifndef EDA_STEP_DIR_CONTROLLER_H
#define EDA_STEP_DIR_CONTROLLER_H
#include <stdint.h>
#include "EDAMotorCommand.h"

// v1.7.6 DEVELOPMENT - architecture-neutral, allocation-free pulse scheduler.
// tick() must be called at a PRECISE, fixed interval from an application-owned
// hardware timer. This class NEVER owns GPIO, interrupts or motor power.
// The application must enforce STEP/DIR electrical timing and independent
// motor-driver disable; its timer must keep running during a blocked loop.
namespace EDA {
struct StepDirConfig {
  uint32_t tickUs;
  uint32_t maxStepsPerSecond;
  uint32_t commandTimeoutMs;
  uint16_t directionSetupUs;
  StepDirConfig(uint32_t tick=20, uint32_t maxSps=4000,
                uint32_t timeout=150, uint16_t dirSetup=40)
    : tickUs(tick), maxStepsPerSecond(maxSps),
      commandTimeoutMs(timeout), directionSetupUs(dirSetup) {}
};
struct StepDirPins {
  bool stepHigh;
  int8_t direction; // -1 / 0 / +1; 0 means no direction has been selected
};
class StepDirController {
public:
  explicit StepDirController(const StepDirConfig& c=StepDirConfig())
    : cfg_(c), valid_(validConfig(c)), armed_(false), timedOut_(false),
      lastCommandTick_(0), clockTicks_(0),
      timeoutTicks_((c.commandTimeoutMs*1000ul)/(c.tickUs ? c.tickUs : 1)) {
    for (int i=0;i<2;++i) {
      motors_[i].desiredDirection=0; motors_[i].physicalDirection=0;
      motors_[i].targetSps=0; motors_[i].phase=0;
      motors_[i].setupTicksLeft=0; motors_[i].stepHigh=false;
    }
  }
  static bool validConfig(const StepDirConfig& c) {
    // 1-tick STEP HIGH, >=1-tick LOW. Tick period must not be sub-us or too slow.
    if (c.tickUs<2 || c.tickUs>100 || c.maxStepsPerSecond==0 ||
        c.maxStepsPerSecond>1000000ul/(2ul*c.tickUs) ||
        c.directionSetupUs<c.tickUs || c.commandTimeoutMs==0 ||
        c.commandTimeoutMs>60000) return false;
    return true;
  }
  bool configValid() const { return valid_; }
  bool armed() const { return armed_; }
  bool timedOut() const { return timedOut_; }
  uint32_t clockTicks() const { return clockTicks_; }
  // Only call from the timing owner or while holding its interrupt lock.
  bool arm() {
    if (!valid_) return false;
    stop(); timedOut_=false; armed_=true; lastCommandTick_=clockTicks_; return true;
  }
  void stop() {
    armed_=false;
    for (int i=0;i<2;++i) {
      motors_[i].targetSps=0; motors_[i].desiredDirection=0;
      motors_[i].phase=0; motors_[i].setupTicksLeft=0;
      motors_[i].stepHigh=false;
    }
  }
  // Reissue a fresh command at least once per timeout interval; this is NOT
  // an emergency-stop substitute if the timer or MCU itself stops.
  bool setSpeeds(float leftSps, float rightSps) {
    if (!armed_ || !valid_ || !isFiniteValue(leftSps) || !isFiniteValue(rightSps)) return false;
    // FIX3: over-limit requests are CLAMPED to maxStepsPerSecond (previously
    // rejected, which let the command timeout stop the motors when a PID output
    // saturated). Non-finite values are still rejected.
    setOne(0,leftSps); setOne(1,rightSps);
    lastCommandTick_=clockTicks_;
    return true;
  }
  // Called exactly once per tick (e.g. every 20us) from the hardware timer ISR.
  // Outputs the STEP level and direction to apply at this instant.
  // No floats, heap allocation, division, GPIO, or Arduino dependencies here.
  void tick(StepDirPins& left, StepDirPins& right) {
    ++clockTicks_;
    if (armed_ && static_cast<uint32_t>(clockTicks_-lastCommandTick_) > timeoutTicks_) {
      timedOut_=true; stop();
    }
    left = tickOne(0);
    right = tickOne(1);
  }
private:
  struct Motor {
    uint32_t targetSps, phase, setupTicksLeft;
    int8_t desiredDirection, physicalDirection;
    bool stepHigh;
  };
  StepDirConfig cfg_;
  Motor motors_[2];
  bool valid_, armed_, timedOut_;
  uint32_t lastCommandTick_, clockTicks_, timeoutTicks_;
  void setOne(int n, float speed) {
    float magnitude=speed < 0.0f ? -speed : speed;
    const float limit=static_cast<float>(cfg_.maxStepsPerSecond);
    if (magnitude>limit) magnitude=limit;
    Motor& m=motors_[n];
    const uint32_t rounded=static_cast<uint32_t>(magnitude+0.5f);
    const int8_t dir=rounded==0?0:(speed>0.0f?1:-1);
    m.targetSps=rounded;
    m.desiredDirection=dir;
  }
  StepDirPins tickOne(int n) {
    Motor& m=motors_[n];
    if (!armed_ || !m.targetSps || !m.desiredDirection) {
      m.phase=0; m.stepHigh=false;
      return {false,m.physicalDirection};
    }
    if (m.stepHigh) {
      m.phase += m.targetSps*cfg_.tickUs; // accumulate even while STEP is falling
      m.stepHigh=false; // one complete HIGH tick
      return {false,m.physicalDirection};
    }
    if (m.desiredDirection != m.physicalDirection) {
      m.physicalDirection=m.desiredDirection;
      m.setupTicksLeft=(cfg_.directionSetupUs+cfg_.tickUs-1)/cfg_.tickUs;
      m.phase=0;
      return {false,m.physicalDirection};
    }
    if (m.setupTicksLeft) {
      --m.setupTicksLeft;
      return {false,m.physicalDirection};
    }
    // Phase accumulator eliminates coarse integer period rounding.
    m.phase += m.targetSps*cfg_.tickUs;
    if (m.phase >= 1000000ul) {
      m.phase-=1000000ul;
      m.stepHigh=true;
    }
    return {m.stepHigh,m.physicalDirection};
  }
};
}
#endif
