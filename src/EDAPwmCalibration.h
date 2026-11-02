#ifndef EDA_PWM_CALIBRATION_H
#define EDA_PWM_CALIBRATION_H
#include <stdint.h>
#include "EDAMotorCommand.h"

// Operator-observed, encoder-free PWM movement threshold calibration.
// This class owns NO motor pins. The application must physically disable
// driver PWM/EN on STOP/FAULT, missed calls, timeout, or reset.
//
// v1.7.5 candidate, revision 2:
//  - command(nowMs) checks the clock itself: a pulse never drives longer than
//    pulseMs even if tick() is called late (it cannot help if the loop hangs).
//  - The operator has an IDLE timeout (time spent waiting for a key press), not
//    a short total-session limit; a separate hard session cap still applies.
//  - New fault codes IdleTimeout and Aborted (appended; earlier values unchanged).
//  - suggestDeadZoneMinPwm() turns measured values into a conservative suggestion.
// v1.7.5 fixes:
//  - begin() invalidates the stored result of the selected wheel/direction, so a
//    re-run that is aborted/timed out never leaves the old value "confirmed".
//    Other wheel/direction slots are kept.
//  - idleTimeoutMs / maxSessionMs must be <= 0x7fffffff (the wrap-safe comparison
//    treats larger differences as errors), otherwise the config is invalid.
//  - PwmCalibrationResults::has()/get() reject invalid wheel/direction values.
namespace EDA {
enum class CalibrationWheel : uint8_t { Left=0, Right=1 };
enum class CalibrationDirection : int8_t { Reverse=-1, Forward=1 };
enum class CalibrationState : uint8_t { Idle, Ready, Pulsing, Cooldown, Complete, Fault };
enum class CalibrationFault : uint8_t { None, InvalidConfig, Timeout, LimitReached, IdleTimeout, Aborted };
struct PwmCalibrationConfig {
  uint8_t startPwm, maxPwm, stepPwm;
  uint16_t pulseMs, cooldownMs;
  uint32_t idleTimeoutMs;   // max wait for an operator action while READY
  uint32_t maxSessionMs;    // hard cap for one begin()..result session
  PwmCalibrationConfig(uint8_t start=30, uint8_t max=160, uint8_t step=2,
                       uint16_t pulse=120, uint16_t cooldown=400,
                       uint32_t idle=15000, uint32_t session=300000)
    : startPwm(start), maxPwm(max), stepPwm(step), pulseMs(pulse),
      cooldownMs(cooldown), idleTimeoutMs(idle), maxSessionMs(session) {}
};
struct PwmCalibrationResults {
  uint8_t pwm[2][2];
  bool confirmed[2][2];
  PwmCalibrationResults() : pwm(), confirmed() {}
  // An out-of-range wheel or direction value (e.g. from a bad cast) is reported
  // as "not confirmed" (has()==false, get()==0) instead of indexing out of bounds.
  bool has(CalibrationWheel wheel, CalibrationDirection direction) const {
    const uint8_t w=static_cast<uint8_t>(wheel);
    if (w>1) return false;
    if (direction!=CalibrationDirection::Forward && direction!=CalibrationDirection::Reverse) return false;
    return confirmed[w][direction==CalibrationDirection::Forward?1:0];
  }
  uint8_t get(CalibrationWheel wheel, CalibrationDirection direction) const {
    if (!has(wheel,direction)) return 0;
    return pwm[static_cast<uint8_t>(wheel)][direction==CalibrationDirection::Forward?1:0];
  }
  // Forget one measurement (used by PwmCalibration::begin() so an aborted re-run
  // cannot leave the previous value looking freshly confirmed).
  void clear(CalibrationWheel wheel, CalibrationDirection direction) {
    const uint8_t w=static_cast<uint8_t>(wheel);
    if (w>1) return;
    if (direction!=CalibrationDirection::Forward && direction!=CalibrationDirection::Reverse) return;
    const uint8_t d=direction==CalibrationDirection::Forward?1:0;
    pwm[w][d]=0; confirmed[w][d]=false;
  }
};

// Conservative suggestion for DeadZoneConfig::minEffectivePwm for ONE wheel.
// Uses the LOWER confirmed direction (DeadZoneConfig is symmetric) and subtracts
// marginPercent (0..50, default 15). Returns 0 when nothing is confirmed.
// Notes: the measured value is a short-pulse BREAKAWAY threshold of an unloaded wheel,
// rounded up by at most stepPwm-1; sustained/loaded motion differs. In the project's
// synthetic simulation an overestimated minimum (1.3x) caused strong chatter around
// the center while an underestimate (0.7x) did not. Treat the result as a starting
// point and verify on the robot; never apply it automatically.
inline uint8_t suggestDeadZoneMinPwm(const PwmCalibrationResults& r, CalibrationWheel wheel,
                                     uint8_t marginPercent=15) {
  const bool f=r.has(wheel,CalibrationDirection::Forward);
  const bool b=r.has(wheel,CalibrationDirection::Reverse);
  if (!f && !b) return 0;
  const uint8_t fw=r.get(wheel,CalibrationDirection::Forward);
  const uint8_t bw=r.get(wheel,CalibrationDirection::Reverse);
  const uint8_t base=(f&&b)?(fw<bw?fw:bw):(f?fw:bw);
  if (marginPercent>50) marginPercent=50;
  uint16_t v=static_cast<uint16_t>(static_cast<uint16_t>(base)*(100-marginPercent)/100);
  if (v<1) v=1;
  return static_cast<uint8_t>(v);
}

class PwmCalibration {
public:
  explicit PwmCalibration(const PwmCalibrationConfig& cfg=PwmCalibrationConfig())
    : cfg_(cfg), state_(valid(cfg)?CalibrationState::Idle:CalibrationState::Fault),
      fault_(valid(cfg)?CalibrationFault::None:CalibrationFault::InvalidConfig),
      wheel_(CalibrationWheel::Left), direction_(CalibrationDirection::Forward),
      pwm_(cfg.startPwm), sessionStart_(0), phaseStart_(0), idleStart_(0) {}
  static bool valid(const PwmCalibrationConfig& c) {
    return c.startPwm>0 && c.startPwm<=c.maxPwm && c.stepPwm>0 &&
           c.pulseMs>0 && c.cooldownMs>0 &&
           c.idleTimeoutMs>0 && c.idleTimeoutMs<=0x7ffffffful &&
           c.maxSessionMs>0 && c.maxSessionMs<=0x7ffffffful &&
           static_cast<uint32_t>(c.pulseMs)+c.cooldownMs < c.maxSessionMs;
  }
  CalibrationState state() const {return state_;}
  CalibrationFault fault() const {return fault_;}
  uint8_t candidatePwm() const {return pwm_;}
  CalibrationWheel wheel() const {return wheel_;}
  CalibrationDirection direction() const {return direction_;}
  const PwmCalibrationResults& results() const {return results_;}
  // An explicit new operator event; never call continuously for a held button.
  // Only one motor/direction is tested at a time.
  bool begin(CalibrationWheel wheel, CalibrationDirection direction, uint32_t nowMs) {
    if (!valid(cfg_) || state_==CalibrationState::Pulsing || state_==CalibrationState::Cooldown ||
        static_cast<uint8_t>(wheel)>1 ||
        (direction!=CalibrationDirection::Forward && direction!=CalibrationDirection::Reverse)) return false;
    results_.clear(wheel,direction);   // a new attempt invalidates the old measurement of this slot
    wheel_=wheel; direction_=direction; pwm_=cfg_.startPwm;
    sessionStart_=nowMs; phaseStart_=nowMs; idleStart_=nowMs;
    state_=CalibrationState::Ready; fault_=CalibrationFault::None; pulsed_=false;
    return true;
  }
  // One deliberate button edge triggers a bounded motor test pulse.
  bool requestPulse(uint32_t nowMs) {
    tick(nowMs);
    if (state_!=CalibrationState::Ready) return false;
    phaseStart_=nowMs; state_=CalibrationState::Pulsing; pulsed_=true;
    return true;
  }
  // Advances the state machine. Call every loop BEFORE command(nowMs).
  void tick(uint32_t nowMs) {
    if (!active()) return;
    const uint32_t age=nowMs-sessionStart_;
    if (age>=cfg_.maxSessionMs || age>0x7ffffffful) { fail(CalibrationFault::Timeout); return; }
    uint32_t phaseAge=nowMs-phaseStart_;
    if (phaseAge>0x7ffffffful) { fail(CalibrationFault::Timeout); return; }
    if (state_==CalibrationState::Pulsing && phaseAge>=cfg_.pulseMs) {
      state_=CalibrationState::Cooldown; phaseStart_+=cfg_.pulseMs; phaseAge=nowMs-phaseStart_;
    }
    if (state_==CalibrationState::Cooldown && phaseAge>=cfg_.cooldownMs) {
      state_=CalibrationState::Ready; idleStart_=phaseStart_+cfg_.cooldownMs;
    }
    if (state_==CalibrationState::Ready) {
      const uint32_t idleAge=nowMs-idleStart_;
      if (idleAge>0x7ffffffful || idleAge>=cfg_.idleTimeoutMs) fail(CalibrationFault::IdleTimeout);
    }
  }
  // Motor command for time nowMs. Zero unless a pulse is running AND its time has
  // not elapsed, even if tick() has not been called yet. Cannot stop a hung loop.
  DCCommand command(uint32_t nowMs) const {
    DCCommand zero={0,0};
    if (state_!=CalibrationState::Pulsing) return zero;
    const uint32_t phaseAge=nowMs-phaseStart_;
    if (phaseAge>=cfg_.pulseMs || phaseAge>0x7ffffffful) return zero;
    const uint32_t age=nowMs-sessionStart_;
    if (age>=cfg_.maxSessionMs || age>0x7ffffffful) return zero;
    DCCommand out={static_cast<int8_t>(direction_),pwm_};
    return out;
  }
  // User must visually confirm movement; this function never guesses it.
  // Confirmation is permitted after a pulse, not merely on begin().
  bool confirmMovement(uint32_t nowMs) {
    tick(nowMs);
    if (state_!=CalibrationState::Cooldown && state_!=CalibrationState::Ready) return false;
    if (!pulsed_) return false;
    const uint8_t w=static_cast<uint8_t>(wheel_);
    const uint8_t d=direction_==CalibrationDirection::Forward?1:0;
    results_.pwm[w][d]=pwm_; results_.confirmed[w][d]=true;
    state_=CalibrationState::Complete; pulsed_=false;
    return true;
  }
  // Operator reports no movement. Only allowed after pulse has ended.
  bool advance(uint32_t nowMs) {
    tick(nowMs);
    if (state_!=CalibrationState::Ready || !pulsed_) return false;
    if (static_cast<uint16_t>(pwm_)+cfg_.stepPwm>cfg_.maxPwm) {
      fail(CalibrationFault::LimitReached); return false;
    }
    pwm_=static_cast<uint8_t>(pwm_+cfg_.stepPwm);
    pulsed_=false; idleStart_=nowMs;
    return true;
  }
  void stop() {state_=CalibrationState::Idle; fault_=CalibrationFault::None; pulsed_=false;}
  // Application can invoke if driver / sensor / power supply faults occur.
  void abort() {fail(CalibrationFault::Aborted);}
  bool pulseObserved() const {return pulsed_;}
private:
  bool active() const {return state_==CalibrationState::Ready || state_==CalibrationState::Pulsing || state_==CalibrationState::Cooldown;}
  void fail(CalibrationFault why) {state_=CalibrationState::Fault; fault_=why; pulsed_=false;}
  PwmCalibrationConfig cfg_;
  CalibrationState state_;
  CalibrationFault fault_;
  CalibrationWheel wheel_;
  CalibrationDirection direction_;
  uint8_t pwm_;
  uint32_t sessionStart_,phaseStart_,idleStart_;
  bool pulsed_=false;
  PwmCalibrationResults results_;
};
} // namespace EDA
#endif
