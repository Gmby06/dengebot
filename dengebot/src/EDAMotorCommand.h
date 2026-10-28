#ifndef EDA_MOTOR_COMMAND_H
#define EDA_MOTOR_COMMAND_H
#include <stdint.h>
// Pure command translation; no GPIO or timer ownership.
// The caller must implement motor-disable, timing and driver-specific limits.
//
// V1.6.1 contract:
//  * Any non-finite input (NaN, +-Inf) or non-positive/non-finite limit yields a ZERO command.
//    (V1.6.0 saturated +-Inf commands; zero is the fail-safe choice, and the controller never outputs Inf.)
//  * direction == 0  <=>  zero output. A command that rounds to PWM 0 reports direction 0.
//  * The conversion is symmetric (f(-x) mirrors f(x)) and monotonic in |command|.
//  * Finite-ness is tested arithmetically; do not compile with -ffast-math.
namespace EDA {
struct DCCommand { int8_t direction; uint8_t pwm; };
struct StepperCommand { int8_t direction; float stepsPerSecond; };
inline bool isFiniteValue(float x) { return (x == x) && (x - x == 0.0f); }
inline float clampMagnitude(float x, float maxValue) {
  if (!isFiniteValue(maxValue) || !(maxValue > 0.0f) || !isFiniteValue(x)) return 0.0f;
  return x > maxValue ? maxValue : (x < -maxValue ? -maxValue : x);
}
inline DCCommand toDC(float command, float commandLimit, uint8_t maxPwm=255) {
  DCCommand out={0,0};
  if (!isFiniteValue(commandLimit) || !(commandLimit > 0.0f) || !isFiniteValue(command)) return out;
  const float x=clampMagnitude(command,commandLimit);
  if (x==0.0f) return out;
  const float magnitude=x>0.0f ? x : -x;
  // Ratio first (<=1), so large finite values cannot overflow.
  const uint8_t pwm=static_cast<uint8_t>((magnitude/commandLimit)*static_cast<float>(maxPwm)+0.5f);
  if (pwm==0) return out;
  out.direction=x>0.0f ? 1 : -1;
  out.pwm=pwm;
  return out;
}
inline StepperCommand toStepperVelocity(float command, float commandLimit, float maxStepsPerSecond) {
  StepperCommand out={0,0.0f};
  if (!isFiniteValue(commandLimit) || !(commandLimit>0.0f) ||
      !isFiniteValue(maxStepsPerSecond) || !(maxStepsPerSecond>0.0f) || !isFiniteValue(command)) return out;
  const float x=clampMagnitude(command,commandLimit);
  if (x==0.0f) return out;
  const float magnitude=x>0.0f ? x : -x;
  const float sps=(magnitude/commandLimit)*maxStepsPerSecond;
  if (!(sps>0.0f)) return out;
  out.direction=x>0.0f ? 1 : -1;
  out.stepsPerSecond=sps;
  return out;
}
}
#endif
