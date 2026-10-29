#ifndef EDA_DEAD_ZONE_COMPENSATION_H
#define EDA_DEAD_ZONE_COMPENSATION_H
#include "EDAMotorCommand.h"
namespace EDA {
// Optional, stateless DC motor mapping. The existing toDC() API is unchanged.
// A nonzero input maps into [minEffectivePwm,maxPwm], while exact zero stays off.
// A configurable command threshold avoids unintended movement near zero.
struct DeadZoneConfig {
  uint8_t minEffectivePwm;
  uint8_t maxPwm;
  float zeroCommandThreshold;
  DeadZoneConfig(uint8_t minPwm=0, uint8_t maxOutput=255, float threshold=0.0f)
    : minEffectivePwm(minPwm), maxPwm(maxOutput), zeroCommandThreshold(threshold) {}
};
inline bool validDeadZoneConfig(const DeadZoneConfig& cfg, float commandLimit) {
  return isFiniteValue(commandLimit) && commandLimit>0.0f &&
         cfg.maxPwm>0 && cfg.minEffectivePwm<=cfg.maxPwm &&
         isFiniteValue(cfg.zeroCommandThreshold) && cfg.zeroCommandThreshold>=0.0f &&
         cfg.zeroCommandThreshold<commandLimit;
}
inline DCCommand toDCWithDeadZone(float command, float commandLimit, const DeadZoneConfig& cfg) {
  DCCommand out={0,0};
  if (!validDeadZoneConfig(cfg,commandLimit) || !isFiniteValue(command)) return out;
  const float x=clampMagnitude(command,commandLimit);
  const float mag=x<0.0f?-x:x;
  if (mag<=cfg.zeroCommandThreshold) return out;
  const float ratio=(mag-cfg.zeroCommandThreshold)/(commandLimit-cfg.zeroCommandThreshold);
  const float span=static_cast<float>(cfg.maxPwm-cfg.minEffectivePwm);
  const float raw=static_cast<float>(cfg.minEffectivePwm)+ratio*span;
  const uint8_t pwm=static_cast<uint8_t>(raw+0.5f);
  if (pwm==0) return out;
  out.direction=x>0.0f?1:-1;
  out.pwm=pwm;
  return out;
}
}
#endif
