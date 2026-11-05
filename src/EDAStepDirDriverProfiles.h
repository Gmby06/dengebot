#ifndef EDA_STEP_DIR_DRIVER_PROFILES_H
#define EDA_STEP_DIR_DRIVER_PROFILES_H
#include <stdint.h>

// Informational profiles for STEP/DIR drivers. These do not reconfigure
// microstep straps, motor current, SLEEP or RESET electrical wiring.
// FIX3 LEDC backend remains unchanged and its existing API is retained.
namespace EDA {
enum class StepDirDriverType : uint8_t { A4988=0, DRV8825=1 };
struct StepDirDriverProfile {
  StepDirDriverType type;
  const char *name;
  uint16_t minStepHighNs;
  uint16_t minStepLowNs;
  uint16_t minDirSetupNs;
  uint16_t minDirHoldNs;
  uint16_t minEnableSetupNs;
  uint32_t maxDeviceStepHz;
  uint8_t maxMicrosteps;
  bool enableActiveLow;
};
// A4988 STEP min high/low 1us, DIR setup/hold 200ns (Allegro datasheet).
static constexpr StepDirDriverProfile A4988_PROFILE = {
  StepDirDriverType::A4988, "A4988", 1000, 1000, 200, 200, 1000,
  40000, 16, true
};
// DRV8825 (TI SLVSA73F, Timing Requirements 7.6).
// 250kHz is the chip limit, not the limit of our FIX3 backend (40kHz).
static constexpr StepDirDriverProfile DRV8825_PROFILE = {
  StepDirDriverType::DRV8825, "DRV8825", 1900, 1900, 650, 650, 650,
  250000, 32, true
};
constexpr bool supportsFix3Ledc(const StepDirDriverProfile &p,
                                uint32_t hz) {
  return hz > 0 && hz <= 40000 && hz <= p.maxDeviceStepHz &&
         (500000000ULL/hz)>=p.minStepHighNs &&
         (500000000ULL/hz)>=p.minStepLowNs;
}
constexpr const StepDirDriverProfile &driverProfile(StepDirDriverType t) {
  return t==StepDirDriverType::DRV8825 ? DRV8825_PROFILE : A4988_PROFILE;
}
} // namespace EDA
#endif
