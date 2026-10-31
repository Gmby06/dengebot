#include <dengebot.h>
#include <cassert>
// Every public module must be reachable through the single <dengebot.h> header.
int main() {
  EDABalancePID pid(edaDefaultConfig());
  EDASafetyConfig sc={30.0f,50}; EDASafetyGuard guard(sc);
  EDA::DeadZoneConfig dz(48,170,0.25f);
  assert(EDA::toDCWithDeadZone(0.0f,200.0f,dz).pwm==0);
  assert(EDA::toDC(0.0f,200.0f).pwm==0);
  EDAComplementaryFilter filter; (void)filter;
  EDATelemetry telemetry; (void)telemetry;
  EDAGyroBiasCalibrator cal; (void)cal;
  (void)pid; (void)guard;
  return 0;
}
