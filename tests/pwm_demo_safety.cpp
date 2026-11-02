#include "EDAPwmCalibration.h"
#include <cassert>
int main() {
  EDA::PwmCalibration cal;
  assert(cal.begin(EDA::CalibrationWheel::Left,EDA::CalibrationDirection::Forward,100));
  assert(cal.requestPulse(100));
  assert(cal.command(150).pwm>0);
  // Library-level check only. That the demo sketch itself aborts on a mid-pulse key press is
  // covered by tests/demo_harness.cpp.
  cal.abort();
  assert(cal.command(151).pwm==0);
  cal.tick(152);
  assert(cal.command(152).pwm==0);
  assert(!cal.requestPulse(153));
  assert(cal.begin(EDA::CalibrationWheel::Right,EDA::CalibrationDirection::Reverse,154));
  assert(cal.command(154).pwm==0);
  return 0;
}
