// Host test: on non-AVR targets the demo must touch no pin at all (built WITHOUT ARDUINO_ARCH_AVR, and also for AVR boards other than Nano/Uno/Mega).
#include "Arduino.h"
unsigned long g_now=0; std::vector<PinEv> g_ev; int g_pwm[24]={0}; int g_dig[24]={0}; int g_mode[24]={0}; FakeSerial Serial;
#include "../examples/PwmCalibrationDemo/PwmCalibrationDemo.ino"
#include <cstdio>
int main(){ setup(); for(char c: std::string("1p23ynps")) Serial.in.push_back(c); for(int i=0;i<3000;i++){ g_now++; loop(); }
  printf("non-AVR: pin yazimi=%zu pinMode ayari=%d mesaj=\"%s\"\n", g_ev.size(), g_mode[5]+g_mode[6]+g_mode[4], Serial.out.substr(0,70).c_str()); return g_ev.empty()?0:1; }
