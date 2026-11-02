// Host test: runs examples/PwmCalibrationDemo/PwmCalibrationDemo.ino itself against a fake Arduino
// (tests/fake_arduino/Arduino.h). Needs -DARDUINO_ARCH_AVR. Checks real pin writes, not just the library.
#ifndef ARDUINO_ARCH_AVR
#error "build with -DARDUINO_ARCH_AVR"
#endif
#include "Arduino.h"
unsigned long g_now=0; std::vector<PinEv> g_ev; int g_pwm[24]={0}; int g_dig[24]={0}; int g_mode[24]={0}; FakeSerial Serial;
#include "../examples/PwmCalibrationDemo/PwmCalibrationDemo.ino"
#include <cstdio>
#include <cstdlib>
#include <cstring>
static int fails=0;
#define CHECK(c,msg) do{ if(!(c)){ printf("  FAIL: %s\n",msg); fails++; } }while(0)
// motor "driven" = EN pwm>0 and exactly one direction pin high
static bool driven(int en,int a,int b){ return g_pwm[en]>0 && (g_dig[a]!=g_dig[b]); }
static unsigned long drivenMs[2];
static void step(unsigned long ms,unsigned long loopMs=1){
  for(unsigned long i=0;i<ms;i+=loopMs){ g_now+=loopMs; loop();
    if(driven(ENA,IN1,IN2)) { drivenMs[0]+=loopMs; }
    if(driven(ENB,IN3,IN4)) { drivenMs[1]+=loopMs; } }
}
static void key(char c){ Serial.in.push_back(c); }
static void reset(){ Serial.in.push_back('s'); { for(int i=0;i<3;i++){ g_now+=1; loop(); } } g_ev.clear(); memset(g_pwm,0,sizeof g_pwm); memset(g_dig,0,sizeof g_dig); drivenMs[0]=drivenMs[1]=0; Serial.in.clear(); Serial.out.clear(); }
static size_t countPwmWrites(){ size_t n=0; for(auto&e:g_ev) if(e.pwm) n++; return n; }
static bool outputsOff(){ return g_pwm[ENA]==0&&g_pwm[ENB]==0&&!g_dig[IN1]&&!g_dig[IN2]&&!g_dig[IN3]&&!g_dig[IN4]; }
static bool has(const char*s){ return Serial.out.find(s)!=std::string::npos; }
int main(){
  g_now=5000; setup();
  CHECK(outputsOff(),"boot: outputs must be off");
  puts("S1 normal pulse (left forward, 150 ms, loop 1 ms)");
  reset(); key('1'); step(5); size_t w0=countPwmWrites(); key('p'); step(1000);
  printf("  driven %lu ms, pwm writes during pulse+after=%zu\n",drivenMs[0],countPwmWrites()-w0);
  CHECK(drivenMs[0]>=148&&drivenMs[0]<=155,"pulse should drive ~150 ms"); CHECK(drivenMs[1]==0,"right wheel must stay off"); CHECK(outputsOff(),"off after pulse");
  CHECK(countPwmWrites()-w0<=6,"PWM must be written only on change (<=6 writes), not every loop");
  puts("S2 interrupt mid-pulse with '2' (switch wheel/direction)");
  reset(); key('1'); step(5); key('p'); step(50); key('2'); step(1); bool offNow=outputsOff(); step(2000);
  printf("  driven %lu ms; off right after key=%d; out-has-READY=%d\n",drivenMs[0],(int)offNow,(int)has("READY"));
  CHECK(offNow,"outputs off in the same ms as the key"); CHECK(drivenMs[0]<=56,"no resumption of remainder"); CHECK(outputsOff(),"stays off");
  puts("S3 'n' during pulse");
  reset(); key('1'); step(5); key('p'); step(40); key('n'); step(2000);
  CHECK(drivenMs[0]<=45,"no resume after n"); CHECK(outputsOff(),"off");
  puts("S4 'y' during pulse must NOT save a result");
  reset(); key('3'); step(5); key('p'); step(40); key('y'); step(2000);
  CHECK(!has("CONFIRMED"),"y during pulse must not confirm"); CHECK(drivenMs[1]<=45,"no resume after y"); CHECK(outputsOff(),"off");
  puts("S5 'p' pressed again during pulse must not extend or restart it");
  reset(); key('1'); step(5); key('p'); step(30); key('p'); step(2000);
  CHECK(drivenMs[0]>=148&&drivenMs[0]<=155,"pulse stays 150 ms");
  puts("S6 CR/LF/garbage during pulse must not disturb it");
  reset(); key('1'); step(5); key('p'); step(30); key('\r'); key('\n'); key('x'); step(2000);
  CHECK(drivenMs[0]>=148&&drivenMs[0]<=155,"CR/LF ignored, full pulse");
  puts("S7 's' stop mid-pulse");
  reset(); key('1'); step(5); key('p'); step(30); key('s'); step(100); key('p'); step(2000);
  CHECK(drivenMs[0]<=35,"stop works, 'p' after stop refused"); CHECK(has("STOP"),"STOP printed");
  puts("S8 idle timeout (no key for 16 s while READY)");
  reset(); key('2'); step(5); step(16000);
  CHECK(has("idle timeout"),"idle fault reported"); CHECK(outputsOff(),"off"); key('p'); step(500); CHECK(drivenMs[0]==0,"p after idle fault must not drive");
  puts("S9 operator ramp: true breakaway 60 PWM -> expect CONFIRMED PWM=60 + suggestion 51");
  reset(); key('1'); step(5); int guard=0;
  while(guard++<200){ key('p'); step(160); /* pulse */ step(520); /* cooldown */ // read candidate from the output
    size_t pos=Serial.out.rfind("candidate PWM="); int cand=pos==std::string::npos?0:atoi(Serial.out.c_str()+pos+14); const bool moved=cand>=60;
    if(moved){ key('y'); step(5); break; } key('n'); step(5); }
  printf("  %s", Serial.out.substr(Serial.out.rfind("CONFIRMED")).c_str());
  CHECK(has("CONFIRMED PWM=60"),"confirmed 60"); CHECK(has("=51"),"suggestion 51");
  puts("S10 other wheel stays off: right-forward pulse");
  reset(); key('3'); step(5); key('p'); step(400); CHECK(drivenMs[0]==0&&drivenMs[1]>=148,"only right wheel driven");
  puts("S11 slow main loop: loop period 50 ms -> overshoot bounded by one loop period");
  reset(); key('1'); step(50,50); key('p'); step(1000,50); printf("  driven %lu ms\n",drivenMs[0]); CHECK(drivenMs[0]<=150+50,"overshoot <= one loop period");
  printf("\n%s (%d failures)\n",fails?"FAILED":"ALL DEMO SCENARIOS PASSED",fails); return fails?1:0;
}
