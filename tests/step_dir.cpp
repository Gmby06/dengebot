#include "EDAStepDirController.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace EDA;
static int pulses(StepDirController& c,int ticks, int motor=0) {
  StepDirPins l={},r={}; int count=0; bool prev=false;
  for(int i=0;i<ticks;++i){c.tick(l,r);bool h=(motor==0?l:r).stepHigh;if(h&&!prev)++count;prev=h;}
  return count;
}
int main(){
  assert(!StepDirController::validConfig(StepDirConfig(20,4000,0)));
  assert(!StepDirController::validConfig(StepDirConfig(20,50000,150,40)));
  StepDirController c;
  assert(c.configValid() && !c.armed());
  assert(!c.setSpeeds(100,100));
  assert(c.arm());
  assert(!c.setSpeeds(NAN,100));
  assert(c.setSpeeds(5000,100)); // FIX3: clamped to max, not rejected
  assert(!c.setSpeeds(INFINITY,100));
  assert(c.setSpeeds(2000,-1000));
  // refresh command each 50ms to avoid the independent 150ms timeout
  StepDirPins l={},r={}; bool pl=false,pr=false;int nl=0,nr=0;
  for(int i=0;i<50000;++i) { // 1 second at 20us/tick
    if(i%2500==0)assert(c.setSpeeds(2000,-1000));
    c.tick(l,r);
    if(l.stepHigh&&!pl)++nl;
    if(r.stepHigh&&!pr)++nr;
    if(l.stepHigh)assert(l.direction==1);
    if(r.stepHigh)assert(r.direction==-1);
    pl=l.stepHigh;pr=r.stepHigh;
  }
  assert(std::abs(nl-2000)<=2 && std::abs(nr-1000)<=2);
  // reversal: STEP must be low on the tick that sets the new DIR
  assert(c.setSpeeds(-2000,0));
  bool seenDirection=false;
  for(int i=0;i<30;++i){c.tick(l,r);if(l.direction==-1){seenDirection=true;assert(!l.stepHigh);break;}}
  assert(seenDirection);
  // after timeout, both motors must be low and cannot re-arm themselves
  for(int i=0;i<10000;++i)c.tick(l,r);
  assert(c.timedOut() && !c.armed() && !l.stepHigh && !r.stepHigh);
  assert(!c.setSpeeds(100,100));
  assert(c.arm()); assert(!c.timedOut()); assert(c.setSpeeds(4000,4000));
  assert(pulses(c,2500)>190); // ~200 pulses in 50ms
  c.stop(); c.tick(l,r); assert(!l.stepHigh&&!r.stepHigh);
  std::puts("STEP/DIR scheduler host tests passed");
}
