#include "dengebot.h"
#include <cassert>
#include <cmath>
int main() {
  EDABalancePID pid(edaDefaultConfig());
  EDASafetyConfig cfg={30.0f,50}; EDASafetyGuard g(cfg);
  assert(!g.armed() && g.update(pid,100,100,true,0,2,0,0,.01f)==0);
  assert(g.start(100,100,2,true,pid));
  assert(g.update(pid,110,110,true,0,2,0,0,.01f)!=0);
  g.stop(); assert(!g.armed() && g.update(pid,120,120,true,0,2,0,0,.01f)==0);
  assert(g.start(130,130,2,true,pid));
  assert(g.update(pid,200,130,true,0,2,0,0,.01f)==0);
  assert(g.fault()==EDASafetyFault::StaleSensor && !g.armed());
  assert(!g.start(210,210,0,true,pid));
  assert(g.clearFault()); assert(g.start(210,210,0,true,pid));
  assert(g.update(pid,220,220,true,0,31,0,0,.01f)==0);
  assert(g.fault()==EDASafetyFault::Fallen);
  assert(g.clearFault()); assert(g.start(230,230,0,true,pid));
  assert(g.update(pid,240,240,false,0,0,0,0,.01f)==0);
  assert(g.fault()==EDASafetyFault::InvalidSensor);
  assert(g.clearFault()); assert(g.start(250,250,0,true,pid));
  assert(g.update(pid,260,260,true,0,0,NAN,0,.01f)==0);
  assert(g.fault()==EDASafetyFault::ControllerFault);
  assert(!g.start(270,270,0,true,pid));
  pid.clearFault(); assert(g.clearFault()); assert(g.start(280,280,0,true,pid));
  assert(g.update(pid,290,290,true,0,1,0,0,.001f)==0);
  assert(g.fault()==EDASafetyFault::ControllerFault);
  pid.clearFault(); assert(g.clearFault());
  assert(g.start(0xfffffff0u,0xfffffff0u,0,true,pid));
  assert(g.update(pid,0x10u,0xfffffff0u,true,0,1,0,0,.01f)!=0); // millis wrap
  g.stop(); assert(!g.start(100,101,0,true,pid)); // future sample
  // Single explicit START recovers from a latched fall without power cycle.
  EDASafetyGuard recovery(cfg);
  assert(recovery.requestStart(100,100,0,true,pid));
  assert(recovery.update(pid,110,110,true,0,35,0,0,.01f)==0);
  assert(recovery.fault()==EDASafetyFault::Fallen);
  assert(!recovery.requestStart(120,120,35,true,pid)); // still fallen
  assert(recovery.fault()==EDASafetyFault::Fallen);
  assert(!recovery.requestStart(130,0,0,true,pid)); // stale
  assert(recovery.fault()==EDASafetyFault::Fallen);
  assert(!recovery.requestStart(140,140,0,false,pid)); // invalid
  assert(recovery.fault()==EDASafetyFault::Fallen);
  assert(recovery.requestStart(150,150,0,true,pid)); // one button press
  assert(recovery.armed() && recovery.fault()==EDASafetyFault::None);
  assert(!recovery.requestStart(160,160,0,true,pid)); // held START not toggle
  recovery.stop(); assert(recovery.requestStart(170,170,0,true,pid));
  recovery.trip(EDASafetyFault::StaleSensor);
  assert(recovery.requestStart(180,180,0,true,pid)); // valid sample permits rearm
  assert(recovery.armed());
  recovery.trip(EDASafetyFault::ControllerFault);
  pid.update(0,0,NAN,0,.01f);
  assert(pid.faulted());
  assert(!recovery.requestStart(190,190,0,true,pid));
  assert(recovery.fault()==EDASafetyFault::ControllerFault);
  pid.clearFault();
  assert(recovery.requestStart(200,200,0,true,pid));
  EDASafetyConfig bad={0,50}; EDASafetyGuard invalid(bad);
  assert(!invalid.start(0,0,0,true,pid));
  return 0;
}
