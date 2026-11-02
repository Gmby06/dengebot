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
    // v1.7.4: STOP -> START must not carry stale integral/filter state.
  {
    EDAPIDConfig c=edaDefaultConfig(); c.ki=3.0f;
    EDABalancePID p(c); EDASafetyGuard s({25.0f,50});
    uint32_t t=1000; assert(s.start(t,t,0,true,p));
    for(int i=0;i<200;i++){t+=10; s.update(p,t,t,true,0,2.0f,0,0,0.01f);}
    assert(fabsf(p.iTerm())>1.0f);                 // integral really accumulated
    s.stop(); t+=10; assert(s.start(t,t,0,true,p)); t+=10;
    s.update(p,t,t,true,0,0.0f,0,0,0.01f);
    assert(fabsf(p.iTerm())<1e-4f);                // reset on first update after arming
    // Recovery from a latched FAULT also starts clean.
    for(int i=0;i<200;i++){t+=10; s.update(p,t,t,true,0,2.0f,0,0,0.01f);}
    assert(fabsf(p.iTerm())>1.0f);
    s.update(p,t+500,t,true,0,2.0f,0,0,0.01f);      // stale sample -> FAULT
    assert(s.fault()==EDASafetyFault::StaleSensor);
    t+=600; assert(s.requestStart(t,t,0,true,p)); t+=10;
    s.update(p,t,t,true,0,0.0f,0,0,0.01f);
    assert(fabsf(p.iTerm())<1e-4f);
  }
  // v1.7.4: optional fall limit relative to the balance target.
  {
    EDABalancePID p(edaDefaultConfig()); EDASafetyGuard s({25.0f,50});
    assert(s.maxAbsErrorDeg()==0.0f);
    assert(!s.setMaxAbsErrorDeg(-1.0f) && !s.setMaxAbsErrorDeg(NAN) && !s.setMaxAbsErrorDeg(INFINITY));
    assert(s.maxAbsErrorDeg()==0.0f);
    assert(s.start(100,100,-3.85f,true,p));
    s.update(p,110,110,true,-3.85f,22.0f,0,0,0.01f); // 25.85 deg from target, default: not tripped
    assert(s.armed());
    s.stop(); assert(s.setMaxAbsErrorDeg(25.0f) && s.maxAbsErrorDeg()==25.0f);
    assert(!s.start(120,120,-3.85f,true,p)); // fail closed: legacy cannot check target
    assert(s.start(120,120,-3.85f,-3.85f,true,p));
    assert(s.update(p,130,130,true,-3.85f,22.0f,0,0,0.01f)==0);
    assert(s.fault()==EDASafetyFault::Fallen && !s.armed());
    assert(s.clearFault() && s.start(140,140,-3.85f,-3.85f,true,p));
    assert(s.update(p,150,150,true,-3.85f,-26.0f,0,0,0.01f)==0); // absolute limit still trips
    assert(s.fault()==EDASafetyFault::Fallen);
    assert(s.clearFault() && s.start(160,160,-3.85f,-3.85f,true,p));
    assert(s.update(p,170,170,true,NAN,0.0f,0,0,0.01f)==0);        // non-finite target
    assert(s.fault()==EDASafetyFault::ControllerFault);
  }
  // Target-aware arming: reject unsafe request without clearing an existing fault.
  {
    EDAPIDConfig c = edaDefaultConfig(); c.ki = 3.0f;
    EDABalancePID p(c); EDASafetyGuard s({30.0f,50});
    assert(s.setMaxAbsErrorDeg(10.0f));
    assert(!s.start(100,100,0.0f,true,p)); // legacy API rejects if target limit enabled
    assert(!s.requestStart(100,100,0.0f,true,p));
    assert(!s.armed() && s.fault()==EDASafetyFault::Stopped);
    assert(!s.requestStart(100,100,9.0f,-3.0f,true,p)); // target error 12 deg
    assert(!s.armed() && s.fault()==EDASafetyFault::Stopped);
    assert(!s.start(100,100,0.0f,NAN,true,p));
    assert(s.start(100,100,0.0f,0.0f,true,p));
    uint32_t t=100;
    for (int i=0;i<150;i++) {t+=10; s.update(p,t,t,true,0,2,0,0,.01f);}
    assert(fabsf(p.iTerm())>1.0f);
    s.update(p,t+100,t,true,0,0,0,0,.01f);
    assert(s.fault()==EDASafetyFault::StaleSensor);
    assert(!s.requestStart(t+110,t+110,9,-3,true,p));
    assert(!s.requestStart(t+110,t+110,0,true,p)); // legacy cannot bypass target safety
    assert(s.fault()==EDASafetyFault::StaleSensor);
    assert(s.requestStart(t+120,t+120,0,0,true,p));
    assert(!s.faulted() && s.armed());
    assert(fabsf(p.iTerm())<1e-4f); // reset IMMEDIATE on successful targeted START
    assert(s.update(p,t+130,t+130,true,0,0,0,0,.01f)==0);
    assert(!s.requestStart(t+140,t+140,0,0,true,p)); // already armed
  }
  // v1.7.5: invalid configuration is reported as InvalidConfig (not InvalidSensor)
  { EDASafetyConfig badCfg[4]={{0.0f,50},{30.0f,0},{NAN,50},{30.0f,0x80000000u}};
    for (int i=0;i<4;i++) {
      EDASafetyGuard b(badCfg[i]); EDABalancePID bp(edaDefaultConfig());
      assert(!EDASafetyGuard::validConfig(badCfg[i]));
      assert(b.fault()==EDASafetyFault::InvalidConfig && b.faulted() && !b.armed());
      assert(!b.clearFault() && !b.start(100,100,0,true,bp) && !b.requestStart(100,100,0,true,bp));
      assert(b.update(bp,100,100,true,0,0,0,0,.01f)==0);
    } }
  // maxSampleAgeMs boundary: 0x7fffffff is the largest accepted value
  { EDASafetyConfig edge={30.0f,0x7fffffffu}; assert(EDASafetyGuard::validConfig(edge));
    EDASafetyGuard e(edge); assert(e.fault()==EDASafetyFault::Stopped); }
  return 0;
}
