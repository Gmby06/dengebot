#include "EDAPwmCalibration.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
using namespace EDA;

// Deterministic xorshift so the fuzz part needs no <random> and is reproducible.
static uint32_t rngState=2463534242u;
static uint32_t rnd() { rngState^=rngState<<13; rngState^=rngState>>17; rngState^=rngState<<5; return rngState; }

static void basicFlow() {
  // start 40, max 46, step 2, pulse 100, cooldown 200, idle 3000, session 60000
  PwmCalibration c(PwmCalibrationConfig(40,46,2,100,200,3000,60000));
  assert(c.state()==CalibrationState::Idle);
  assert(c.command(0).pwm==0);
  assert(!c.confirmMovement(0));
  assert(c.begin(CalibrationWheel::Left,CalibrationDirection::Forward,100));
  assert(!c.confirmMovement(100));                     // nothing pulsed yet
  assert(c.requestPulse(100));
  assert(c.command(100).pwm==40 && c.command(100).direction==1);
  assert(!c.requestPulse(110));                        // already pulsing
  assert(!c.confirmMovement(110));                     // not during the pulse
  c.tick(200); assert(c.command(200).pwm==0);          // pulse over
  assert(!c.advance(201));                             // cooling down
  c.tick(400); assert(c.advance(400)); assert(c.candidatePwm()==42);
  assert(c.requestPulse(401)); c.tick(501);
  assert(c.confirmMovement(502));
  assert(c.results().has(CalibrationWheel::Left,CalibrationDirection::Forward));
  assert(c.results().get(CalibrationWheel::Left,CalibrationDirection::Forward)==42);
  assert(c.command(502).pwm==0);
  // limit reached
  assert(c.begin(CalibrationWheel::Right,CalibrationDirection::Reverse,1000));
  assert(c.requestPulse(1000)); assert(c.command(1000).direction==-1);
  c.tick(1100); c.tick(1300); assert(c.advance(1300));
  assert(c.requestPulse(1301)); c.tick(1401); c.tick(1601); assert(c.advance(1601));
  assert(c.requestPulse(1602)); c.tick(1702); c.tick(1902); assert(c.advance(1902));
  assert(c.candidatePwm()==46); assert(c.requestPulse(1903)); c.tick(2003); c.tick(2203);
  assert(!c.advance(2203)); assert(c.fault()==CalibrationFault::LimitReached);
  assert(c.command(2203).pwm==0);
  c.stop(); assert(c.state()==CalibrationState::Idle && c.fault()==CalibrationFault::None);
  PwmCalibration bad(PwmCalibrationConfig(0,10,1,50,100,500,1000));
  assert(bad.state()==CalibrationState::Fault && bad.fault()==CalibrationFault::InvalidConfig);
  assert(!bad.requestPulse(0));
  assert(!bad.begin(CalibrationWheel::Left,CalibrationDirection::Forward,0));
  PwmCalibration bad2(PwmCalibrationConfig(30,60,2,100,200,0,1000));   // idle 0
  assert(bad2.state()==CalibrationState::Fault);
}

static void idleAndSessionTimeouts() {
  PwmCalibration c(PwmCalibrationConfig(30,200,2,100,200,3000,20000));
  assert(c.begin(CalibrationWheel::Left,CalibrationDirection::Forward,0));
  c.tick(2999); assert(c.state()==CalibrationState::Ready);
  c.tick(3000); assert(c.state()==CalibrationState::Fault && c.fault()==CalibrationFault::IdleTimeout);
  assert(!c.requestPulse(3001) && c.command(3001).pwm==0);
  // Pulse + cooldown time does not count as idle; operator acting regularly keeps the session alive
  // far beyond the idle time but not beyond the hard session cap.
  assert(c.begin(CalibrationWheel::Left,CalibrationDirection::Forward,10000));
  uint32_t t=10000; int steps=0;
  while (c.state()!=CalibrationState::Fault && steps<1000) {
    t+=1500; if (!c.requestPulse(t)) break;            // operator presses p after 1.5 s (refused once the session cap hits)
    t+=100; c.tick(t);                                 // pulse ends
    t+=200; c.tick(t);                                 // cooldown ends
    t+=1500; if (!c.advance(t)) break;                 // presses n after 1.5 s
    steps++;
  }
  assert(c.state()==CalibrationState::Fault && c.fault()==CalibrationFault::Timeout);   // session cap (20 s)
  assert(t-10000 >= 20000-3500);
  assert(steps>=4);                                    // idle time (3 s) was NOT the limit here
  // Late tick: cooldown end is scheduled from the pulse, not from the late tick.
  PwmCalibration d(PwmCalibrationConfig(30,100,2,100,200,3000,60000));
  d.begin(CalibrationWheel::Left,CalibrationDirection::Forward,0); assert(d.requestPulse(1000));
  d.tick(1000+100+200+2900); assert(d.state()==CalibrationState::Ready);      // 2.9 s into idle
  d.tick(1000+100+200+3000); assert(d.state()==CalibrationState::Fault && d.fault()==CalibrationFault::IdleTimeout);
}

static void commandChecksClockItself() {
  PwmCalibration c(PwmCalibrationConfig(40,100,2,120,500,15000,300000));
  c.begin(CalibrationWheel::Left,CalibrationDirection::Forward,0);
  assert(c.requestPulse(1000));
  assert(c.command(1000).pwm==40 && c.command(1119).pwm==40);
  assert(c.command(1120).pwm==0);                      // expired even though tick() was never called
  assert(c.command(10000).pwm==0);
  assert(c.command(500).pwm==0);                       // clock earlier than pulse start: refuse
  c.tick(10000); assert(c.state()==CalibrationState::Ready);   // late tick catches up both transitions
}

static void abortAndStopAreDistinct() {
  PwmCalibration c;
  c.begin(CalibrationWheel::Left,CalibrationDirection::Forward,0); c.requestPulse(0);
  c.abort(); assert(c.state()==CalibrationState::Fault && c.fault()==CalibrationFault::Aborted);
  assert(c.command(1).pwm==0 && !c.requestPulse(2));
  c.begin(CalibrationWheel::Left,CalibrationDirection::Forward,100); c.requestPulse(100);
  c.stop(); assert(c.state()==CalibrationState::Idle && c.command(101).pwm==0);
}

static void suggestion() {
  PwmCalibrationResults r;
  assert(suggestDeadZoneMinPwm(r,CalibrationWheel::Left)==0);                   // nothing confirmed
  r.pwm[0][1]=60; r.confirmed[0][1]=true;                                       // left forward only
  assert(suggestDeadZoneMinPwm(r,CalibrationWheel::Left)==51);                  // 60 * 85%
  assert(suggestDeadZoneMinPwm(r,CalibrationWheel::Right)==0);
  r.pwm[0][0]=52; r.confirmed[0][0]=true;                                       // reverse lower
  assert(suggestDeadZoneMinPwm(r,CalibrationWheel::Left)==44);                  // min(60,52)*85% = 44.2
  assert(suggestDeadZoneMinPwm(r,CalibrationWheel::Left,0)==52);
  assert(suggestDeadZoneMinPwm(r,CalibrationWheel::Left,200)==26);              // margin clamped to 50%
  PwmCalibrationResults one; one.pwm[1][0]=1; one.confirmed[1][0]=true;
  assert(suggestDeadZoneMinPwm(one,CalibrationWheel::Right)==1);                // never below 1
}

static void clockWrap() {
  const uint32_t t0=0xFFFFFFFFu-300u;
  PwmCalibration c(PwmCalibrationConfig(40,100,2,120,500,15000,300000));
  assert(c.begin(CalibrationWheel::Right,CalibrationDirection::Reverse,t0));
  assert(c.requestPulse(t0+50));
  assert(c.command(t0+50+119).pwm==40);                // crosses the 2^32 boundary
  assert(c.command(t0+50+120).pwm==0);
  c.tick(t0+50+120); assert(c.state()==CalibrationState::Cooldown);
  c.tick(t0+50+620); assert(c.state()==CalibrationState::Ready);
  assert(c.advance(t0+50+700)); assert(c.candidatePwm()==42);
}

// Random operation sequences with random (also huge) time jumps; checks safety invariants.
static void fuzzInvariants() {
  long checked=0;
  for (int run=0; run<3000; run++) {
    const uint8_t st=1+rnd()%100; uint8_t mx=st+rnd()%120; const uint8_t step=1+rnd()%10;
    const uint16_t pulse=1+rnd()%300, cool=1+rnd()%600;
    const uint32_t idle=1+rnd()%4000, sess=static_cast<uint32_t>(pulse)+cool+1+rnd()%20000;
    PwmCalibration c(PwmCalibrationConfig(st,mx,step,pulse,cool,idle,sess));
    uint32_t t=rnd(); uint32_t pulseStart=0; bool havePulse=false;
    for (int i=0;i<300;i++) {
      t+=(rnd()%5==0)?rnd()%3000:rnd()%40;
      switch (rnd()%8) {
        case 0: if (c.begin(static_cast<CalibrationWheel>(rnd()%2),(rnd()&1)?CalibrationDirection::Forward:CalibrationDirection::Reverse,t)) havePulse=false; break;
        case 1: if (c.requestPulse(t)) { pulseStart=t; havePulse=true; } break;
        case 2: c.advance(t); break;
        case 3: c.confirmMovement(t); break;
        case 4: c.stop(); break;
        case 5: c.abort(); break;
        default: c.tick(t);
      }
      // a few reads at times after the last event; command() must self-limit
      for (int k=0;k<3;k++) {
        const uint32_t now=t+(k==0?0:rnd()%2000);
        const DCCommand d=c.command(now);
        checked++;
        if (d.pwm!=0) {
          assert(c.state()==CalibrationState::Pulsing);
          assert(d.pwm>=st && d.pwm<=mx);
          assert(d.direction==static_cast<int8_t>(c.direction()));
          assert(havePulse && static_cast<uint32_t>(now-pulseStart)<pulse);   // never longer than pulseMs
        } else assert(d.direction==0);
      }
      assert(c.candidatePwm()>=st && c.candidatePwm()<=mx);
      const PwmCalibrationResults& r=c.results();
      for (int w=0;w<2;w++) for (int dd=0;dd<2;dd++) {
        const CalibrationWheel cw=static_cast<CalibrationWheel>(w);
        const CalibrationDirection cd=dd?CalibrationDirection::Forward:CalibrationDirection::Reverse;
        if (r.has(cw,cd)) { const uint8_t v=r.get(cw,cd); assert(v>=st && v<=mx); }
        else assert(r.get(cw,cd)==0);
      }
      if (c.state()==CalibrationState::Fault || c.state()==CalibrationState::Idle ||
          c.state()==CalibrationState::Complete) assert(c.command(t).pwm==0);
    }
  }
  assert(checked>1000000);
}

// v1.7.5: begin() invalidates the old measurement of the selected slot only.
static void rerunInvalidatesOldResult() {
  PwmCalibration c(PwmCalibrationConfig(30,100,2,100,200,3000,60000));
  const CalibrationWheel L=CalibrationWheel::Left, R=CalibrationWheel::Right;
  const CalibrationDirection F=CalibrationDirection::Forward, B=CalibrationDirection::Reverse;
  // confirm Left/Forward at 30 and Right/Reverse at 30
  assert(c.begin(L,F,0)); assert(c.requestPulse(0)); c.tick(100); assert(c.confirmMovement(100));
  assert(c.begin(R,B,1000)); assert(c.requestPulse(1000)); c.tick(1100); assert(c.confirmMovement(1100));
  assert(c.results().has(L,F) && c.results().get(L,F)==30);
  assert(c.results().has(R,B) && c.results().get(R,B)==30);
  // new attempt on Left/Forward, then abort: the old value must be gone
  assert(c.begin(L,F,2000));
  assert(!c.results().has(L,F) && c.results().get(L,F)==0);
  assert(c.requestPulse(2000)); c.abort();
  assert(!c.results().has(L,F) && c.results().get(L,F)==0);
  assert(suggestDeadZoneMinPwm(c.results(),L)==0);
  // other slots untouched
  assert(c.results().has(R,B) && c.results().get(R,B)==30);
  // a REFUSED begin() (bad args / busy) must not erase anything
  assert(!c.begin(static_cast<CalibrationWheel>(2),F,3000));
  assert(!c.begin(R,static_cast<CalibrationDirection>(0),3000));
  assert(c.results().has(R,B));
  assert(c.begin(R,B,4000)); assert(c.requestPulse(4000));
  assert(!c.begin(R,B,4010));                           // pulsing: refused
  // timeout of a re-run also leaves the slot unconfirmed
  PwmCalibration t(PwmCalibrationConfig(30,100,2,100,200,3000,60000));
  assert(t.begin(L,F,0)); assert(t.requestPulse(0)); t.tick(100); assert(t.confirmMovement(100));
  assert(t.begin(L,F,500)); t.tick(500+3000); assert(t.fault()==CalibrationFault::IdleTimeout);
  assert(!t.results().has(L,F));
}

// v1.7.5: timeouts above 0x7fffffff are rejected; the boundary itself is accepted.
static void timeoutUpperBound() {
  assert( PwmCalibration::valid(PwmCalibrationConfig(30,100,2,100,200,0x7fffffffu,0x7fffffffu)));
  assert(!PwmCalibration::valid(PwmCalibrationConfig(30,100,2,100,200,0x80000000u,60000)));
  assert(!PwmCalibration::valid(PwmCalibrationConfig(30,100,2,100,200,3000,0x80000000u)));
  assert(!PwmCalibration::valid(PwmCalibrationConfig(30,100,2,100,200,0xFFFFFFFFu,0xFFFFFFFFu)));
  PwmCalibration bad(PwmCalibrationConfig(30,100,2,100,200,3000,0x80000000u));
  assert(bad.state()==CalibrationState::Fault && bad.fault()==CalibrationFault::InvalidConfig);
  // accepted maximum: a normal pulse still works and nothing times out spuriously
  PwmCalibration ok(PwmCalibrationConfig(30,100,2,100,200,0x7fffffffu,0x7fffffffu));
  assert(ok.begin(CalibrationWheel::Left,CalibrationDirection::Forward,5));
  assert(ok.requestPulse(6)); ok.tick(106); ok.tick(306);
  assert(ok.state()==CalibrationState::Ready);
}

// v1.7.5: invalid enum values (bad casts) never index out of bounds.
static void resultsRejectInvalidEnums() {
  PwmCalibrationResults r;
  r.pwm[0][0]=11; r.confirmed[0][0]=true; r.pwm[1][1]=22; r.confirmed[1][1]=true;
  const CalibrationWheel badW[3]={static_cast<CalibrationWheel>(2),static_cast<CalibrationWheel>(7),static_cast<CalibrationWheel>(255)};
  const CalibrationDirection badD[3]={static_cast<CalibrationDirection>(0),static_cast<CalibrationDirection>(2),static_cast<CalibrationDirection>(-5)};
  for (int i=0;i<3;i++) {
    assert(!r.has(badW[i],CalibrationDirection::Forward) && r.get(badW[i],CalibrationDirection::Forward)==0);
    assert(!r.has(CalibrationWheel::Left,badD[i]) && r.get(CalibrationWheel::Left,badD[i])==0);
    assert(!r.has(badW[i],badD[i]));
    r.clear(badW[i],CalibrationDirection::Forward); r.clear(CalibrationWheel::Left,badD[i]);   // no-ops
  }
  assert(r.has(CalibrationWheel::Left,CalibrationDirection::Reverse) && r.get(CalibrationWheel::Left,CalibrationDirection::Reverse)==11);
  assert(r.has(CalibrationWheel::Right,CalibrationDirection::Forward) && r.get(CalibrationWheel::Right,CalibrationDirection::Forward)==22);
  assert(suggestDeadZoneMinPwm(r,static_cast<CalibrationWheel>(9))==0);
}

int main() {
  basicFlow();
  rerunInvalidatesOldResult();
  timeoutUpperBound();
  resultsRejectInvalidEnums();
  idleAndSessionTimeouts();
  commandChecksClockItself();
  abortAndStopAreDistinct();
  suggestion();
  clockWrap();
  fuzzInvariants();
  puts("pwm_calibration: OK");
  return 0;
}
