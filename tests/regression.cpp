#include "EDADengeBot.h"
#include <cassert>
#include <cmath>
int main(){
 EDAPIDConfig c=edaDefaultConfig(); EDABalancePID p(c);
 assert(p.configValid() && !p.faulted());
 assert(p.update(0,2,0,0,.01f)<0);
 p.update(0,2,NAN,0,.01f);assert(p.faulted() && p.fault()==EDAFault::InvalidSensor);
 p.update(0,2,0,0,.01f);assert(p.lastUpdateValid() && p.faulted());
 p.reset();assert(p.faulted());p.clearFault();assert(!p.faulted());
 p.update(0,0,0,0,.001f);assert(p.fault()==EDAFault::InvalidTimeStep);
 p.clearFault();
 EDAPIDConfig bad=c;bad.outputLimit=-1;EDABalancePID invalid(bad);
 assert(!invalid.configValid() && invalid.faulted() && !invalid.lastUpdateValid());
 assert(invalid.update(0,0,0,0,.01f)==0);
 invalid.clearFault();assert(invalid.faulted());
 assert(invalid.setConfig(c));invalid.clearFault();assert(!invalid.faulted());
 bad=c;bad.pushMaxExtra=-1;assert(!p.setConfig(bad));
 bad=c;bad.floorMinOutput=-1;assert(!p.setConfig(bad));
 c.kp=0;c.kd=0;c.ki=100;c.iLimit=100;c.iEnableAngle=10;
 EDABalancePID q(c);for(int i=0;i<100;i++)q.update(1,0,0,0,.01f);
 float before=q.iTerm();assert(before>0);
 assert(q.setTunings(0,200,0,false));assert(std::fabs(q.iTerm()-before)<0.001f);
 assert(q.setTunings(0,0,0,false));assert(q.iTerm()==0);
 c.kp=500;c.ki=0;EDABalancePID sat(c);float u=sat.update(10,0,0,0,.01f);
 assert(std::fabs(u-c.outputLimit)<0.001f && sat.saturatedSamples()>0);
 // V1.5.2: ozellige bagli dogrulama
 { EDAPIDConfig d=edaDefaultConfig(); d.outputLimit=100; EDABalancePID lo(d); assert(lo.configValid()&&!lo.faulted()); }
 { EDAPIDConfig d=edaDefaultConfig(); d.outputLimit=100; d.enableFloor=true; EDABalancePID lo(d); assert(!lo.configValid()); }
 { EDAPIDConfig d=edaDefaultConfig(); d.centerExitRate=1; d.enableCenterFilter=false; EDABalancePID z(d); assert(z.configValid());
   d.enableCenterFilter=true; EDABalancePID z2(d); assert(!z2.configValid()); }
 { EDAPIDConfig d=edaDefaultConfig(); d.kp=-1; EDABalancePID z(d); assert(!z.configValid()); }
 // V1.5.2: ilk hata saklanir
 { EDABalancePID f(edaDefaultConfig()); f.update(0,0,NAN,0,.01f); f.update(0,0,0,0,.001f);
   assert(f.firstFault()==EDAFault::InvalidSensor && f.fault()==EDAFault::InvalidTimeStep);
   f.clearFault(); assert(f.firstFault()==EDAFault::None && !f.faulted()); }
 // V1.5.2: istege bagli sifir cikis
 { EDABalancePID z(edaDefaultConfig()); z.setZeroOutputOnFault(true);
   assert(z.update(0,2,0,0,.01f)!=0); z.update(0,2,NAN,0,.01f);
   assert(z.update(0,2,0,0,.01f)==0 && !z.lastUpdateValid());
   z.clearFault(); assert(z.update(0,2,0,0,.01f)!=0 && z.lastUpdateValid()); }
 { EDABalancePID d0(edaDefaultConfig()); d0.update(0,2,NAN,0,.01f); assert(d0.update(0,2,0,0,.01f)!=0); } // varsayilan: V1.5.1 davranisi
 return 0;
}
