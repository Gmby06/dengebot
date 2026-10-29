#include "EDADeadZoneCompensation.h"
#include <cassert>
#include <cmath>
using namespace EDA;
int main(){
 DeadZoneConfig cfg(48,170,0.5f);
 assert(validDeadZoneConfig(cfg,100));
 assert(toDCWithDeadZone(0,100,cfg).pwm==0);
 assert(toDCWithDeadZone(0.5f,100,cfg).pwm==0);
 assert(toDCWithDeadZone(-0.5f,100,cfg).direction==0);
 DCCommand c=toDCWithDeadZone(100,100,cfg);
 assert(c.direction==1&&c.pwm==170);
 c=toDCWithDeadZone(-100,100,cfg);
 assert(c.direction==-1&&c.pwm==170);
 c=toDCWithDeadZone(1,100,cfg);
 assert(c.direction==1&&c.pwm>=48);
 int previous=0;
 for(int i=0;i<=1000;++i){
  float v=i*0.1f;
  DCCommand a=toDCWithDeadZone(v,100,cfg);
  DCCommand b=toDCWithDeadZone(-v,100,cfg);
  assert(a.pwm>=previous);previous=a.pwm;
  assert(a.pwm==b.pwm&&a.direction==-b.direction);
  assert((a.direction==0)==(a.pwm==0));
 }
 assert(toDCWithDeadZone(NAN,100,cfg).pwm==0);
 assert(toDCWithDeadZone(INFINITY,100,cfg).pwm==0);
 assert(toDCWithDeadZone(50,0,cfg).pwm==0);
 assert(toDCWithDeadZone(50,100,DeadZoneConfig(171,170)).pwm==0);
 assert(toDCWithDeadZone(50,100,DeadZoneConfig(48,170,100)).pwm==0);
 assert(toDCWithDeadZone(50,100,DeadZoneConfig(48,170,NAN)).pwm==0);
 assert(toDCWithDeadZone(50,100,DeadZoneConfig(0,0)).pwm==0);
 // Default mapping with zero minimum matches legacy toDC behavior.
 for(int i=-100;i<=100;++i){
  DCCommand a=toDCWithDeadZone(static_cast<float>(i),100,DeadZoneConfig());
  DCCommand b=toDC(static_cast<float>(i),100);
  assert(a.pwm==b.pwm&&a.direction==b.direction);
 }
 return 0;
}
