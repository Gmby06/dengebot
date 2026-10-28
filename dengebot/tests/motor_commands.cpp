#include "EDAMotorCommand.h"
#include <cassert>
#include <cmath>
using namespace EDA;
int main(){
 const float INF=INFINITY, NANV=NAN;
 // temel
 DCCommand a=toDC(50,100);assert(a.direction==1 && a.pwm==128);
 a=toDC(-200,100);assert(a.direction==-1 && a.pwm==255);
 a=toDC(0,100);assert(a.direction==0 && a.pwm==0);
 StepperCommand s=toStepperVelocity(-50,100,2000);
 assert(s.direction==-1 && s.stepsPerSecond==1000);
 // maxPwm parametresi
 a=toDC(100,100,200);assert(a.direction==1 && a.pwm==200);
 a=toDC(100,100,0);assert(a.direction==0 && a.pwm==0);
 // gecersiz girisler => sifir komut (Inf dahil)
 a=toDC(NANV,100);assert(a.direction==0&&a.pwm==0);
 a=toDC(INF,100);assert(a.direction==0&&a.pwm==0);
 a=toDC(-INF,100);assert(a.direction==0&&a.pwm==0);
 a=toDC(10,0);assert(a.direction==0&&a.pwm==0);
 a=toDC(10,-5);assert(a.direction==0&&a.pwm==0);
 a=toDC(10,NANV);assert(a.direction==0&&a.pwm==0);
 a=toDC(10,INF);assert(a.direction==0&&a.pwm==0);
 // yon <=> pwm degismezi: cok kucuk komut yon vermez
 a=toDC(0.1f,100);assert(a.direction==0&&a.pwm==0);
 // simetri + monotonluk
 { int prev=-1; for(int i=0;i<=1000;i++){ float x=static_cast<float>(i)*0.1f;
     DCCommand p=toDC(x,100),n=toDC(-x,100);
     assert(p.pwm==n.pwm); assert(p.direction==-n.direction); assert(p.pwm>=prev); prev=p.pwm;
     assert((p.direction==0)==(p.pwm==0)); } }
 // buyuk sonlu degerler tasmaz
 a=toDC(3.0e38f,100);assert(a.direction==1&&a.pwm==255);
 // stepper
 s=toStepperVelocity(NANV,100,2000);assert(s.direction==0&&s.stepsPerSecond==0);
 s=toStepperVelocity(INF,100,2000);assert(s.direction==0&&s.stepsPerSecond==0);
 s=toStepperVelocity(50,100,INF);assert(s.direction==0&&s.stepsPerSecond==0);
 s=toStepperVelocity(50,100,NANV);assert(s.direction==0&&s.stepsPerSecond==0);
 s=toStepperVelocity(50,100,0);assert(s.direction==0&&s.stepsPerSecond==0);
 s=toStepperVelocity(50,INF,2000);assert(s.direction==0&&s.stepsPerSecond==0);
 s=toStepperVelocity(50,0,2000);assert(s.direction==0&&s.stepsPerSecond==0);
 s=toStepperVelocity(500,100,2000);assert(s.direction==1&&s.stepsPerSecond==2000);
 s=toStepperVelocity(3.0e38f,100,2000);assert(s.direction==1&&s.stepsPerSecond==2000);
 // yon sadece saniyede adim > 0 ise
 s=toStepperVelocity(1.0e-30f,1.0e30f,1.0f);assert(s.direction==0&&s.stepsPerSecond==0);
 return 0;
}
