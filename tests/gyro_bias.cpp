#include "EDAGyroBiasCalibrator.h"
#include "EDAComplementaryFilter.h"
#include <cassert>
#include <cmath>
#include <limits>
int main(){
 EDAGyroBiasCalibrator c(100);
 assert(!c.ready() && !c.stationary() && c.samples()==0);
 for(int i=0;i<99;i++) c.addSample(0.5f+((i&1)?0.1f:-0.1f));
 assert(!c.ready());
 c.addSample(0.5f); assert(c.ready() && c.stationary());
 assert(std::fabs(c.bias()-0.5f)<0.02f && c.stdDev()>0.05f && c.stdDev()<0.2f);
 // hareket (yuksek std) -> stationary degil
 EDAGyroBiasCalibrator m(100); for(int i=0;i<100;i++) m.addSample((i&1)?20.0f:-20.0f);
 assert(m.ready() && !m.stationary());
 // makul olmayan buyuk bias -> stationary degil
 EDAGyroBiasCalibrator b(100); for(int i=0;i<100;i++) b.addSample(9.0f);
 assert(b.ready() && !b.stationary());
 // NaN ornegi calismayi bozar, reset ile duzelir
 EDAGyroBiasCalibrator n(100); for(int i=0;i<50;i++) n.addSample(0.1f);
 assert(!n.addSample(std::numeric_limits<float>::quiet_NaN()) && n.failed());
 for(int i=0;i<200;i++) { n.addSample(0.1f); }
 assert(!n.ready() && !n.stationary());
 n.reset(); assert(!n.failed() && n.samples()==0);
 // gecersiz yapilandirma
 assert(!c.configure(5,1,5) && !c.configure(100,0,5) && !c.configure(100,1,std::numeric_limits<float>::infinity()) && c.configure(50,2,3));
 // filtreye uygulama
 EDAComplementaryFilter f(0.98f); assert(f.setGyroBias(c.bias()));
 return 0;
}
