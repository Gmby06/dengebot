#include "EDAComplementaryFilter.h"
#include "EDATelemetry.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
struct MockPrint {
 std::ostringstream s;
 void print(const char *x) { s << x; }
 void print(char x) { s << x; }
 void print(float x, int precision) { s.setf(std::ios::fixed); s.precision(precision); s << x; }
 void print(unsigned int x) { s << x; }
 void println(unsigned int x) { s << x << '\n'; }
 void println(float x, int precision) { print(x, precision); s << '\n'; }
};
int main() {
 EDAComplementaryFilter f;
 assert(f.alpha()==0.98f && !f.initialized());
 assert(!f.setAlpha(-0.1f) && !f.setAlpha(1.1f));
 assert(!f.setAlpha(std::numeric_limits<float>::quiet_NaN()));
 assert(f.setAlpha(0.5f));
 assert(f.update(10.0f, 2.0f, 0.01f));
 assert(std::fabs(f.pitchDegrees()-10.0f)<0.0001f);
 assert(f.update(8.0f, 2.0f, 0.1f));
 assert(std::fabs(f.pitchDegrees()-9.1f)<0.0001f);
 assert(!f.update(0.0f, 0.0f, 0.0f));
 assert(!f.valid() && std::fabs(f.pitchDegrees()-9.1f)<0.0001f);
 assert(!f.update(std::numeric_limits<float>::infinity(),0.0f,0.01f));
 assert(f.update(8.0f, 0.0f, 0.01f) && f.valid());
 f.reset(); assert(!f.initialized());
 EDAComplementaryFilter adaptive;
 assert(adaptive.configureAdaptive(0.998f,0.15f,0.10f));
 adaptive.setAdaptiveEnabled(true);
 assert(adaptive.setGyroBias(1.0f));
 assert(adaptive.update(0,0,0.01f,1.0f));
 assert(adaptive.update(0,1,0.01f,1.0f)); // bias corrected
 assert(std::fabs(adaptive.pitchDegrees())<1e-5f);
 assert(adaptive.update(0,1,0.01f,1.2f));
 assert(adaptive.highAcceleration() && std::fabs(adaptive.usedAlpha()-0.998f)<1e-5f);
 assert(adaptive.update(0,1,0.01f,1.12f) && adaptive.highAcceleration());
 assert(adaptive.update(0,1,0.01f,1.05f) && !adaptive.highAcceleration());
 assert(!adaptive.update(0,1,0,1.2f) && !adaptive.valid());
 assert(!adaptive.update(0,1,0.01f,std::numeric_limits<float>::quiet_NaN()));
 assert(!adaptive.setGyroBias(std::numeric_limits<float>::infinity()));
 assert(adaptive.update(0,1,0.01f) && std::fabs(adaptive.usedAlpha()-0.98f)<1e-5f);

 EDATelemetryFrame frame={123,1,2,3,4,5,10000,1,0};
 assert(EDATelemetry::valid(frame));
 MockPrint p;
 assert(EDATelemetry::printPlotter(p,frame));
 assert(p.s.str().find("Target:1.000\tPitch:2.000")!=std::string::npos);
 MockPrint csv;
 assert(EDATelemetry::printCSV(csv,frame));
 assert(csv.s.str()=="123,1.000,2.000,3.000,4.000,5.000,10000,1,0\n");
 MockPrint extended;
 assert(EDATelemetry::printCSVExtended(extended,frame,2.5f,1,120,0.998f));
 assert(extended.s.str().find("1,0,2.500,1,120,0.998")!=std::string::npos);
 frame.pitchDeg=std::numeric_limits<float>::quiet_NaN();
 MockPrint bad;
 assert(!EDATelemetry::printCSV(bad,frame) && bad.s.str().empty());
 // ---- V1.7.1
 { // setAlpha / setAdaptiveEnabled tutarliligi
   EDAComplementaryFilter g(0.98f); assert(g.configureAdaptive(0.998f)); assert(g.setAdaptiveEnabled(true));
   assert(!g.setAlpha(0.999f) && std::fabs(g.alpha()-0.98f)<1e-6f);   // yuksek alpha'dan buyuk reddedilir
   assert(g.setAlpha(0.998f));
   assert(g.setAdaptiveEnabled(false)); assert(g.setAlpha(0.999f));    // kapaliyken serbest
   assert(!g.setAdaptiveEnabled(true) && !g.adaptiveEnabled());         // alpha > highAlpha -> acilamaz
 }
 { // varsayilan esikler 0.03 / 0.02
   EDAComplementaryFilter g(0.98f); assert(g.setAdaptiveEnabled(true)); assert(g.update(0,0,0.01f,1.0f));
   assert(g.update(0,0,0.01f,1.04f) && g.highAcceleration());        // 0.04 >= 0.03
   assert(g.update(0,0,0.01f,1.025f) && g.highAcceleration());       // 0.025 > 0.02: histerezis
   assert(g.update(0,0,0.01f,1.015f) && !g.highAcceleration());      // 0.015 <= 0.02
 }
 { // eski (v1.7.0) esikler hala ayarlanabilir
   EDAComplementaryFilter g(0.98f); assert(g.configureAdaptive(0.998f,0.15f,0.10f)); g.setAdaptiveEnabled(true);
   assert(g.update(0,0,0.01f,1.0f)); assert(g.update(0,0,0.01f,1.04f) && !g.highAcceleration());
 }
 { // referanssiz genisletilmis CSV: bos alan
   EDATelemetryFrame fr={123,1,2,3,4,5,10000,1,0}; MockPrint nr;
   assert(EDATelemetry::printCSVExtended(nr,fr,std::numeric_limits<float>::quiet_NaN(),EDA_EVENT_DISTURBANCE,120,0.98f));
   assert(nr.s.str()=="123,1.000,2.000,3.000,4.000,5.000,10000,1,0,,1,120,0.980\n");
   MockPrint inf; assert(!EDATelemetry::printCSVExtended(inf,fr,std::numeric_limits<float>::infinity(),0,0,0.98f) && inf.s.str().empty());
 }
 return 0;
}
