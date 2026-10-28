#include "dengebot.h"
#include <cassert>
#include <cmath>
int main(){
 EDAPIDConfig c=edaDefaultConfig(); EDABalancePID p(c);
 assert(p.lastUpdateValid());
 float u=p.update(0,2,0,0,.01f); assert(std::isfinite(u));assert(p.lastUpdateValid());
 p.update(0,2,NAN,0,.01f);assert(!p.lastUpdateValid());
 assert(!p.setTunings(-1,0,1));assert(p.setTunings(1,0,1));
 assert(!p.setConfig(EDAPIDConfig{}));
 p.reset();assert(p.lastUpdateValid());
 return 0;
}
