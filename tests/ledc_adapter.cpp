// Host test for EDAStepDirLEDC_ESP32.h against a fake core (logic only, NOT hardware).
#define ARDUINO_ARCH_ESP32
#define CONFIG_IDF_TARGET_ESP32
#include "Arduino.h"
#include "EDAStepDirLEDC_ESP32.h"
#include <cassert>
#include <cstdio>
std::vector<std::string> g_log; int g_pin[48];
void pinMode(uint8_t,uint8_t){}
void digitalWrite(uint8_t p,uint8_t v){g_pin[p]=v; if(p==33) g_log.push_back(v?"EN_HI":"EN_LO");}
void delayMicroseconds(unsigned){}
static int tones=0;
static void tone(){++tones;}
#if FAKE_ESP_MAJOR>=3
bool ledcAttachChannel(uint8_t,uint32_t,uint8_t,uint8_t){return true;}
bool ledcWrite(uint8_t,uint32_t){return true;}
uint32_t ledcWriteTone(uint8_t,uint32_t f){tone();return f;}
#else
uint32_t ledcSetup(uint8_t,uint32_t f,uint8_t){return f;}
void ledcAttachPin(uint8_t,uint8_t){}
void ledcWrite(uint8_t,uint32_t){}
uint32_t ledcWriteTone(uint8_t,uint32_t f){tone();return f;}
#endif
static int count(const char*s){int n=0;for(auto&x:g_log)if(x==s)++n;return n;}
int main(){
  EDA::StepDirLEDC_ESP32 m(25,26,27,14,33,false,true);
  assert(m.begin()); assert(g_pin[33]==HIGH);
  g_log.clear(); tones=0;
  assert(m.command(200,200,0)); assert(g_pin[33]==LOW); assert(tones==2);
  assert(g_pin[26]==LOW && g_pin[14]==HIGH);               // forward levels
  g_log.clear();
  for(int v=400;v<=40000;v+=200) assert(m.command(v,v,v));  // ramp: frequency only
  assert(count("EN_HI")==0 && count("EN_LO")==0);            // FIX3: ENABLE untouched
  g_log.clear(); assert(m.command(40000,40000,99999)); assert(g_log.empty()); // refresh no-op
  assert(m.command(-40000,40000,100000));                    // reversal of left: full path
  assert(count("EN_HI")==1 && g_pin[33]==LOW && g_pin[26]==HIGH);
  assert(!m.command(40001,0,1));                             // out of range disables
  assert(g_pin[33]==HIGH && !m.running());
  assert(m.command(0,500,2) && m.rightRate()==500 && m.leftRate()==0);
  assert(m.command(0,0,3) && g_pin[33]==HIGH && !m.running());
  assert(m.command(300,0,4)); assert(!m.watchdog(500,150)); assert(g_pin[33]==HIGH);
  puts("ledc_adapter: OK"); return 0;
}
