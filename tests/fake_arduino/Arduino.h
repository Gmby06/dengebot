#pragma once
#include <stdint.h>
#include <string>
#include <deque>
#include <vector>
#include <type_traits>
#define F(x) (x)
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
extern unsigned long g_now;
inline unsigned long millis(){return g_now;}
struct PinEv{unsigned long t; uint8_t pin; int val; bool pwm;};
extern std::vector<PinEv> g_ev; extern int g_pwm[24]; extern int g_dig[24]; extern int g_mode[24];
inline void pinMode(uint8_t p,uint8_t m){g_mode[p]=m;}
inline void digitalWrite(uint8_t p,uint8_t v){g_dig[p]=v; g_ev.push_back({g_now,p,v,false});}
inline int digitalRead(uint8_t p){return g_dig[p];}
inline void analogWrite(uint8_t p,int v){g_pwm[p]=v; g_ev.push_back({g_now,p,v,true});}
struct FakeSerial{
  std::deque<char> in; std::string out;
  void begin(long){}
  int available(){return (int)in.size();}
  int read(){ if(in.empty()) return -1; char c=in.front(); in.pop_front(); return (unsigned char)c; }
  void print(const char*s){out+=s;}
  template<class T> typename std::enable_if<std::is_arithmetic<T>::value>::type print(T v){out+=std::to_string((long long)v);}
  void println(const char*s){out+=s;out+="\n";}
  template<class T> typename std::enable_if<std::is_arithmetic<T>::value>::type println(T v){print(v);out+="\n";}
  void println(){out+="\n";}
};
extern FakeSerial Serial;
