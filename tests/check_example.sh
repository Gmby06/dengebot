#!/bin/sh
# Syntax-checks every example sketch against a tiny Arduino stub (no board toolchain needed).
# The stub covers Serial, F(), timing and basic digital/PWM pin functions. It is NOT a behavior simulator.
set -e
D=$(cd "$(dirname "$0")" && pwd)
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
cat > "$T/Arduino.h" <<'STUB'
#pragma once
#include <stdint.h>
#define F(x) (x)
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
struct EDAStubSerial{
  void begin(long){}
  template<class T> void print(T){}
  template<class T> void print(T,int){}
  template<class T> void println(T){}
  template<class T> void println(T,int){}
  void println(){}
  int available(){return 0;}
  int read(){return -1;}
};
extern EDAStubSerial Serial;
void delay(unsigned long); void delayMicroseconds(unsigned int);
unsigned long millis(); unsigned long micros();
void pinMode(uint8_t,uint8_t); void digitalWrite(uint8_t,uint8_t); int digitalRead(uint8_t);
void analogWrite(uint8_t,int);
STUB
N=0
for ino in "$D"/../examples/*/*.ino; do
  { echo '#include "Arduino.h"'; cat "$ino"; } > "$T/ex.cpp"
  g++ -std=c++11 -fsyntax-only -Wall -Wextra -DARDUINO_ARCH_AVR -DARDUINO_AVR_NANO -I"$T" -I"$D/../src" "$T/ex.cpp"
  g++ -std=c++11 -fsyntax-only -Wall -Wextra -DARDUINO_ARCH_AVR -DARDUINO_AVR_LEONARDO -I"$T" -I"$D/../src" "$T/ex.cpp"
  g++ -std=c++11 -fsyntax-only -Wall -Wextra -I"$T" -I"$D/../src" "$T/ex.cpp"
  N=$((N+1))
done
# Gemma (ATtiny85) has no HardwareSerial Serial instance. Check the demo
# without providing any Serial stub, so an accidental Serial reference fails.
cat > "$T/Arduino.h" <<'GEMMA_STUB'
#pragma once
GEMMA_STUB
{ echo '#include "Arduino.h"'; cat "$D/../examples/PwmCalibrationDemo/PwmCalibrationDemo.ino"; } > "$T/gemma.cpp"
g++ -std=c++11 -fsyntax-only -Wall -Wextra -Werror -DARDUINO_ARCH_AVR -DARDUINO_AVR_GEMMA -D__AVR_ATtiny85__ -I"$T" -I"$D/../src" "$T/gemma.cpp"
echo "Gemma/ATtiny85 inert demo syntax OK (no Serial)"
echo "examples OK ($N)"
