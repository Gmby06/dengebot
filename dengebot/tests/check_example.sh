#!/bin/sh
# Syntax-checks every example sketch against a tiny Arduino stub (no board toolchain needed).
set -e
D=$(cd "$(dirname "$0")" && pwd)
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
cat > "$T/Arduino.h" <<'STUB'
#pragma once
struct EDAStubSerial{
  void begin(long){}
  template<class T> void print(T){}
  template<class T> void print(T,int){}
  template<class T> void println(T){}
  void println(){}
};
extern EDAStubSerial Serial; void delay(unsigned long); unsigned long millis();
STUB
N=0
for ino in "$D"/../examples/*/*.ino; do
  { echo '#include "Arduino.h"'; cat "$ino"; } > "$T/ex.cpp"
  g++ -std=c++11 -fsyntax-only -Wall -Wextra -I"$T" -I"$D/../src" "$T/ex.cpp"
  N=$((N+1))
done
echo "examples OK ($N)"
