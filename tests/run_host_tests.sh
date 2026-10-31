#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
SAN=""
if [ "${EDA_SANITIZE:-0}" = "1" ]; then SAN="-fsanitize=address,undefined"; echo "(sanitizers on)"; fi
for t in smoke regression safety_guard; do
  c++ -std=c++11 -Wall -Wextra -Werror $SAN -I"$ROOT/src" "$ROOT/src/dengebot.cpp" "$ROOT/tests/$t.cpp" -o "$TMP/$t"
  "$TMP/$t"
done
c++ -std=c++11 -Wall -Wextra -Werror $SAN -I"$ROOT/src" "$ROOT/tests/motor_commands.cpp" -o "$TMP/motor"
c++ -std=c++11 -Wall -Wextra -Werror $SAN -I"$ROOT/src" "$ROOT/tests/dead_zone.cpp" -o "$TMP/dead_zone"
"$TMP/dead_zone"
"$TMP/motor"
c++ -std=c++11 -Wall -Wextra -Werror $SAN -I"$ROOT/src" "$ROOT/tests/filter_telemetry.cpp" -o "$TMP/filter_telemetry"
"$TMP/filter_telemetry"
c++ -std=c++11 -Wall -Wextra -Werror $SAN -I"$ROOT/src" "$ROOT/tests/gyro_bias.cpp" -o "$TMP/gyro_bias"
"$TMP/gyro_bias"
sh "$ROOT/tests/check_example.sh"
echo 'All host tests passed'
