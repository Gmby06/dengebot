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
c++ -std=c++11 -Wall -Wextra -Werror $SAN -I"$ROOT/src" "$ROOT/src/dengebot.cpp" "$ROOT/tests/unified_header.cpp" -o "$TMP/unified_header"
"$TMP/unified_header"
c++ -std=c++11 -Wall -Wextra -Werror $SAN -I"$ROOT/src" "$ROOT/tests/pwm_demo_safety.cpp" -o "$TMP/pwm_demo_safety"
"$TMP/pwm_demo_safety"
sh "$ROOT/tests/check_example.sh"
# The active motor demo is compiled only for boards its pin map was written for.
for BOARD in ARDUINO_AVR_NANO ARDUINO_AVR_UNO ARDUINO_AVR_MEGA2560; do
  c++ -std=c++11 -Wall -Wextra -Werror $SAN -DARDUINO_ARCH_AVR -D$BOARD -I"$ROOT/tests/fake_arduino" -I"$ROOT/src" "$ROOT/tests/demo_harness.cpp" -o "$TMP/demo_harness"
  "$TMP/demo_harness"
done
# Non-AVR and unsupported AVR boards (Gemma/ATtiny85, Leonardo, generic AVR): demo must touch no pin.
c++ -std=c++11 -Wall -Wextra -Werror $SAN -I"$ROOT/tests/fake_arduino" -I"$ROOT/src" "$ROOT/tests/demo_inert_nonavr.cpp" -o "$TMP/demo_inert_nonavr"
"$TMP/demo_inert_nonavr"
for BOARD in ARDUINO_AVR_LEONARDO ARDUINO_AVR_GEMMA ARDUINO_AVR_PRO; do
  c++ -std=c++11 -Wall -Wextra -Werror $SAN -DARDUINO_ARCH_AVR -D$BOARD -I"$ROOT/tests/fake_arduino" -I"$ROOT/src" "$ROOT/tests/demo_inert_nonavr.cpp" -o "$TMP/demo_inert_otheravr"
  "$TMP/demo_inert_otheravr"
done
c++ -std=c++11 -Wall -Wextra -Werror $SAN -I"$ROOT/src" "$ROOT/tests/pwm_calibration.cpp" -o "$TMP/pwm_calibration"
"$TMP/pwm_calibration"
c++ -std=c++11 -Wall -Wextra -Werror $SAN -I"$ROOT/src" "$ROOT/tests/step_dir.cpp" -o "$TMP/step_dir"
"$TMP/step_dir"
for MAJ in 2 3; do
  c++ -std=c++11 -Wall -Wextra -Werror $SAN -DFAKE_ESP_MAJOR=$MAJ -I"$ROOT/tests/fake_esp32" -I"$ROOT/src" "$ROOT/tests/ledc_adapter.cpp" -o "$TMP/ledc_adapter$MAJ"
  "$TMP/ledc_adapter$MAJ"
done
echo 'All host tests passed' 
