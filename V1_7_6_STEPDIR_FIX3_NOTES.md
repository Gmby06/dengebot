# v1.7.6 development - STEP/DIR FIX3

**Development build, not a release. Not tested on physical hardware.**

## Changes versus FIX2
- `EDAStepDirLEDC_ESP32::command()`: ENABLE stays LOW during pure frequency changes; ENABLE is cycled only for a direction reversal. A side starting from standstill sets its own DIR first. Unchanged side is not restarted.
- Example: start 200 STEP/s, +200 STEP/s every 10 ms (20000 STEP/s^2), then 2 s hold (6 s hard cap). Rates 200/500/1000/2000/5000/10000/20000/40000, default 2000.
- `StepDirController::setSpeeds()` clamps to `maxStepsPerSecond`.
- New host test `tests/ledc_adapter.cpp` (logic only, fake core).

## Still unverified (check on the bench)
1. `ledcWriteTone()` may reject frequencies below ~77 Hz (10-bit resolution, 80 MHz APB): `command()` would then disable and return false.
2. Arduino-ESP32 **3.x**: the two LEDC channels might share one timer (same initial freq/resolution). Run left and right at DIFFERENT rates (e.g. 7000 / 3000) and measure both STEP pins.
3. Duty 0 may take effect at the next LEDC period; check with a scope that no pulse leaks past a direction reversal.
4. 40000 STEP/s is a signal-rate test, not a realistic NEMA17 speed with A4988 at 12-24 V.

## If the motor still buzzes at 200 STEP/s
Check coil pairs (A1/A2, B1/B2), Vref/current limit, VMOT, MS1-3 wiring, **RESET and SLEEP not floating (tie RESET to SLEEP/VDD on A4988)**, 100 uF across VMOT, and scope STEP. Never plug/unplug the motor while powered.

## Run
`sh tests/run_host_tests.sh` (also `EDA_SANITIZE=1`).
