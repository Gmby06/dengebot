# v1.7.6 development — STEP/DIR FIX2 (2026-11-05)

**Development build, not an official release. No GitHub changes.**

## Why this revision

- Previous example used a 20 us software timer ISR to toggle STEP pins; its 4000 STEP/s limit rejected 5000–40000 commands.
- Reports showed the motor energized and made sound at 200 STEP/s but the shaft did not visibly rotate. This physical symptom is **not yet explained**, and cannot be fixed with a speed-limit change alone.
- This test example now uses the ESP32's **hardware LEDC** to emit two independent STEP waveforms (channels 0 and 2). The example no longer calls the portable 20-us tick scheduler.
- The new backend is `src/EDAStepDirLEDC_ESP32.h`. It is a **platform-specific adapter**, not a driver-specific A4988 implementation.

## Wiring (unchanged)

- Left STEP GPIO25, DIR GPIO26; right STEP GPIO27, DIR GPIO14.
- Active-LOW common ENABLE GPIO33. External 10k pull-up to ESP32 3.3V, common grounds.
- Old-tested direction conventions: left forward LOW, right forward HIGH. Verify wheels lifted.
- Adjust the current limit and sleep/reset of your particular STEP/DIR driver.
- Do **not** connect opto-isolated industrial driver inputs directly to a 3.3V GPIO without a compatible interface circuit.

## Operation

Serial at 115200, no line ending: `1..6` motor/direction selection; `+`/`-` rate selection; `p` runs 2 seconds; `s` stops. Rates are 2000, 5000, 10000, 20000, 40000 STEP/s. For >2000 the command rises in increments of 5000 STEP/s per 50 ms. 40000 is an **experimental signal-rate setting**, not a supported NEMA17 mechanical RPM promise. 1/8 microstep + 200-step motor: 2000 STEP/s = 75 theoretical RPM, 40000 = 1500 theoretical RPM. Motor may stall or lose torque.

A `TEST START` message indicates a software command was accepted, NOT that STEP edges were physically measured or that the motor shaft rotated.

## Limits and safety

- LEDC runs even if `loop()` pauses; the `watchdog()` requires loop calls and cannot remove pulses if the MCU/main loop locks up. Use an independent hardware power/enable cutoff if movement on MCU failure is hazardous.
- `s` and test timeout set the output duty to zero and ENABLE HIGH.
- The adapter changes the output waveform on every command update (50ms); designed **only for bench verification**, not yet a smooth closed-loop balancing actuator.
- `src/EDAStepDirController.h` portable 20-us scheduler is intentionally retained unchanged for backward compatibility/host tests, including its own 4000 STEP/s default max. Do NOT use that old example/scheduler for the new 40000 rate table.
- No physical speed, timing, current or rotation test has been carried out in this development environment.

## If motor still just buzzes

1. Power off before moving wiring. Check motor's two independent coil pairs across A1/A2 and B1/B2.
2. Check VMOT and motor current limit and driver temperature. Driver ENABLE active LOW on the typical carrier, but verify actual board.
3. Check MS1/MS2/MS3 physical wiring, not just remembered setting. `HIGH,HIGH,LOW` is 1/8 on an A4988-type module, but may differ for another driver.
4. Scope or logic analyzer STEP GPIO25/27: STEP should be a ~50% duty pulse train. At 2000 STEP/s period 500us, HIGH ~250us; this comfortably exceeds A4988 min pulse widths. Also verify DIR and ENABLE on the driver inputs.
5. If the shaft still doesn't turn with valid STEP and power, inspect driver/motor and coil wiring rather than changing PID gains.

## Verification

- Host suite: run `sh tests/run_host_tests.sh`.
- Physical verification pending: start at 2000 STEP/s, separate motors and directions. Do not start the robot balancing from this test.
