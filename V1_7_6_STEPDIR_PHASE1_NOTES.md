# dengebot v1.7.6 - STEP/DIR Phase 1 development package

**Status:** development prototype, not the published v1.7.6 release.

## First bench procedure
1. Disconnect motor power; confirm controller boots with ENABLE disabled (logic HIGH) using a physical 10-kohm pull-up and appropriate logic-interface circuitry.
2. Verify each driver input is compatible with ESP32 3.3-V logic and its STEP/DIR timing. Do not assume all STEP/DIR brands have identical inputs.
3. Set drivers current-limit and microstep mode according to their datasheets; record them. Use common logic ground only if the driver's interface requires it.
4. Verify wiring: STEP L=25, DIR L=26, STEP R=27, DIR R=14, active-low EN=33. Ensure the chosen drivers' ENABLE pins share compatible polarity; otherwise do not wire them together.
5. Raise and secure wheels; ensure easy access to motor-power cutoff. Upload the `ESP32_StepDirFirstTest` example. Use Serial Monitor at 115200, No line ending.
6. Choose 1/2/3/4 (individual wheel and direction); choose speed using + and - (200, 500, 1000, 2000, 4000 STEP/s). Press p for one 2-second test. s aborts.
7. Confirm motor direction by eye; verify STEP timing electrically with a logic analyzer if available. Repeat at the same microstep/current and supply for consistent comparisons.
8. Do not attempt balancing in this phase. ESP32 hardware-frequency and direction results must be physically measured before promoting this into a stable motor driver layer.

## Important constraints
- No acceleration ramp in this first standalone frequency test; start at 200 STEP/s, higher-speed direct-start tests can stall/skip steps.
- This header emits logical STEP levels; physical electrical polarity/width, external watchdog and driver power cutoff remain application responsibilities.
- The scheduler is C++11 and has no dynamic allocation; actual ISR execution performance must still be benchmarked on the target.
- `library.properties` version is 1.7.6 for the installable **development build**, while the stable published GitHub release remains 1.7.5.
- AVR Timer backend, generic driver profiles, load tests, real balancing and full First Run verification are future subphases.
