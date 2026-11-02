# v1.7.5 development candidate - verification

- Run `sh tests/run_host_tests.sh` and `EDA_SANITIZE=1 sh tests/run_host_tests.sh`.
- `check_example.sh` is a HOST stub syntax check (both AVR and non-AVR paths), not Arduino GCC or ESP compilation.
- Check real GitHub CI on 6 architectures before publishing.
- Verify AVR PWM waveform and physical driver cutoff with wheels up, external stop, and verified pins.
- D12 (IN4) is the provided example, NOT the user's private Nano robot pinout (may be D9). Adjust wiring explicitly.
- Start with a current-limited supply, remove L298N EN jumpers; keep motor power disconnected during upload.
- Ensure independent watchdog/driver-EN hardware cutoff: `command(nowMs)` cannot stop a frozen loop.
- v1.7.4 PID algorithm and safety guard are unchanged.
