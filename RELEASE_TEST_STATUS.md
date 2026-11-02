# dengebot v1.7.5 — Community Preview verification status

- This archive is **prepared for publication**; publication itself and GitHub Actions results must be confirmed on GitHub.
- `library.properties` version: **1.7.5**. Tag/release title should be **v1.7.5**.
- Local host tests, fake-Arduino demo tests, and ASan/UBSan: run before packaging; record the actual result in the release response.
- Arduino IDE compilation: project owner reports PASS on a broad selection of AVR, ESP32 and ESP8266 boards for prior v1.7.5 review code. **This exact final archive's 150 ms demo change has not been compiled on target hardware here.**
- Real Nano + L298N calibration: owner measured breakaway thresholds across left/right, forward/reverse; thresholds vary with motor, power, mechanical condition, and pulse duration. Physical evidence is limited to that setup.
- Motor waveform, independent EN disable on MCU stall and multi-robot validation: **NOT VERIFIED**.
- `PwmCalibrationDemo`: Nano/Uno/Mega only, pins: ENA D5 IN1 D4 IN2 D7; ENB D6 IN3 D8 IN4 D9. Test pulse **150 ms**; library default **120 ms**; cooldown 500 ms.
- IMPORTANT: Standalone software guard cannot cut motor power during MCU/loop lockup. Implement independent hardware disable and an accessible motor power disconnect. Wheels raised for calibration.
- PID math, SafetyGuard fault defaults and public API remain compatible; see CHANGELOG for additive defensive fixes.
