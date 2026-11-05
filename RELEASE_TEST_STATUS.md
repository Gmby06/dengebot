# dengebot v1.7.6 — GitHub yayın adayı kontrol durumu (2026-11-05)

**Durum:** Geliştirme adayı; GitHub sürümü yayımlanmadı. Yayın kararı için aşağıdaki eksik doğrulamalar tamamlanmalıdır.

- `library.properties`: 1.7.6; önceki yayımlanmış referans: v1.7.5.
- Önceki `(6)` paketinde bildirilen `-Werror=reorder` FIX4 hatası, kaynak `(7)` paketinde önceden düzeltilmişti.
- İç içe tekrarlanan `examples/PwmCalibrationDemo/PwmCalibrationDemo` klasörü `(7)` paketinde yoktu.
- A4988 ve DRV8825 için ayrı örnekler mevcuttur: `ESP32_A4988_Demo` ve `ESP32_DRV8825_Demo`.
- Klasik ESP32 demo pinleri: SOL STEP=18 DIR=19; SAĞ STEP=23 DIR=5; ortak ENABLE=33 (aktif LOW). Diğer kartlarda örnekler boştur (pin sürmez).
- CI board derleme matrisine iki örnek eklendi; **GitHub Actions çıktıları henüz doğrulanmadı**.
- **Yerel test sonuçları:** Son paketin test kayıtlarına ve teslim mesajına bakın. Host testleri gerçek ESP32 kart derlemesi yerine geçmez.
- **Arduino-ESP32 2.0.17 ve 3.0.7 gerçek kart derlemesi:** bu ortamda Arduino CLI yoksa doğrulanamaz; GitHub CI'da tamamlanmalıdır.
- **Gerçek A4988/DRV8825 kablo, STEP dalga formu, hız, akım, ısınma ve bağımsız acil durdurma:** bu revizyon için doğrulanmadı.
- PID, sensör füzyonu, DC PWM kodları ve STEP/DIR motor sürme mantığı bu yayın hazırlığında değiştirilmedi.
- Motor testinde tekerlekleri havaya kaldırın, sürücü akım limitini ayarlayın, 3.3V mantık uyumluluğunu doğrulayın, ENABLE'a harici 10k pull-up ve bağımsız güç kesme kullanın. MCU kilitlenirse yazılım watchdog'u çalışmaz.

**GitHub açıklaması (TR):** A4988 ve DRV8825 motor sürücüleri için ayrı demolar eklendi ve gerekli düzeltmeler yapıldı.
