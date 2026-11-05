# v1.7.6 A4988 + DRV8825 STEP/DIR uyumluluk adayı

**Çalışan FIX3 `src/EDAStepDirLEDC_ESP32.h` kaynağı değiştirilmedi.** Bu pakette STEP/DIR sürücü profilleri ve A4988/DRV8825 için ayrı motor test örnekleri vardır. Kontrol çekirdeği ve LEDC motor sürme mantığı bu yayın hazırlığında değiştirilmedi.

## Donanım
- ESP32 klasik: SOL STEP=18, SOL DIR=19, SAG STEP=23, SAG DIR=5, ENABLE=33 (aktif LOW).
- Motor ileri yönleri: sol HIGH, sağ LOW.
- DRV8825 nSLEEP ve nRESET HIGH olmalı. Bazı taşıyıcı kartlarda nSLEEP'in A4988 gibi kendiliğinden HIGH olmadığını unutmayın. Taşıyıcı üreticinizin şemasını kontrol edin.
- DRV8825 mikrostep MODE0/MODE1/MODE2 tablo değerleri A4988'in 1/16 düzeninden farklıdır. Kod bunları otomatik ayarlamaz.
- DRV8825 akım limitinin VREF formülü A4988 ile aynı değildir. Kendi kartınızın akım algılama direnci için üretici verisini kullanın.
- VMOT'a uygun güç ve GND ortaklığı gerekir. Motor kablosunu sistem enerjiliyken çıkarıp takmayın.

## Yazılım
- Arduino IDE -> Dosya -> Örnekler -> dengebot -> ESP32_A4988_Demo veya ESP32_DRV8825_Demo.
- Sürücü seçimini ayrı örnek dosyasıyla yapın; `EDA_DRIVER_SELECTION` makrosu kullanılmaz.
- Seri 115200. `1` sol ileri; `3` sağ ileri; `5` ikisi ileri; `6` ikisi geri; `p` başlat; `s` durdur.
- 200, 500, 1000 ve 2000 STEP/s önce deneyin, sonrasında yüksek hızları değerlendirin.
- Tekerlekler havada, ayaklar eller ve kablolar motorlardan uzakta test edin.

## Datasheet (resmi)
- Texas Instruments DRV8825 datasheet: https://www.ti.com/lit/ds/symlink/drv8825.pdf — STEP HIGH/LOW >= 1.9 us; DIR setup/hold >= 650 ns; nENABLE >= 650 ns.
- Pololu DRV8825 carrier: https://www.pololu.com/product/2133 — nSLEEP, microstep ve A4988 farkları.

## Test durumu
- A4988 için önceki FIX3 fiziksel motor denemeleri raporlandı; bu ZIP içindeki iki yeni ayrı demonun gerçek donanım doğrulaması yapılmadı.
- DRV8825 henüz gerçek donanımda test edilmedi. Profil datasheet uyumluluğu incelemesi sağlar; elektriksel uyumluluğu garanti etmez.
- FIX3 LEDC backend yalnızca klasik ESP32 için ve 200-40000 STEP/s aralığındaki test için tasarlanmıştır.
