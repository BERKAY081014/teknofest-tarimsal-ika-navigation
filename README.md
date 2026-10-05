# TEKNOFEST – Tarımsal İnsansız Kara Aracı (TİKA)

TEKNOFEST Tarımsal İKA yarışması kapsamında geliştirilen, tarla koordinatlarını otonom takip edebilen, ultrasonik sensörlerle engelleri aşan otonom kara aracı yazılımı. (TİKA kategorisinin yarışmadan kaldırılması nedeniyle proje rafa kaldırılmıştır.)

## 🚜 Donanım Mimarisi
- **Kontrolcü:** Arduino Mega 2560 (16 MHz AVR)
- **Konumlandırma:** u-blox NEO-6M Yüksek Hassasiyetli GPS Modülü
- **Yönelim (Compass):** HMC5883L 3 Eksenli Dijital Manyetometre
- **Engel Algılama:** 3 Yönlü HC-SR04 Ultrasonik Dizi
- **Aktüatör:** Çift BTS7960 43A MOSFET H-Bridge Sürücü ve Redüktörlü DC Motorlar

## 📍 Algoritma Detayları
- **Haversine Metodu:** İki GPS koordinatı arasındaki küresel yüzey mesafesini milimetre hassasiyetiyle bulur.
- **Diferansiyel Dümenleme:** Hata açısına göre sol ve sağ tekerleklerin bağımsız PWM hız kontrolü.

**Geliştirici:** Berkay Bilgin ([@BERKAY081014](https://github.com/BERKAY081014)) - Takım Kaptanı\n