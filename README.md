# Smart EGG Incubator ESP32 — MQTT (IoT)

Monitoring dan kontrol inkubator telur otomatis berbasis **ESP32** dengan koneksi **MQTT (IoT)**. Data suhu & kelembapan dikirim real-time ke broker MQTT dan dapat dikontrol dari mana saja via dashboard seperti [Unimon-dashboard](https://github.com/okapmna/unimon-dashboard.git) atau Node-RED.

## Fitur Utama

- **WiFiManager:** Koneksi WiFi mudah tanpa hardcode — cukup buka portal konfigurasi AP `ESP32_Incubator_AP`.
- **Internet Access:** Monitor dan kontrol dari mana saja.
- **MQTT Pub/Sub:** Data suhu/kelembapan dikirim real-time ke broker.
- **Menu OLED + Rotary Encoder:** Set target suhu & kelembapan langsung dari perangkat.
- **Kontrol Otomatis:**
  - **PID Heater:** PWM heater dijaga pada target suhu.
  - **Humidifier:** Relay nyala/mati otomatis berdasarkan target kelembapan.
  - **Fan:** Nyala terus menerus (bisa disesuaikan).
- **Status Sync:** Target suhu/kelembapan tersinkron dengan dashboard.
- **NVS (Preferences):** Pengaturan target tersimpan permanen di memori.

## Hardware & Komponen

- ESP32 Development Board
- Sensor AHT20 (Suhu & Kelembapan, I2C)
- OLED SSD1306 0.96" (I2C)
- LCD I2C 16x2
- Rotary Encoder (dengan tombol)
- AOD4148 MOSFET (kontrol heater PWM)
- L298N Motor Driver (kontrol fan PWM)
- Relay Module (kontrol humidifier)
- Power supply 5V / 12V

## Pin Configuration

| Komponen | GPIO ESP32 | Deskripsi |
| :--- | :--- | :--- |
| **HEATER** | GPIO 18 | PWM Heater (MOSFET) |
| **FAN** | GPIO 19 | PWM Fan |
| **RELAY_HUM** | GPIO 12 | Relay Humidifier |
| **SDA** | GPIO 21 | I2C Data (OLED, LCD, AHT20) |
| **SCL** | GPIO 22 | I2C Clock (OLED, LCD, AHT20) |
| **Rotary CLK** | GPIO 25 | Encoder |
| **Rotary DT** | GPIO 26 | Encoder |
| **Rotary SW** | GPIO 27 | Tombol Encoder |

## Menjalankan di PlatformIO (branch `main`)

Struktur project mengikuti standar PlatformIO.

```bash
# Install dependencies & build
pio run

# Upload ke ESP32
pio run -t upload

# Monitor serial
pio monitor
```

Kredensial MQTT & WiFi diatur lewat **WiFiManager** (pertama kali boot, konek ke AP `ESP32_Incubator_AP` untuk mengisi SSID/password WiFi).

Kredensial MQTT disimpan di `include/secret.h` — salin dari `include/secret-example.h` lalu isi server/port/user/pass dan topic.

## Menjalankan di Arduino IDE (branch `arduino-ide`)

Branch `arduino-ide` berisi versi sketch klasik Arduino IDE.

1. Buka file `firmware/incubator_esp32_mqtt/incubator_esp32_mqtt.ino` di Arduino IDE.
2. Instal library berikut lewat Library Manager:
   - ArduinoJson (bblanchon)
   - PubSubClient (knolleary)
   - WiFiManager (tzapu)
   - LiquidCrystal I2C
   - Adafruit AHTX0
   - Adafruit GFX Library
   - Adafruit SSD1306
   - AutoPID
3. Salin `secret-example.h` → `secret.h` lalu isi kredensial MQTT.
4. Pilih board **ESP32 Dev Module** dan upload.

## MQTT Topics

| Topic | Arah | Isi |
| :--- | :--- | :--- |
| `incubator/xx/data` | Publish | Data sensor (`temperature`, `humidity`) |
| `incubator/xx/con` | Subscribe | Perintah kontrol |

Perintah yang didukung di topic `con`:

- Kirim teks `dev_getinfo` → perangkat membalas nilai target saat ini.
- Kirim JSON untuk update target, contoh:
  ```json
  { "target_temp": 38.5, "target_hum": 65 }
  ```
  Nilai target otomatis disimpan ke NVS.

> Ganti `xx` di topic dengan ID perangkat kamu.

## Kode Lama (WiFi AP)

Kode inkubator versi WiFi Access Point (tanpa internet) yang lama diarsipkan dan **tidak lagi dikembangkan**:
- **branch `main`:** `old_code/inkubatorAP.ino`
- **branch `arduino-ide`:** `firmware/inkubatorAP/inkubatorAP.ino`

## Schematic & Dokumentasi

**Schematic Inkubator:**
<br>
<img width="800" alt="Incubator Schematic" src="schematics/skematik1.png" />

**Gambar Asli:**
<br>
<img width="800" alt="Incubator Real Picture" src="images/incubator32IoT.jpeg" />

## Kontributor

- **Oka Pmna** - [@okapmna](https://github.com/okapmna)
- **IDA BAGUS WILLI PARMITA** - [@WILIOP-666](https://github.com/WILIOP-666)
