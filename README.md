# Inkubator WiFi AP
Project Inkubator WiFi AP adalah sebuah proyek yang bertujuan untuk mengontrol dan monitoring suhu, kelembaban, dan lampu dalam suatu inkubator. Proyek ini menggunakan ESP32 sebagai mikrokontroler utama, yang terhubung dengan sensor DHT11 untuk membaca suhu dan kelembaban, serta lampu yang diatur melalui relay.
Pengontrolan dilakukan melalui antarmuka web yang dapat diakses melalui jaringan WiFi yang disediakan oleh ESP32. Antarmuka web ini memungkinkan pengguna untuk mengatur suhu, kelembaban, dan lampu, serta melihat data historis dan status inkubator secara real-time.

## Fitur
- Pemantauan Suhu dan Kelembaban: Menggunakan sensor DHT11
- Kontrol Lampu: Melalui relay yang dapat diaktifkan/nonaktifkan
- Kontrol Kipas: Pengaturan kecepatan manual atau otomatis berdasarkan suhu
- Timer Inkubasi: Menghitung hari yang telah berlalu sejak inkubasi dimulai
- Antarmuka Web: Kontrol dan pemantauan melalui browser

## Hardware & Komponen
- ESP32
- Sensor DHT11
- Relay
- Fan DC 12V
- Motor Driver
- RTC DS1302
- Breadboard
- Kabel Jumper
- Adaptor 5V

## Konfigurasi Pin
- Sensor DHT11: Pin 23
- Relay: Pin 18
- Fan DC: Pin 19
- RTC DS1302:
  - DATA: Pin 4
  - CLK: Pin 5
  - RST: Pin 2

## Cara penggunaan
- Hubungkan perangkat ke jaringan WiFi dengan SSID "NYUDISSS" (password: "87654321C")
- Open file index.html atau bisa gunakan liveserver maka akan otomatis membuka halaman web Inkubator WiFi AP

Memantau suhu dan kelembaban
Mengontrol lampu (hidup/mati)
Mengatur kecepatan kipas (manual atau otomatis)
Mengelola timer inkubasi (mulai/berhenti/reset)