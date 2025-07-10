# Inkubator WiFi AP
Project Inkubator WiFi AP adalah sebuah proyek yang bertujuan untuk mengontrol dan monitoring suhu, kelembaban, dan lampu dalam suatu inkubator. Proyek ini menggunakan ESP32 sebagai mikrokontroler utama, yang terhubung dengan sensor DHT11 untuk membaca suhu dan kelembaban, serta lampu yang diatur melalui relay.Pengontrolan dilakukan melalui sebuah halaman web. Antarmuka web ini memungkinkan pengguna untuk mengatur suhu, kelembaban, dan lampu, serta set timer waktu mulai dan stop inkubasi 

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

## TOPOLOGI
<img width="3071" height="1361" alt="TOPOLOGI INKUBATOR" src="https://github.com/user-attachments/assets/c8cc301e-4aeb-42fd-81d8-8f2d1081cb95" />

## REAL PICTURE + POSTER
<img width="1587" height="2245" alt="Lilac Pastel Blue Simple Lined Literary Devices Portrait Educational Poster (2)" src="https://github.com/user-attachments/assets/2dbddeca-c2ca-453d-af8a-b3c1678c55ac" />
