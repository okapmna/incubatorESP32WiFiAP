#pragma once
#include "config.h"

// Baca sensor AHT20, jalankan PID, kontrol humidifier & update OLED
void readSensorAndControl() {

  // ─── Baca AHT20 ───────────────────────────────────────────
  sensors_event_t humidity_evt, temp_evt;
  aht.getEvent(&humidity_evt, &temp_evt);  // humidity first, temp second

  current_temp = temp_evt.temperature;
  current_hum  = humidity_evt.relative_humidity;

  if (isnan(current_temp) || isnan(current_hum)) {
    Serial.println("Gagal membaca dari sensor AHT20!");
    return;
  }

  // ─── PID Heater ───────────────────────────────────────────
  myPID.run();
  int final_pwm = (int)heater_pwm_value;
  // Bang-bang boost: paksa PWM minimal jika suhu masih jauh di bawah target
  if (current_temp < target_temp && final_pwm < 15) {
    final_pwm = 120;
  }
  ledcWrite(HEATER_PWM_PIN, final_pwm);
  ledcWrite(FAN_PWM_PIN, 250);

  // ─── Humidifier Control ───────────────────────────────────
  if (current_hum <= (target_hum - 1.0)) {
    digitalWrite(RELAY_HUM_PIN, HIGH);  // Nyala jika kelembapan di bawah toleransi
  } else if (current_hum >= target_hum) {
    digitalWrite(RELAY_HUM_PIN, LOW);   // Mati jika kelembapan sudah mencapai target
  }

  // ─── OLED Display (128x64) ────────────────────────────────
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // Status WiFi di pojok kanan atas (Area warna putih pada OLED dual-color, y=0 s/d 15)
  oled.setTextSize(1);
  oled.setCursor(80, 0); // x=80 (kanan), y=0 (atas)
  if (WiFi.status() == WL_CONNECTED) {
    oled.print("WiFi:OK");
  } else if (wm.getConfigPortalActive()) {
    oled.print("WiFi:AP");
  } else {
    oled.print("WiFi:DC");
  }

  // ─── Area bawah (Warna biru pada OLED dual-color, y=16 s/d 63) ───
  // Suhu — teks besar
  oled.setTextSize(2);
  oled.setCursor(0, 18);
  oled.print("T:");
  oled.print(current_temp, 1);
  oled.print("C");

  // Kelembapan di baris tengah
  oled.setTextSize(2);
  oled.setCursor(0, 36);
  oled.print("H:");
  oled.print(current_hum, 1);
  oled.print("%");

  // Target di baris bawah (kecil)
  oled.setTextSize(1);
  oled.setCursor(0, 54);
  oled.print("Set T:");
  oled.print(target_temp, 1);
  oled.print(" H:");
  oled.print(target_hum, 0);
  
  oled.display();

  // ─── LCD I2C 16x2 Display ────────────────────────────────
  // LCD selalu menampilkan suhu dan kelembapan
  lcd.setCursor(0, 0);
  lcd.print("T:" + String(current_temp, 1) + "C Set:" + String(target_temp, 1));
  lcd.setCursor(0, 1);
  lcd.print("H:" + String(current_hum, 1) + "% Set:" + String(target_hum, 0));
}
