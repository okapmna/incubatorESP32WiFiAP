#pragma once
#include "config.h"

// Baca sensor AHT20, jalankan PID, kontrol humidifier & update display
void readSensorAndControl() {

  // ─── Baca AHT20 ───────────────────────────────────────────
  sensors_event_t humidity_evt, temp_evt;
  aht.getEvent(&humidity_evt, &temp_evt);   // humidity first, temp second

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

  if (wm.getConfigPortalActive()) {
    // Mode konfigurasi WiFi
    oled.setTextSize(1);
    oled.setCursor(10, 20);
    oled.print("== WIFI SETUP ==");
    oled.setCursor(8, 36);
    oled.print("Connect: ESP32_AP");
  } else {
    // Tampilan suhu besar di baris atas
    oled.setTextSize(2);
    oled.setCursor(0, 0);
    oled.print("T:");
    oled.print(current_temp, 1);
    oled.print("C");

    // Kelembapan di baris tengah
    oled.setTextSize(2);
    oled.setCursor(0, 20);
    oled.print("H:");
    oled.print(current_hum, 1);
    oled.print("%");

    // Target di baris bawah (kecil)
    oled.setTextSize(1);
    oled.setCursor(0, 48);
    oled.print("Set T:");
    oled.print(target_temp, 1);
    oled.print(" H:");
    oled.print(target_hum, 0);
  }
  oled.display();
}
