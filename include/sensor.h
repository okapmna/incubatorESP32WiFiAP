#pragma once
#include "config.h"

// Baca sensor SHT30, jalankan PID, kontrol humidifier & update OLED
void readSensorAndControl() {

  // ─── Baca SHT30 ───────────────────────────────────────────
  current_temp = sht30.readTemperature();
  current_hum  = sht30.readHumidity();

  if (isnan(current_temp) || isnan(current_hum)) {
    Serial.println("Gagal membaca dari sensor SHT30!");
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


  // ─── LCD I2C 16x2 Display ────────────────────────────────
  // LCD selalu menampilkan suhu dan kelembapan
  String line1 = "T:" + String(current_temp, 1) + "C Set:" + String(target_temp, 1);
  while (line1.length() < 16) line1 += " ";
  lcd.setCursor(0, 0);
  lcd.print(line1);

  String line2 = "H:" + String(current_hum, 1) + "% Set:" + String((int)target_hum);
  while (line2.length() < 16) line2 += " ";
  lcd.setCursor(0, 1);
  lcd.print(line2);
}
