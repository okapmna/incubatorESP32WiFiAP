#pragma once
#include "config.h"

// Baca sensor DHT22, jalankan PID, kontrol humidifier & update LCD
void readSensorAndControl() {
  current_temp = dht.readTemperature();
  current_hum  = dht.readHumidity();

  if (isnan(current_temp) || isnan(current_hum)) {
    Serial.println("Gagal membaca dari sensor DHT22!");
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

  // ─── LCD Display ──────────────────────────────────────────
  if (wm.getConfigPortalActive()) {
    lcd.setCursor(0, 0);
    lcd.print("SETUP WIFI MODE ");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("T:" + String(current_temp, 1) + "C Set:" + String(target_temp, 1));
  }
  lcd.setCursor(0, 1);
  lcd.print("H:" + String(current_hum, 1) + "% Set:" + String(target_hum, 0));
}
