#pragma once
#include "config.h"

void readSensorAndControl() {
  sensors_event_t humidity_evt, temp_evt;
  aht.getEvent(&humidity_evt, &temp_evt);

  current_temp = temp_evt.temperature;
  current_hum  = humidity_evt.relative_humidity;

  if (isnan(current_temp) || isnan(current_hum)) {
    Serial.println("Gagal membaca dari sensor AHT20!");
    return;
  }

  myPID.run();
  int final_pwm = (int)heater_pwm_value;
  if (current_temp < target_temp && final_pwm < 15) {
    final_pwm = 120;
  }
  ledcWrite(HEATER_PWM_PIN, final_pwm);
  ledcWrite(FAN_PWM_PIN, 250);

  if (current_hum <= (target_hum - 1.0)) {
    digitalWrite(RELAY_HUM_PIN, HIGH);
  } else if (current_hum >= target_hum) {
    digitalWrite(RELAY_HUM_PIN, LOW);
  }

  String line1 = "T:" + String(current_temp, 1) + "C Set:" + String(target_temp, 1);
  while (line1.length() < 16) line1 += " ";
  lcd.setCursor(0, 0);
  lcd.print(line1);

  String line2 = "H:" + String(current_hum, 1) + "% Set:" + String((int)target_hum);
  while (line2.length() < 16) line2 += " ";
  lcd.setCursor(0, 1);
  lcd.print(line2);
}
