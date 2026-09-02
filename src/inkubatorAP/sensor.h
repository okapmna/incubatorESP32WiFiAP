#pragma once
#include "config.h"

bool readSensor(float &tempC, float &humidity) {
  humidity = dht.readHumidity();
  tempC = dht.readTemperature();

  if (isnan(tempC) || isnan(humidity)) {
    Serial.println("Gagal membaca data dari sensor DHT!");
    return false;
  }
  return true;
}
