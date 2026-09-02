#pragma once
#include "config.h"

void fanAuto(float temperature) {
  if (!fanAutoMode) return;

  if (temperature >= fanAutoThreshold) {
    fanSpd = 255;
  } else {
    float constrainedTemp = constrain(temperature, 30.0, fanAutoThreshold);
    fanSpd = map(constrainedTemp * 10, 300, fanAutoThreshold * 10, 100, 255);
  }
  analogWrite(PWMPIN, fanSpd);
  Serial.printf("Fan Auto Mode - Temp: %.2fC, Speed set to: %d\n", temperature, fanSpd);
}
