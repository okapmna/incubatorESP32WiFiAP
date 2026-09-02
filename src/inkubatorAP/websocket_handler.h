#pragma once
#include "config.h"

void handleMessage(String message);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String message = String((char*)payload);
    Serial.printf("[%u] WebSocket message received: %s\n", num, message.c_str());
    handleMessage(message);
  }
}
