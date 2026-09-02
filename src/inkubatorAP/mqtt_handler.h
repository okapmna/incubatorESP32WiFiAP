#pragma once
#include "config.h"

void handleMessage(String message);

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("MQTT message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Serial.println(message);

  handleMessage(message);
}

void reconnect() {
  unsigned long now = millis();
  static unsigned long lastReconnectAttempt = 0;

  if (now - lastReconnectAttempt < 5000) return;
  lastReconnectAttempt = now;

  Serial.print("Attempting MQTT connection...");
  String clientId = "ESP32Client-";
  clientId += String(random(0xffff), HEX);
  if (client.connect(clientId.c_str())) {
    Serial.println("connected");
    client.subscribe(mqtt_topic_subscribe);
    Serial.print("Subscribed to: ");
    Serial.println(mqtt_topic_subscribe);
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
  }
}
