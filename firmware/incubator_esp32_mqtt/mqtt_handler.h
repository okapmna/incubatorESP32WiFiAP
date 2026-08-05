#pragma once
#include "config.h"

// MQTT Subscribe Callback
void callback(char* topic, byte* payload, unsigned int length) {
  // Convert payload ke String
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message received: ");
  Serial.println(message);

  // Handler perintah dev_getinfo: kirim balik nilai target saat ini
  if (message == "dev_getinfo") {
    StaticJsonDocument<200> docResp;
    docResp["target_temp"] = target_temp;
    docResp["target_hum"]  = target_hum;
    char buffer[200];
    serializeJson(docResp, buffer);
    client.publish(mqtt_topic_data, buffer);
    Serial.println("Respon 'dev_getinfo' sent!");
    return;
  }

  // Handler JSON: update target temp / hum & simpan ke Memory permanen
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (!error) {
    if (doc.containsKey("target_temp")) {
      target_temp = doc["target_temp"];
      preferences.putDouble("t_temp", target_temp);
    }
    if (doc.containsKey("target_hum")) {
      target_hum = doc["target_hum"];
      preferences.putDouble("t_hum", target_hum);
    }
    Serial.println("Target updated from MQTT and saved.");
  } else {
    Serial.println("Not valid JSON and unknown command.");
  }
}

// MQTT Reconnect (Non-Blocking)
void reconnect() {
  unsigned long now = millis();
  if (now - lastMqttReconnectAttempt > 5000) {
    lastMqttReconnectAttempt = now;
    Serial.print("Menghubungkan ke MQTT...");
    String clientId = "ESP32-Incubator-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("Terhubung!");
      client.subscribe(mqtt_topic_con);
    } else {
      Serial.print("Gagal, rc=");
      Serial.print(client.state());
      Serial.println(" Coba lagi nanti (Non-Blocking)");
    }
  }
}

// Publish Data Sensor ke MQTT
void publishSensorData() {
  if (WiFi.status() == WL_CONNECTED && client.connected() && !isnan(current_temp)) {
    StaticJsonDocument<200> doc;
    doc["temperature"] = current_temp;
    doc["humidity"]    = current_hum;
    char buffer[200];
    serializeJson(doc, buffer);
    client.publish(mqtt_topic_data, buffer);
  }
}
