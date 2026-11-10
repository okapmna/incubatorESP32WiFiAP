#pragma once
#include <Arduino.h>
#include <esp_mac.h>

// ==============================================================================
// MQTT Config Credentials
// ==============================================================================
const char* mqtt_server = "broker.hivemq.com";
const int   mqtt_port   = 8883;
const char* mqtt_user   = "";
const char* mqtt_pass   = "";

// ==============================================================================
// Device ID & MQTT Topics
// ==============================================================================
// Format ID : inc-<3 byte terakhir MAC eFuse, hex huruf kecil>
// Contoh    : inc-d4e5f6
// Topic dibentuk saat runtime oleh initDeviceId():
//   incubator/<device_id>/data   (perangkat -> broker: data sensor)
//   incubator/<device_id>/con    (broker -> perangkat: perintah kontrol)
char device_id[16]        = "";   // "inc-" + 6 hex + null
char mqtt_topic_data[48]  = "";
char mqtt_topic_con[48]   = "";

/** @brief Membaca ID unik ESP32 lalu menyusun topic MQTT. Panggil sekali di awal setup(). */
void initDeviceId() {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);   // MAC bawaan pabrik (tidak berubah walau flash dihapus)
  snprintf(device_id, sizeof(device_id), "inc-%02x%02x%02x", mac[3], mac[4], mac[5]);
  snprintf(mqtt_topic_data, sizeof(mqtt_topic_data), "incubator/%s/data", device_id);
  snprintf(mqtt_topic_con,  sizeof(mqtt_topic_con),  "incubator/%s/con",  device_id);
}