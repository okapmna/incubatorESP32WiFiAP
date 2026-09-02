#pragma once
#include "config.h"

void setupWiFi() {
  Serial.print("Connecting to WiFi : ");
  Serial.println(sta_ssid);
  WiFi.begin(sta_ssid, sta_pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nBerhasil terhubung ke WiFi utama!");
  Serial.print("IP : ");
  Serial.println(WiFi.localIP());

  Serial.print("Membuat Access Point: ");
  Serial.println(ap_ssid);
  WiFi.softAP(ap_ssid, ap_pass);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}
