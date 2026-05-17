#pragma once
#include "config.h"

// ─── WiFiManager Setup (dipanggil di setup()) ─────────────────
void setupWifi() {
  lcd.setCursor(0, 0);
  lcd.print("WiFi Config...");

  wm.setConfigPortalBlocking(false);

  if (wm.autoConnect("ESP32_Incubator_AP")) {
    Serial.println("Terhubung ke WiFi (Boot)");
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected");
  } else {
    Serial.println("WiFi Config Portal Aktif");
    lcd.setCursor(0, 0);
    lcd.print("Connect AP: ESP32");
  }
  delay(1000);
  lcd.clear();
}

// ─── WiFi Reconnect Check (dipanggil di loop()) ───────────────
void handleWifiCheck() {
  if (WiFi.status() != WL_CONNECTED) {
    if (!wm.getConfigPortalActive()) {
      Serial.println("WiFi Putus! Memulai Config Portal...");
      wm.startConfigPortal("ESP32_Incubator_AP");
    }
  }
}
