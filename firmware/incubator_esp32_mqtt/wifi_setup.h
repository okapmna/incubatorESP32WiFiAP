#pragma once
#include "config.h"

// ─── WiFiManager Setup (dipanggil di setup()) ─────────────────
void setupWifi() {
  wm.setConfigPortalBlocking(false);

  if (wm.autoConnect("ESP32_Incubator_AP")) {
    Serial.println("Terhubung ke WiFi (Boot)");
  } else {
    Serial.println("WiFi Config Portal Aktif");
  }
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
