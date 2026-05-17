#include "config.h"
#include "wifi_setup.h"
#include "mqtt_handler.h"
#include "sensor.h"

void setup() {
  Serial.begin(115200);

  // LCD & I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // PWM & Relay
  pinMode(RELAY_HUM_PIN, OUTPUT);
  ledcAttach(FAN_PWM_PIN, 5000, 8);
  ledcAttach(HEATER_PWM_PIN, 5000, 8);

  // PID
  myPID.setBangBang(0.5);

  // DHT22
  dht.begin();
  Serial.println("DHT22 Ditemukan dan Dimulai.");

  // Load Preferences (NVS)
  preferences.begin("incubator", false);
  target_temp = preferences.getDouble("t_temp", 37.0);
  target_hum  = preferences.getDouble("t_hum",  60.0);

  // WiFi
  setupWifi();

  // MQTT SSL
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  wm.process();

  unsigned long now = millis();

  // WiFi check setiap 1 detik
  if (now - lastWifiCheck >= 1000) {
    lastWifiCheck = now;
    handleWifiCheck();
  }

  // MQTT reconnect / loop
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    } else {
      client.loop();
    }
  }

  // Baca sensor & kontrol PID setiap 2 detik
  if (now - lastSensorRead >= 2000) {
    readSensorAndControl();
    lastSensorRead = now;
  }

  // Publish data ke MQTT setiap 5 detik
  if (now - lastMqttPublish >= 5000) {
    publishSensorData();
    lastMqttPublish = now;
  }
}