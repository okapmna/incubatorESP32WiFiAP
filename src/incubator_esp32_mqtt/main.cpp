#include <Arduino.h>
#include "config.h"
#include "wifi_setup.h"
#include "mqtt_handler.h"
#include "sensor.h"
#include "menu.h"

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED SSD1306 tidak ditemukan!");
  } else {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(20, 28);
    oled.print("Initializing...");
    oled.display();
    Serial.println("OLED SSD1306 OK.");
  }

  setupMenu();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  Serial.println("LCD I2C 16x2 OK.");

  pinMode(RELAY_HUM_PIN, OUTPUT);
  ledcAttach(FAN_PWM_PIN, 5000, 8);
  ledcAttach(HEATER_PWM_PIN, 5000, 8);

  myPID.setBangBang(0.5);

  if (!aht.begin()) {
    Serial.println("Sensor AHT20 tidak ditemukan! Periksa wiring.");
  } else {
    Serial.println("AHT20 Ditemukan dan Dimulai.");
  }

  preferences.begin("incubator", false);
  target_temp = preferences.getDouble("t_temp", 37.0);
  target_hum  = preferences.getDouble("t_hum",  60.0);

  setupWifi();

  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  wm.process();

  handleMenu();

  unsigned long now = millis();

  if (now - lastWifiCheck >= 1000) {
    lastWifiCheck = now;
    handleWifiCheck();
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    } else {
      client.loop();
    }
  }

  if (now - lastSensorRead >= 2000) {
    readSensorAndControl();
    lastSensorRead = now;
  }

  if (now - lastMqttPublish >= 5000) {
    publishSensorData();
    lastMqttPublish = now;
  }
}
