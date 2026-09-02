#include "config.h"
#include "wifi_handler.h"
#include "mqtt_handler.h"
#include "websocket_handler.h"
#include "sensor.h"
#include "fan_control.h"
#include "rtc_handler.h"

void handleMessage(String message) {
  Serial.println("Processing message: " + message);

  if (message == "ON") {
    digitalWrite(RELAYPIN_1, HIGH);
  } else if (message == "OFF") {
    digitalWrite(RELAYPIN_1, LOW);
  } else if (message.startsWith("pwm:")) {
    if (!fanAutoMode) {
      fanSpd = message.substring(4).toInt();
      fanSpd = constrain(fanSpd, 0, 255);
      analogWrite(PWMPIN, fanSpd);
    }
  } else if (message == "FAN_AUTO") {
    fanAutoMode = true;
    Serial.println("Fan mode set to: AUTO");
  } else if (message == "FAN_MANUAL") {
    fanAutoMode = false;
    Serial.println("Fan mode set to: MANUAL");
  } else if (message == "START_DATE") {
    String dateStr = rtcGetDateString();
    preferences.putString("startdate", dateStr);
    Serial.printf("Start date saved from RTC: %s\n", dateStr.c_str());
  } else if (message == "RESET_DATE") {
    preferences.remove("startdate");
    Serial.println("Start date preference has been reset.");
  } else {
    Serial.println("Unknown command received.");
  }
}

void setup() {
  Serial.begin(9600);
  preferences.begin("iot-data", false);

  rtcInit();
  dht.begin();

  pinMode(RELAYPIN_1, OUTPUT);
  digitalWrite(RELAYPIN_1, LOW);
  pinMode(PWMPIN, OUTPUT);
  analogWrite(PWMPIN, fanSpd);

  setupWiFi();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  Serial.println("MQTT client configured.");
}

void loop() {
  webSocket.loop();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static unsigned long lastTime = 0;
  const long interval = 2000;

  if (millis() - lastTime > interval) {
    lastTime = millis();

    float humidity, tempC;
    bool sensorOk = readSensor(tempC, humidity);
    float tempF = dht.convertCtoF(tempC);

    if (sensorOk && fanAutoMode) {
      fanAuto(tempC);
    }

    bool relayStatus = digitalRead(RELAYPIN_1);
    String startDate = preferences.getString("startdate", "Not Set");

    StaticJsonDocument<350> doc;
    doc["tempC"] = String(tempC, 2);
    doc["tempF"] = String(tempF, 2);
    doc["humidity"] = String(humidity, 2);
    doc["relayStatus"] = relayStatus;
    doc["fanSpd"] = fanSpd;
    doc["fanMode"] = fanAutoMode;
    doc["startdate"] = startDate;

    String espString;
    serializeJson(doc, espString);

    webSocket.broadcastTXT(espString);
    Serial.println("Data sent to WebSocket: " + espString);

    client.publish(mqtt_topic_publish, espString.c_str());
    Serial.println("Data published to MQTT: " + espString);
  }
}
