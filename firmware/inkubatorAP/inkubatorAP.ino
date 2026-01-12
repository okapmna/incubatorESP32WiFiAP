#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <WebSocketsServer.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <Preferences.h>

// Deklarasi pin RTC
#define IO_PIN 4                             // DATA
#define SCLK_PIN 5                           // Clock pin
#define CE_PIN 2                             // Chip Enable pin
ThreeWire myWire(IO_PIN, SCLK_PIN, CE_PIN);  // DAT, SCLK, RST
RtcDS1302<ThreeWire> Rtc(myWire);

// Preferences untuk menyimpan data
Preferences preferences;

// Connect to ssid
#define STA_SSID "NYUDIS"
#define STA_PASS "87654321C"

// Create new AP
#define AP_SSID "ESP32-INCUBATOR"
#define AP_PASS "87654321"

// Sensor DHT init
#define DHTPIN 23
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// RELAY IN1
#define RELAYPIN_1 18

// PWM
#define PWMPIN 19

// MQTT Config
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_topic_publish = ........";    // Topic ufor publish
const char* mqtt_topic_subscribe = "........"; // Topic for subscribe/command

// Init Network
WiFiClient espClient;
PubSubClient client(espClient);

// Init WS
WebSocketsServer webSocket = WebSocketsServer(81);

// Fan variable
int fanSpd = 180;
bool fanAutoMode = false;
float fanAutoThreshold = 38.0;


void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
void handleMessage(String message);
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);
void fanAuto(float temperature);


void setup() {
  Serial.begin(9600);
  preferences.begin("iot-data", false);

  // Memulai RTC
  Rtc.Begin();

  // Memulai DHT
  dht.begin();

  // Sebagai OUPUT Relay
  pinMode(RELAYPIN_1, OUTPUT);
  digitalWrite(RELAYPIN_1, LOW);
  pinMode(PWMPIN, OUTPUT);
  analogWrite(PWMPIN, fanSpd);

  // Connect to WiFi
  Serial.print("Connecting to WiFi : ");
  Serial.println(STA_SSID);
  WiFi.begin(STA_SSID, STA_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nBerhasil terhubung ke WiFi utama!");
  Serial.print("IP : ");
  Serial.println(WiFi.localIP());

  // Create AP
  Serial.print("Membuat Access Point: ");
  Serial.println(AP_SSID);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Start WS
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  Serial.println("MQTT client configured.");
}

void loop() {
  webSocket.loop();

  // Try connect mqtt
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Send data 
  static unsigned long lastTime = 0;
  const long interval = 2000;

  if (millis() - lastTime > interval) {
    lastTime = millis();

    // Baca sensor DHT
    float humidity = dht.readHumidity();
    float tempC = dht.readTemperature();
    float tempF = dht.readTemperature(true);

    if (isnan(tempC) || isnan(humidity)) {
      Serial.println("Gagal membaca data dari sensor DHT!");
    }

    // Check fanMode
    if (fanAutoMode) {
      fanAuto(tempC);
    }

    // Read start date preferences
    bool relayStatus = digitalRead(RELAYPIN_1);
    String startDate = preferences.getString("startdate", "Not Set");

    // Pack data to JSON
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

    // Send data
    webSocket.broadcastTXT(espString);
    Serial.println("Data sent to WebSocket: " + espString);

    client.publish(mqtt_topic_publish, espString.c_str());
    Serial.println("Data published to MQTT: " + espString);
  }
}

// Message Handler
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
    RtcDateTime now = Rtc.GetDateTime();
    char dateString[11];
    snprintf(dateString, sizeof(dateString), "%04u-%02u-%02u", now.Year(), now.Month(), now.Day());
    preferences.putString("startdate", dateString);
    Serial.printf("Start date saved from RTC: %s\n", dateString);
  } else if (message == "RESET_DATE") {
    preferences.remove("startdate");
    Serial.println("Start date preference has been reset.");
  } else {
    Serial.println("Unknown command received.");
  }
}

// Web socket event
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String message = String((char*)payload);
    Serial.printf("[%u] WebSocket message received: %s\n", num, message.c_str());
    handleMessage(message); // // call Handle function
  }
}

// Callback MQTT / event
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("MQTT message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Serial.println(message);

  handleMessage(message); // call Handle function
}

// Reconnect MQTT
void reconnect() {
  while (!client.connected()) {
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
      delay(5000);
    }
  }
}

// Fan Auto
void fanAuto(float temperature) {
  if (fanAutoMode) {
    if (temperature >= fanAutoThreshold) {
      fanSpd = 255;
    } else {
      float constrainedTemp = constrain(temperature, 30.0, fanAutoThreshold);
      fanSpd = map(constrainedTemp * 10, 300, fanAutoThreshold * 10, 100, 255);
    }
    analogWrite(PWMPIN, fanSpd);
    Serial.printf("Fan Auto Mode - Temp: %.2fC, Speed set to: %d\n", temperature, fanSpd);
  }
}