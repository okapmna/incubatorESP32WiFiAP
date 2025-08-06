#include <WiFi.h>
#include <WiFiAP.h>
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

#define DHTPIN 23
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// RELAY IN1
#define RELAYPIN_1 18

// PWM
#define PWMPIN 19

// Web Socket
WebSocketsServer webSocket = WebSocketsServer(81);

int fanSpd = 180;
bool fanAutoMode = false;

// Threshold suhu untuk kipas otomatis
float fanAutoThreshold = 38.0;


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

  // Membuat dan memulai wifi
  Serial.print("Connecting to WiFi : ");
  Serial.println(STA_SSID);
  WiFi.begin(STA_SSID, STA_PASS);

  // Menunggu koneksi STA
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Berhasil terhubung ke WiFi utama!");
  Serial.print("IP : ");
  Serial.println(WiFi.localIP());

  // Create New AP
  Serial.print("Membuat Access Point: ");
  Serial.println(AP_SSID);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Start ws
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();

  static unsigned long lastTime = 0;
  const long interval = 800;

  if (millis() - lastTime > interval) {
    lastTime = millis();

    //DHT sensor
    float humidity = dht.readHumidity();
    float tempC = dht.readTemperature();
    float tempF = dht.readTemperature(true);

    // Jalankan kontrol kipas otomatis jika mode auto aktif
    if (fanAutoMode && !isnan(tempC)) {
      fanAuto(tempC);
    }

    //RELAY status
    bool relayStatus = digitalRead(RELAYPIN_1);
    String startDate = preferences.getString("startdate", "Not Set");


    StaticJsonDocument<350> doc;
    doc["tempC"] = tempC;
    doc["tempF"] = tempF;
    doc["humidity"] = humidity;
    doc["relayStatus"] = relayStatus;
    doc["fanSpd"] = fanSpd;
    doc["fanMode"] = fanAutoMode;
    doc["startdate"] = startDate;

    String espString;
    serializeJson(doc, espString);

    webSocket.broadcastTXT(espString);
    Serial.println("Data send: " + espString);

    if (isnan(tempC) || isnan(humidity)) {
      Serial.println("Gagal membaca data dari sensor DHT!");
    }
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    Serial.printf("[%u] Pesan diterima: %s\n", num, payload);
    String message = String((char*)payload);

    if (message == "ON") {
      digitalWrite(RELAYPIN_1, HIGH);
    } else if (message == "OFF") {
      digitalWrite(RELAYPIN_1, LOW);
    } else if (message.startsWith("pwm:")) {
      if (!fanAutoMode) {
        fanSpd = message.substring(4).toInt();
        fanSpd = constrain(fanSpd, 0, 255);
        analogWrite(PWMPIN, fanSpd);
        Serial.printf("Set fan : %d\n", fanSpd);
      }
    } else if (message == "FAN_AUTO") {
      fanAutoMode = true;
      Serial.println("Fan mode: AUTO");
    } else if (message == "FAN_MANUAL") {
      fanAutoMode = false;
      Serial.println("Fan mode: MANUAL");
    }
    else if (message == "START_DATE") {
      RtcDateTime now = Rtc.GetDateTime();
      // YYYY-MM-DD
      char dateString[11];
      snprintf(dateString, sizeof(dateString), "%04u-%02u-%02u", now.Year(), now.Month(), now.Day());

      // Simpan string tanggal ke Preferences
      preferences.putString("startdate", dateString);
      Serial.printf("Start date from RTC saved: %s\n", dateString);
    }

    else if (message == "RESET_DATE") {
      preferences.clear();
      Serial.println("Start date preference has been reset.");
    }
  }
}


void fanAuto(float temperature) {
  if (fanAutoMode) {
    if (temperature >= fanAutoThreshold) {
      fanSpd = 255;
    } else {
      // Di bawah threshold, atur kecepatan secara proporsional berdasarkan suhu
      // Contoh: map suhu dari 30°C hingga threshold (38°C) ke rentang PWM 100-255
      float constrainedTemp = constrain(temperature, 30.0, fanAutoThreshold);
      fanSpd = map(constrainedTemp * 10, 300, fanAutoThreshold * 10, 100, 255);
    }

    // Terapkan kecepatan kipas yang sudah dihitung
    analogWrite(PWMPIN, fanSpd);
    Serial.printf("Fan Auto Mode - Temp: %.2fC, Speed set to: %d\n", temperature, fanSpd);
  }
}