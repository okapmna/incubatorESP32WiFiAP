#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
// #include <DHT.h>            // Tidak digunakan
#include <Adafruit_AHTX0.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <AutoPID.h>

// Pin define
// #define DHTPIN 18           
// #define DHTTYPE DHT22       
#define FAN_PWM_PIN 19
#define HEATER_PWM_PIN 15
#define RELAY_HUM_PIN 4
#define SDA_PIN 21
#define SCL_PIN 22

// MQTT Config
const char* mqtt_server = "1118e8175bef4f45b3d76681ba47bf64.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "iwasabird";
const char* mqtt_pass = "54321Jan";

// Variabel parameter temp n hum
double target_temp = 37.0;
double target_hum = 60.0;
double current_temp, current_hum;
double heater_pwm_value;

// PID Config
#define KP 15.0
#define KI 0.5
#define KD 20.0
AutoPID myPID(&current_temp, &target_temp, &heater_pwm_value, 0, 255, KP, KI, KD);

// Library object def
// DHT dht(DHTPIN, DHTTYPE);
Adafruit_AHTX0 aht;
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClientSecure espClient;
PubSubClient client(espClient);
Preferences preferences;

WiFiManager wm;

// Time variable
unsigned long lastSensorRead = 0;
unsigned long lastMqttPublish = 0;
unsigned long lastWifiCheck = 0;

// Recive/Subscribe MQTT
void callback(char* topic, byte* payload, unsigned int length) {

  // Convert payload to string & text check
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Pesan masuk: ");
  Serial.println(message);

  // dev_getinfo message handler
  if (message == "dev_getinfo") {
    StaticJsonDocument<200> docResp;
    // Masukkan data target saat ini ke JSON
    docResp["target_temp"] = target_temp;
    docResp["target_hum"] = target_hum;

    char buffer[200];
    serializeJson(docResp, buffer);

    // Kirim balik ke topik data
    client.publish("incubator/19/data", buffer);
    Serial.println("Respon 'dev_getinfo' dikirim!");
    return;  // STOP di sini, jangan lanjut ke deserializeJson di bawah
  }

  // set target temp & hum
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
    Serial.println("Target diperbarui dari MQTT dan disimpan.");
  } else {
    Serial.println("Bukan JSON valid dan bukan perintah dikenal.");
  }
}

// Fungsi Reconect
unsigned long lastMqttReconnectAttempt = 0;
void reconnect() {
  unsigned long now = millis();
  if (now - lastMqttReconnectAttempt > 5000) {
    lastMqttReconnectAttempt = now;
    Serial.print("Menghubungkan ke MQTT...");
    String clientId = "ESP32-Incubator-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("Terhubung!");
      // Pastikan subscribe ke topik yang benar untuk menerima perintah
      client.subscribe("incubator/19/con");
    } else {
      Serial.print("Gagal, rc=");
      Serial.print(client.state());
      Serial.println(" Coba lagi nanti (Non-Blocking)");
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Setup LCD n I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Setup PWM
  pinMode(RELAY_HUM_PIN, OUTPUT);
  ledcAttach(FAN_PWM_PIN, 5000, 8);
  ledcAttach(HEATER_PWM_PIN, 5000, 8);

  // Setup heater
  myPID.setBangBang(0.5);

  // dht.begin();

  // AHT20 Setup
  if (!aht.begin()) {
    Serial.println("Gagal menemukan sensor AHT20! Cek kabel.");
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error!");
    while (1) delay(10);
  }
  Serial.println("AHT20 Ditemukan.");

  // Load Preferences
  preferences.begin("incubator", false);
  target_temp = preferences.getDouble("t_temp", 37.0);
  target_hum = preferences.getDouble("t_hum", 60.0);

  // Wifi Manager setup
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

  // Konfigurasi SSL MQTT
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  wm.process();

  unsigned long now = millis();

  // Cek koneksi wifi
  if (now - lastWifiCheck >= 1000) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      if (!wm.getConfigPortalActive()) {
        Serial.println("WiFi Putus! Memulai Config Portal...");
        wm.startConfigPortal("ESP32_Incubator_AP");
      }
    }
  }

  // Reconnect MQTT if WiFi connected
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    } else {
      client.loop();
    }
  }

  // Baca Sensor & Kontrol PID
  if (now - lastSensorRead >= 750) {

    // current_temp = dht.readTemperature(); // <-- DHT COMMANDED
    // current_hum = dht.readHumidity();     // <-- DHT COMMANDED

    // AHT20 READ
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);  // Mengambil data sensor

    current_temp = temp.temperature;
    current_hum = humidity.relative_humidity;

    if (!isnan(current_temp) && !isnan(current_hum)) {
      // Jalankan PID untuk Pemanas
      myPID.run();
      int final_pwm = (int)heater_pwm_value;
      if (current_temp < target_temp && final_pwm < 15) {
        final_pwm = 120;
      }

      ledcWrite(HEATER_PWM_PIN, final_pwm);
      // ledcWrite(HEATER_PWM_PIN, (int)heater_pwm_value);

      ledcWrite(FAN_PWM_PIN, 250);

      // Kontrol Humidifier
      if (current_hum < target_hum) {
        digitalWrite(RELAY_HUM_PIN, HIGH);
      } else {
        digitalWrite(RELAY_HUM_PIN, LOW);
      }

      // Tampilan LCD
      if (wm.getConfigPortalActive()) {
        lcd.setCursor(0, 0);
        lcd.print("SETUP WIFI MODE ");
      } else {
        lcd.setCursor(0, 0);
        lcd.print("T:" + String(current_temp, 1) + "C Set:" + String(target_temp, 1));
      }

      lcd.setCursor(0, 1);
      lcd.print("H:" + String(current_hum, 1) + "% Set:" + String(target_hum, 0));
    }
    lastSensorRead = now;
  }

  // Publish Data JSON ke MQTT Setiap 5 Detik
  if (now - lastMqttPublish >= 5000) {
    if (WiFi.status() == WL_CONNECTED && client.connected() && !isnan(current_temp)) {
      StaticJsonDocument<200> doc;
      doc["temperature"] = current_temp;
      doc["humidity"] = current_hum;

      char buffer[200];
      serializeJson(doc, buffer);
      client.publish("incubator/19/data", buffer);
    }
    lastMqttPublish = now;
  }
}