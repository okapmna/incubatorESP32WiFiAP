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

// Preferences untuk menyimpan data timer
Preferences preferences;

const char* ssid = "NYUDISSS";
const char* password = "87654321C";

#define DHTPIN 23
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// RELAY IN1
#define RELAYPIN_1 18

// PWM
#define PWMPIN 19

// Web Socket
WebSocketsServer webSocket = WebSocketsServer(81);

// Timer variables
bool timerRunning = false;
RtcDateTime timerStartDate;
int dayCount = 0;
int fanSpd = 180;
bool fanAutoMode = false;

// Threshold suhu untuk kipas otomatis
float fanAutoThreshold = 38.0;

void setup() {
  Serial.begin(9600);

  // Memulai RTC
  Rtc.Begin();
  
  // Check if RTC is running
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  if (!Rtc.IsDateTimeValid()) {
    Serial.println("RTC lost confidence in the DateTime!");
    Rtc.SetDateTime(compiled);
  }
  
  if (Rtc.GetIsWriteProtected()) {
    Serial.println("RTC was write protected, enabling writing now");
    Rtc.SetIsWriteProtected(false);
  }
  
  if (!Rtc.GetIsRunning()) {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  // Load timer data from preferences
  preferences.begin("timer", false);
  timerRunning = preferences.getBool("running", false);
  
  if (timerRunning) {
    // Load stored start date
    uint32_t storedTime = preferences.getUInt("startTime", 0);
    if (storedTime > 0) {
      timerStartDate = RtcDateTime(storedTime);
      
      // Hitung ulang dayCount berdasarkan tanggal mulai yang tersimpan
      RtcDateTime now = Rtc.GetDateTime();
      if (now.IsValid() && timerStartDate.IsValid() && storedTime <= now.Epoch32Time()) {
        dayCount = (now.Epoch32Time() - storedTime) / 86400;
        Serial.print("Timer berjalan selama: ");
        Serial.print(dayCount);
        Serial.println(" hari");
      } else {
        Serial.println("Data waktu tidak valid, mereset timer");
        resetTimer();
      }
    } else {
      Serial.println("Tidak ada data waktu mulai, mereset timer");
      resetTimer();
    }
  }
  preferences.end();

  // Memulai DHT
  dht.begin();

  // Sebagai OUPUT Relay
  pinMode(RELAYPIN_1, OUTPUT);
  digitalWrite(RELAYPIN_1, LOW);
  pinMode(PWMPIN, OUTPUT);
  analogWrite(PWMPIN, fanSpd);

  // Membuat dan memulai wifi
  WiFi.softAP(ssid, password);
  // Menampilkan IP Access Point
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP address: ");
  Serial.println(IP);

  // Start ws
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();

  static unsigned long lastTime = 0;

  if (millis() - lastTime > 800) {
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

    // Get date dan time
    RtcDateTime now = Rtc.GetDateTime();
    char datestring[20];
    char timestring[20];
    
    snprintf_P(datestring,
               sizeof(datestring),
               PSTR("%02u/%02u/%04u"),
               now.Day(),
               now.Month(),
               now.Year());
               
    snprintf_P(timestring,
               sizeof(timestring),
               PSTR("%02u:%02u:%02u"),
               now.Hour(),
               now.Minute(),
               now.Second());

    // Calculate days elapsed if timer is running
    if (timerRunning && now.IsValid() && timerStartDate.IsValid()) {
      uint32_t currentEpoch = now.Epoch32Time();
      uint32_t startEpoch = timerStartDate.Epoch32Time();
      
      if (startEpoch <= currentEpoch) {
        dayCount = (currentEpoch - startEpoch) / 86400; // 86400 seconds in a day
      } else {
        // Jika waktu mulai lebih besar dari waktu sekarang, ada masalah
        Serial.println("Waktu mulai lebih besar dari waktu sekarang, mungkin RTC reset");
        // Tetap gunakan data yang tersimpan, jangan reset
      }
    }

    // Format start date
    char startDateStr[20];
    if (timerRunning && timerStartDate.IsValid()) {
      snprintf_P(startDateStr,
                 sizeof(startDateStr),
                 PSTR("%02u/%02u/%04u"),
                 timerStartDate.Day(),
                 timerStartDate.Month(),
                 timerStartDate.Year());
    } else {
      strcpy(startDateStr, "Belum Diatur");
    }

    // SEND data to Web Socket
    if (!isnan(tempC) && !isnan(tempF) && !isnan(humidity)) {

      StaticJsonDocument<300> doc;
      doc["tempC"] = tempC;
      doc["tempF"] = tempF;
      doc["humidity"] = humidity;
      doc["relayStatus"] = relayStatus;
      doc["dateESP"] = datestring;
      doc["timeESP"] = timestring;
      doc["fanSpd"] = fanSpd;
      doc["timerRunning"] = timerRunning;
      doc["dayCount"] = dayCount;
      doc["startDate"] = startDateStr;
      doc["fanMode"] = fanAutoMode;

      String espString;
      serializeJson(doc, espString);

      webSocket.broadcastTXT(espString);  // Broadcast ke satu jaringan
      Serial.println("Data send: " + espString);
    } else {
      Serial.println("Gagal membaca data dari sensor!");
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
      if (!fanAutoMode) {  // Hanya ubah kecepatan jika tidak dalam mode otomatis
        fanSpd = message.substring(4).toInt();
        fanSpd = constrain(fanSpd, 0, 255);
        analogWrite(PWMPIN, fanSpd);
        Serial.printf("Set fan : %d\n", fanSpd);
      }
    } else if (message == "START_TIMER") {
      startTimer();
    } else if (message == "STOP_TIMER") {
      stopTimer();
    } else if (message == "RESET_TIMER") {
      resetTimer();
    } else if (message == "FAN_AUTO") {
      fanAutoMode = true;
      Serial.println("Fan mode: AUTO");
    } else if (message == "FAN_MANUAL") {
      fanAutoMode = false;
      Serial.println("Fan mode: MANUAL");
    }
  }
}

void startTimer() {
  if (!timerRunning) {
    // Pastikan RTC valid
    if (!Rtc.IsDateTimeValid()) {
      Serial.println("RTC tidak valid saat memulai timer!");
      RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
      Rtc.SetDateTime(compiled);
      timerStartDate = compiled;
    } else {
      timerStartDate = Rtc.GetDateTime();
    }
    
    timerRunning = true;
    dayCount = 0;
    
    // Simpan Mode
    preferences.begin("timer", false);
    preferences.putBool("running", true);
    preferences.putUInt("startTime", timerStartDate.Epoch32Time());
    preferences.end();
  }
}

void stopTimer() {
  if (timerRunning) {
    timerRunning = false;
    
    // Save final state to preferences
    preferences.begin("timer", false);
    preferences.putBool("running", false);
    // Tetap simpan waktu mulai untuk referensi
    preferences.end();
    
    Serial.println("Timer dihentikan");
  }
}

void resetTimer() {
  timerRunning = false;
  dayCount = 0;
  
  // Clear preferences
  preferences.begin("timer", false);
  preferences.clear();
  preferences.end();
  
  Serial.println("Timer reset");
}

void fanAuto(float temperature) {
  if (fanAutoMode) {
    // Jika suhu di atas threshold, set kipas ke kecepatan maksimum
    if (temperature >= fanAutoThreshold) {
      fanSpd = 255;
    } else {
      // Di bawah threshold, atur kecepatan berdasarkan suhu
      // Misal: 30°C = 100, 35°C = 180, 38°C = 255
      fanSpd = map(constrain(temperature * 10, 300, 380), 300, 380, 100, 255);
    }
    
    // Terapkan kecepatan kipas
    analogWrite(PWMPIN, fanSpd);
    Serial.printf("Fan Auto Mode");
  }
}
