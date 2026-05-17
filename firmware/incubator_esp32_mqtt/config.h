#pragma once

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
// #include <DHT.h>          // DHT22 dinonaktifkan, diganti AHT20
// #include <LiquidCrystal_I2C.h>  // LCD I2C 16x2 (opsional, diganti OLED)
#include <Adafruit_AHTX0.h>        // Sensor AHT20
#include <Adafruit_GFX.h>          // OLED graphics core
#include <Adafruit_SSD1306.h>      // OLED 0.96" driver
#include <Preferences.h>
#include <ArduinoJson.h>
#include <AutoPID.h>
#include "secret.h"

// Pin Define
// #define DHTPIN   18        // DHT22 dinonaktifkan
// #define DHTTYPE  DHT22     // DHT22 dinonaktifkan
#define HEATER_PWM_PIN  18   // Heater → Pin 18
#define FAN_PWM_PIN     19   // Fan    → Pin 19
#define RELAY_HUM_PIN   12   // Relay humidifier → Pin 12
#define SDA_PIN         21
#define SCL_PIN         22

// OLED Config
#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define OLED_ADDR   0x3C
#define OLED_RESET    -1   // Tidak pakai pin reset

// PID Config
#define KP 15.0
#define KI 0.5
#define KD 20.0

// Variabel Parameter Temp & Hum
double target_temp = 37.0;
double target_hum = 60.0;
double current_temp, current_hum;
double heater_pwm_value;

// Object Instances
// DHT dht(DHTPIN, DHTTYPE);           // DHT22 dinonaktifkan
// LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD I2C 16x2 (opsional)
Adafruit_AHTX0   aht;                  // Sensor AHT20
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET); // OLED 0.96"
WiFiClientSecure espClient;
PubSubClient client(espClient);
Preferences preferences;
WiFiManager wm;

AutoPID myPID(&current_temp, &target_temp, &heater_pwm_value, 0, 255, KP, KI, KD);

// Time Variables
unsigned long lastSensorRead = 0;
unsigned long lastMqttPublish = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttReconnectAttempt = 0;
