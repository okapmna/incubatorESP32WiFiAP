#pragma once

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <AutoPID.h>
#include "secret.h"

// Pin Define
#define DHTPIN 18
#define DHTTYPE DHT22
#define FAN_PWM_PIN 19
#define HEATER_PWM_PIN 15
#define RELAY_HUM_PIN 4
#define SDA_PIN 21
#define SCL_PIN 22

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
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
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
