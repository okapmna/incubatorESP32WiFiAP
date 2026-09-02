#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <AutoPID.h>
#include "secret.h"

#define HEATER_PWM_PIN  18
#define FAN_PWM_PIN     19
#define RELAY_HUM_PIN   12
#define SDA_PIN         21
#define SCL_PIN         22

#define ROTARY_CLK_PIN  25
#define ROTARY_DT_PIN   26
#define ROTARY_SW_PIN   27

#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define OLED_ADDR   0x3C
#define OLED_RESET    -1

#define KP 15.0
#define KI 0.5
#define KD 20.0

double target_temp = 37.0;
double target_hum = 60.0;
double current_temp, current_hum;
double heater_pwm_value;

volatile int encoderValue = 0;
volatile bool buttonPressed = false;

LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_AHTX0   aht;
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
WiFiClientSecure espClient;
PubSubClient client(espClient);
Preferences preferences;
WiFiManager wm;

AutoPID myPID(&current_temp, &target_temp, &heater_pwm_value, 0, 255, KP, KI, KD);

unsigned long lastSensorRead = 0;
unsigned long lastMqttPublish = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttReconnectAttempt = 0;
