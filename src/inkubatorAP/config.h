#pragma once

#include <Arduino.h>
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
#include "secret.h"

#define IO_PIN 4
#define SCLK_PIN 5
#define CE_PIN 2
#define DHTPIN 23
#define DHTTYPE DHT11
#define RELAYPIN_1 18
#define PWMPIN 19

ThreeWire myWire(IO_PIN, SCLK_PIN, CE_PIN);
RtcDS1302<ThreeWire> Rtc(myWire);

Preferences preferences;

DHT dht(DHTPIN, DHTTYPE);

WiFiClient espClient;
PubSubClient client(espClient);

WebSocketsServer webSocket = WebSocketsServer(81);

int fanSpd = 180;
bool fanAutoMode = false;
float fanAutoThreshold = 38.0;
