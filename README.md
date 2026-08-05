# Smart EGG Incubator ESP32 - MQTT (IoT)

An automatic egg incubator control system based on the ESP32 with MQTT (IoT) connectivity. Temperature and humidity data are sent to an MQTT broker in real time. You can monitor and control the incubator from anywhere using dashboards like [Unimon-dashboard](https://github.com/okapmna/unimon-dashboard.git) or Node-RED.

## Features

- WiFiManager: easy WiFi setup without hardcoded credentials. On first boot, connect to the `ESP32_Incubator_AP` access point to enter your SSID and password.
- Internet access: monitor and control from anywhere.
- MQTT pub/sub: sends temperature and humidity data to the broker in real time.
- OLED menu with rotary encoder: set target temperature and humidity directly on the device.
- Automatic control:
  - PID heater: keeps heater PWM at the target temperature.
  - Humidifier: relay turns on and off automatically based on target humidity.
  - Fan: always on (adjustable).
- Status sync: target temperature and humidity stay in sync with the dashboard.
- NVS (Preferences): target settings are saved permanently in memory.

## Hardware and Components

- ESP32 development board
- SHT30 temperature and humidity sensor (I2C)
- SSD1306 0.96" OLED display (I2C)
- 16x2 I2C LCD
- Rotary encoder with button
- AOD4148 MOSFET (heater PWM control)
- L298N motor driver (fan PWM control)
- Relay module (humidifier control)
- 5V / 12V power supply

## Pin Configuration

| Component | ESP32 GPIO | Description |
| :--- | :--- | :--- |
| HEATER | GPIO 18 | Heater PWM (MOSFET) |
| FAN | GPIO 19 | Fan PWM |
| RELAY_HUM | GPIO 12 | Humidifier relay |
| SDA | GPIO 21 | I2C data (OLED, LCD, SHT30) |
| SCL | GPIO 22 | I2C clock (OLED, LCD, SHT30) |
| Rotary CLK | GPIO 25 | Encoder |
| Rotary DT | GPIO 26 | Encoder |
| Rotary SW | GPIO 27 | Encoder button |

## Running with PlatformIO

The project follows the standard PlatformIO layout.

```bash
# Install dependencies and build
pio run

# Upload to ESP32
pio run -t upload

# Serial monitor
pio monitor
```

WiFi is configured with WiFiManager on first boot. MQTT credentials are stored in `include/secret.h`. Copy `include/secret-example.h` and fill in the server, port, username, password, and topics.

## Running with Arduino IDE

For the classic Arduino IDE sketch version, use the `arduino-ide` branch.

1. `git checkout arduino-ide`
2. Open `firmware/incubator_esp32_mqtt/incubator_esp32_mqtt.ino` in the Arduino IDE.
3. Install the following libraries via the Library Manager:
   - ArduinoJson (bblanchon)
   - PubSubClient (knolleary)
   - WiFiManager (tzapu)
   - LiquidCrystal I2C
   - Adafruit SHT31 Library
   - Adafruit GFX Library
   - Adafruit SSD1306
   - AutoPID
4. Copy `secret-example.h` to `secret.h` and fill in the MQTT credentials.
5. Select ESP32 Dev Module and upload.

## MQTT Topics

| Topic | Direction | Content |
| :--- | :--- | :--- |
| `incubator/xx/data` | Publish | Sensor data (temperature, humidity) |
| `incubator/xx/con` | Subscribe | Control commands |

Supported commands on the `con` topic:

- Send `dev_getinfo` to get the current target values.
- Send JSON to update targets, for example:
  ```json
  { "target_temp": 38.5, "target_hum": 65 }
  ```
  Target values are saved to NVS automatically.

> Replace `xx` in the topics with your device ID.

## Old Code (WiFi AP)

The old WiFi Access Point version of the incubator (without internet) is archived in `old_code/inkubatorAP.ino` and is no longer developed.

## Schematic and Documentation

Incubator schematic:
<br>
<img width="800" alt="Incubator Schematic" src="schematics/skematik1.png" />

Device photo:
<br>
<img width="800" alt="Incubator Real Picture" src="images/incubator32IoT.jpeg" />

## Contributors

- Oka Pmna - [@okapmna](https://github.com/okapmna)
- IDA BAGUS WILLI PARMITA - [@WILIOP-666](https://github.com/WILIOP-666)
