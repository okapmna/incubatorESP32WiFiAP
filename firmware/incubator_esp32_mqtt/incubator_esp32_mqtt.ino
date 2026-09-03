// ═══════════════════════════════════════════════════════════════════
//  Incubator ESP32 — Single File Firmware
//  Display : ST7735S 1.8" TFT  Landscape 160×128
//  Sensor  : SHT30
//  Control : PID Heater  |  Relay Humidifier  |  Fan PWM  |  Servo
//  Network : WiFiManager + MQTT (TLS)
// ═══════════════════════════════════════════════════════════════════

// ─── Libraries ────────────────────────────────────────────────────
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <AutoPID.h>
#include "secret.h"

// ─── Pin Define ───────────────────────────────────────────────────
#define HEATER_PWM_PIN  18
#define FAN_PWM_PIN     19
#define RELAY_HUM_PIN   12
#define SERVO_PIN        4   // Dipindah dari 13 ke 4 karena pin 13 dipakai TFT_MOSI

// TFT ST7735S SPI
#define TFT_SCLK        14
#define TFT_MOSI        13   // NOTE: shared pin with SERVO_PIN only if SPI HW used
#define TFT_RST         17
#define TFT_DC          16
#define TFT_CS           5

// Rotary Encoder
#define ROTARY_CLK_PIN  25
#define ROTARY_DT_PIN   26
#define ROTARY_SW_PIN   27

// ─── TFT Layout (Landscape: 160×128) ──────────────────────────────
//
//  y=  0 ┌──────────────────────────────────────────┐
//        │  INKUBATOR                    WIFI:OK    │  Topbar   h=14
//  y= 14 ├──────────────────────────────────────────┤
//        │  SUHU            │  KELEMBAPAN           │
//        │  37.2 °C         │  65.1 %               │  Sensor   h=64
//        │  Set: 37.0°C     │  Set: 60%             │
//  y= 78 ├──────────────────────────────────────────┤
//        │  > SET SUHU          37.0°C              │
//        │    SET LEMBAB        60%                 │  Menu     h=40
//        │    FAN SPEED         100%                │
//        │    SERVO             SWING               │
//  y=118 │  Putar=pilih  Tekan=atur                 │  Hint     h=10
//  y=128 └──────────────────────────────────────────┘
//
#define TFT_W           160
#define TFT_H           128
#define ROW_TOPBAR        0
#define ROW_SENSOR       14
#define ROW_MENU         78
#define ROW_HINT        118

// ── Color palette ───────────────────────────────────────────────
#define C_BG            ST77XX_BLACK
// Dividers — plain dim white
#define C_DIV_HORIZ     0x4208   // dim white (kusam)
#define C_DIV_VERT      0x4208
#define C_DIV_TOPBAR    0x4208
#define C_DIV_MENU      0x4208

// Sensor zone
#define C_LABEL         0x867D   // soft lavender-grey
#define C_LABEL_SUHU    0xFEA0   // warm orange for SUHU label
#define C_LABEL_KELEMB  0x87FC   // cool teal for KELEMBAPAN label
#define C_VALUE         ST77XX_WHITE
#define C_UNIT          ST77XX_CYAN
#define C_SET           ST77XX_YELLOW

// Topbar
#define C_TITLE         ST77XX_WHITE
#define C_TITLE_BG      0x1082   // subtle dark bar behind title
#define C_WIFI_OK       0x07E0   // bright green
#define C_WIFI_AP       0xFBE0   // amber/yellow
#define C_WIFI_DC       0xF800   // red

// Menu zone
#define C_SELECTED      ST77XX_YELLOW
#define C_SEL_BG        0x1082   // dark highlight behind selected row
#define C_DIMMED        0x528A   // muted grey for unselected values
#define C_NAV_LABEL     0xC618   // light silver for nav labels
#define C_EDIT_VAL      0x07FF   // bright cyan for edit value
#define C_CONFIRM_VAL   0xFD20   // orange for confirm

// Hint bar
#define C_HINT          0x5AEB   // cool blue-grey
#define C_HINT_BG       0x0008   // very subtle dark blue tint

// ─── PID Config ───────────────────────────────────────────────────
#define KP 15.0
#define KI  0.5
#define KD 20.0

// ─── Servo Config ─────────────────────────────────────────────────
#define SERVO_MIN_ANGLE   35.0
#define SERVO_MAX_ANGLE  145.0
#define SERVO_MIN_DUTY   1638
#define SERVO_MAX_DUTY   8192
#define SERVO_MODE_JADWAL  0
#define SERVO_MODE_SWING   1

// ─── Global State ─────────────────────────────────────────────────
double target_temp      = 37.0;
double target_hum       = 60.0;
int    target_fan_speed = 100;     // 0–100 %
double current_temp     = 0.0;
double current_hum      = 0.0;
double heater_pwm_value = 0.0;

int   servo_mode            = SERVO_MODE_JADWAL;
int   servo_interval_hours  = 3;
bool  servo_direction_cw    = true;
float current_servo_pos     = 0.0;
float target_servo_pos      = 0.0;
unsigned long last_servo_mode_change = 0;
unsigned long last_servo_update_time = 0;

volatile int  encoderValue  = 0;
volatile bool buttonPressed = false;

// ─── Timing ───────────────────────────────────────────────────────
unsigned long lastSensorRead            = 0;
unsigned long lastMqttPublish           = 0;
unsigned long lastWifiCheck             = 0;
unsigned long lastMqttReconnectAttempt  = 0;

// ─── Display Cache (anti-flicker) ─────────────────────────────────
// Each cached var stores last-drawn value; zone only redraws on change
struct Cache {
  float  temp     = -999;
  float  hum      = -999;
  float  setTemp  = -999;
  float  setHum   = -999;
  int    wifiSt   = -1;   // 0=DC 1=OK 2=AP
  // menu cache
  int    menuState  = -1;
  int    subIdx     = -1;
  // anti-flicker: row-level cache
  int    lastNavSel = -1;     // last highlighted row in NAVIGATE (-1=none)
  char   navVals[4][16] = {}; // per-row value strings for change detection
  char   editStr[16] = {};    // value string for EDIT/CONFIRM modes
} cache;

// ─── Objects ──────────────────────────────────────────────────────
Adafruit_ST7735  tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_SHT31   sht30;
WiFiClientSecure espClient;
PubSubClient     client(espClient);
Preferences      preferences;
WiFiManager      wm;
AutoPID          myPID(&current_temp, &target_temp, &heater_pwm_value, 0, 255, KP, KI, KD);

// ─── Menu State ───────────────────────────────────────────────────
enum MenuState {
  STATE_NAVIGATE,
  STATE_EDIT,
  STATE_SERVO_SUBMENU,
  STATE_SERVO_EDIT_JADWAL,
  STATE_CONFIRM
};
MenuState menuState      = STATE_NAVIGATE;
int currentMenuIndex     = 0;   // 0=Temp 1=Hum 2=Fan 3=Servo
int currentSubmenuIndex  = 0;
const int maxMenuItems   = 4;
int lastEncoderValue     = 0;

// ═══════════════════════════════════════════════════════════════════
//  SERVO
// ═══════════════════════════════════════════════════════════════════

uint32_t angleToDuty(float angle) {
  float phy = SERVO_MIN_ANGLE + (angle / 180.0f) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
  return SERVO_MIN_DUTY + (uint32_t)((phy / 180.0f) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY));
}

void setupServo() {
  ledcAttach(SERVO_PIN, 50, 16);
  servo_mode           = preferences.getInt("s_mode", SERVO_MODE_JADWAL);
  servo_interval_hours = preferences.getInt("s_int",  3);
  servo_direction_cw   = preferences.getBool("s_dir", true);
  if (servo_mode == SERVO_MODE_JADWAL) {
    current_servo_pos = servo_direction_cw ? 180.0f : 0.0f;
    target_servo_pos  = current_servo_pos;
  } else {
    current_servo_pos = 0.0f;
    target_servo_pos  = 180.0f;
  }
  ledcWrite(SERVO_PIN, angleToDuty(current_servo_pos));
  last_servo_mode_change = millis();
}

void updateServo() {
  unsigned long now = millis();
  if (servo_mode == SERVO_MODE_SWING) {
    if (now - last_servo_update_time >= 30) {
      last_servo_update_time = now;
      current_servo_pos += servo_direction_cw ? 1.0f : -1.0f;
      if (current_servo_pos >= 180.0f) { current_servo_pos = 180.0f; servo_direction_cw = false; }
      if (current_servo_pos <=   0.0f) { current_servo_pos =   0.0f; servo_direction_cw = true;  }
      ledcWrite(SERVO_PIN, angleToDuty(current_servo_pos));
    }
  } else {
    unsigned long intv = (unsigned long)servo_interval_hours * 3600UL * 1000UL;
    if (now - last_servo_mode_change >= intv) {
      last_servo_mode_change = now;
      servo_direction_cw = !servo_direction_cw;
      preferences.putBool("s_dir", servo_direction_cw);
      target_servo_pos = servo_direction_cw ? 180.0f : 0.0f;
    }
    if (fabs(current_servo_pos - target_servo_pos) > 0.01f && now - last_servo_update_time >= 30) {
      last_servo_update_time = now;
      current_servo_pos += (current_servo_pos < target_servo_pos) ? 1.0f : -1.0f;
      current_servo_pos = constrain(current_servo_pos, 0.0f, 180.0f);
      ledcWrite(SERVO_PIN, angleToDuty(current_servo_pos));
    }
  }
}

// ═══════════════════════════════════════════════════════════════════
//  DISPLAY — Partial-update system (zero fillScreen in loop)
// ═══════════════════════════════════════════════════════════════════

// ── Helpers ──────────────────────────────────────────────────────
void drawZoneText(int x, int y, int zoneW, int zoneH,
                  const char* txt, uint16_t color, uint8_t sz,
                  uint16_t bg = C_BG) {
  tft.fillRect(x, y, zoneW, zoneH, bg);
  tft.setTextSize(sz);
  tft.setTextColor(color);
  tft.setCursor(x, y);
  tft.print(txt);
}

void drawZoneTextRight(int x, int y, int zoneW, int zoneH,
                       const char* txt, uint16_t color, uint8_t sz,
                       uint16_t bg = C_BG) {
  tft.fillRect(x, y, zoneW, zoneH, bg);
  tft.setTextSize(sz);
  tft.setTextColor(color);
  int tw = strlen(txt) * 6 * sz;
  tft.setCursor(x + zoneW - tw, y);
  tft.print(txt);
}

// ── Chrome: drawn ONCE at boot ─────────────────────────────────
void drawChrome() {
  tft.fillScreen(C_BG);

  // Dividers — dim white, 2px
  tft.fillRect(0, 12, TFT_W, 2, C_DIV_TOPBAR);
  tft.fillRect(0, ROW_MENU - 2, TFT_W, 2, C_DIV_MENU);
  tft.fillRect(79, ROW_SENSOR, 2, ROW_MENU - ROW_SENSOR - 2, C_DIV_VERT);

  // Static sensor labels with distinct colors per column
  tft.setTextSize(1);
  tft.setTextColor(C_LABEL_SUHU);
  tft.setCursor(4, ROW_SENSOR + 4);
  tft.print("SUHU");
  tft.setTextColor(C_LABEL_KELEMB);
  tft.setCursor(84, ROW_SENSOR + 4);
  tft.print("KELEMBAPAN");
}

// ── Topbar ────────────────────────────────────────────────────
void updateTopbar() {
  int st = 0;
  if      (WiFi.status() == WL_CONNECTED)   st = 1;
  else if (wm.getConfigPortalActive())      st = 2;
  if (st == cache.wifiSt) return;
  cache.wifiSt = st;

  // Subtle dark bar behind title
  tft.fillRect(0, ROW_TOPBAR, TFT_W, 13, C_TITLE_BG);
  tft.setTextSize(1);
  tft.setTextColor(C_TITLE);
  tft.setCursor(2, 3);
  tft.print("INKUBATOR");

  const char* wl; uint16_t wc;
  if      (st == 1) { wl = "WIFI:OK"; wc = C_WIFI_OK; }
  else if (st == 2) { wl = "WIFI:AP"; wc = C_WIFI_AP; }
  else              { wl = "WIFI:DC"; wc = C_WIFI_DC; }
  drawZoneTextRight(TFT_W - 52, 3, 50, 9, wl, wc, 1);
}

// ── Sensor Zone ───────────────────────────────────────────────
void updateSensorZone() {
  bool changed = false;

  // ── Suhu (left) — warm orange tones ──
  if (fabs(current_temp - cache.temp) >= 0.05f) {
    cache.temp = current_temp;
    char buf[12]; snprintf(buf, sizeof(buf), "%.1f", current_temp);
    tft.fillRect(2, ROW_SENSOR + 22, 76, 18, C_BG);
    tft.setTextSize(2); tft.setTextColor(0xFD20);  // orange value
    tft.setCursor(4, ROW_SENSOR + 22);
    tft.print(buf);
    tft.setTextSize(1); tft.setTextColor(0xFE60);  // light orange unit
    tft.print(" \xF7""C");
    changed = true;
  }
  if (fabs(target_temp - cache.setTemp) >= 0.05f) {
    cache.setTemp = target_temp;
    char buf[12]; snprintf(buf, sizeof(buf), "Set:%.1f\xF7""C", target_temp);
    tft.fillRect(2, ROW_SENSOR + 46, 76, 10, C_BG);
    tft.setTextSize(1); tft.setTextColor(C_SET);
    tft.setCursor(4, ROW_SENSOR + 46);
    tft.print(buf);
    changed = true;
  }

  // ── Kelembapan (right) — cool blue tones ──
  if (fabs(current_hum - cache.hum) >= 0.3f) {
    cache.hum = current_hum;
    char buf[12]; snprintf(buf, sizeof(buf), "%.1f", current_hum);
    tft.fillRect(81, ROW_SENSOR + 22, 77, 18, C_BG);
    tft.setTextSize(2); tft.setTextColor(0x041F);  // bright blue value
    tft.setCursor(84, ROW_SENSOR + 22);
    tft.print(buf);
    tft.setTextSize(1); tft.setTextColor(0x87FF);  // light blue unit
    tft.print(" %");
    changed = true;
  }
  if (fabs(target_hum - cache.setHum) >= 0.3f) {
    cache.setHum = target_hum;
    char buf[12]; snprintf(buf, sizeof(buf), "Set:%d%%", (int)target_hum);
    tft.fillRect(81, ROW_SENSOR + 46, 77, 10, C_BG);
    tft.setTextSize(1); tft.setTextColor(C_SET);
    tft.setCursor(84, ROW_SENSOR + 46);
    tft.print(buf);
    changed = true;
  }
}

// ── Menu Zone ─────────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════
//  Anti-Flicker Helpers — row-level drawing, no full-zone fillRect
// ═══════════════════════════════════════════════════════════════════
static const char* const NAV_LABELS[] = {"SET SUHU", "SET LEMBAB", "FAN SPEED", "SERVO"};
static const char* const SERVO_OPTS[] = {"JADWAL", "SWING", "KEMBALI"};

void formatMenuVal(int idx, char* buf, size_t len) {
  if      (idx == 0) snprintf(buf, len, "%.1f\xF7""C", target_temp);
  else if (idx == 1) snprintf(buf, len, "%d%%", (int)target_hum);
  else if (idx == 2) snprintf(buf, len, "%d%%", target_fan_speed);
  else               snprintf(buf, len, "%s", servo_mode == SERVO_MODE_SWING ? "SWING" : "JADWAL");
}

void drawNavRow(int idx, bool sel) {
  int y = ROW_MENU + 2 + idx * 10;
  tft.fillRect(0, y - 1, TFT_W, 10, sel ? C_SEL_BG : C_BG);
  tft.setTextSize(1);
  tft.setCursor(4, y);
  tft.setTextColor(sel ? C_SELECTED : C_NAV_LABEL);
  char row[32];
  snprintf(row, sizeof(row), "%s %s", sel ? ">" : " ", NAV_LABELS[idx]);
  tft.print(row);
  char val[16];
  formatMenuVal(idx, val, sizeof(val));
  int tw = strlen(val) * 6;
  tft.setCursor(TFT_W - tw - 2, y);
  tft.setTextColor(sel ? C_VALUE : C_DIMMED);
  tft.print(val);
}

void drawServoOptRow(int idx, bool sel) {
  int y = ROW_MENU + 14 + idx * 10;
  tft.fillRect(0, y - 1, TFT_W, 10, sel ? C_SEL_BG : C_BG);
  tft.setTextSize(1);
  tft.setCursor(8, y);
  tft.setTextColor(sel ? C_SELECTED : C_NAV_LABEL);
  tft.print(sel ? "> " : "  ");
  tft.print(SERVO_OPTS[idx]);
}

void drawHint(const char* txt) {
  tft.fillRect(0, ROW_HINT, TFT_W, 10, C_HINT_BG);
  tft.setTextSize(1); tft.setTextColor(C_HINT);
  tft.setCursor(2, ROW_HINT + 1);
  tft.print(txt);
}

void drawEditScreen(const char* label, const char* valStr, uint16_t valColor, const char* hintTxt) {
  tft.fillRect(0, ROW_MENU, TFT_W, 4, C_BG);
  tft.setTextSize(1); tft.setTextColor(C_UNIT);
  tft.setCursor(4, ROW_MENU + 2);
  tft.print(label);
  tft.fillRect(0, ROW_MENU + 14, TFT_W, 30, C_BG);
  int tw = strlen(valStr) * 18;
  int xc = (TFT_W - tw) / 2; if (xc < 2) xc = 2;
  tft.setTextSize(3); tft.setTextColor(valColor);
  tft.setCursor(xc, ROW_MENU + 14);
  tft.print(valStr);
  drawHint(hintTxt);
}

// ── Menu zone with incremental row-level updates ───────────────
void drawMenuZone() {
  bool stateChanged = ((int)menuState != cache.menuState);
  if (stateChanged) {
    cache.lastNavSel = -1;
    cache.editStr[0] = '\0';
  }

  if (menuState == STATE_NAVIGATE) {
    int newSel = currentMenuIndex;
    if (cache.lastNavSel == -1) {
      for (int i = 0; i < maxMenuItems; i++) {
        drawNavRow(i, i == newSel);
        formatMenuVal(i, cache.navVals[i], sizeof(cache.navVals[i]));
      }
      drawHint("Putar=pilih  Tekan=atur");
    } else {
      if (newSel != cache.lastNavSel) {
        drawNavRow(cache.lastNavSel, false);
        drawNavRow(newSel, true);
      }
      for (int i = 0; i < maxMenuItems; i++) {
        char nv[16];
        formatMenuVal(i, nv, sizeof(nv));
        if (strcmp(nv, cache.navVals[i]) != 0) {
          strcpy(cache.navVals[i], nv);
          drawNavRow(i, (i == newSel));
        }
      }
    }
    cache.lastNavSel = newSel;

  } else if (menuState == STATE_SERVO_SUBMENU) {
    if (stateChanged) {
      tft.fillRect(0, ROW_MENU, TFT_W, 4, C_BG);
      tft.setTextSize(1); tft.setTextColor(C_UNIT);
      tft.setCursor(4, ROW_MENU + 2);
      tft.print("MODE SERVO:");
      cache.subIdx = -1;
      drawHint("Putar=pilih  Tekan=pilih");
    }
    if (currentSubmenuIndex != cache.subIdx) {
      int oldIdx = cache.subIdx;
      if (oldIdx >= 0 && oldIdx < 3) drawServoOptRow(oldIdx, false);
      drawServoOptRow(currentSubmenuIndex, true);
      cache.subIdx = currentSubmenuIndex;
    }

  } else if (menuState == STATE_SERVO_EDIT_JADWAL) {
    char vb[16]; snprintf(vb, sizeof(vb), "%d JAM", servo_interval_hours);
    if (strcmp(vb, cache.editStr) != 0) {
      strcpy(cache.editStr, vb);
      drawEditScreen("INTERVAL JADWAL:", vb, C_EDIT_VAL, "Putar=ubah   Tekan=konfirm");
    }

  } else if (menuState == STATE_EDIT) {
    const char* labels[] = {"ATUR SUHU", "ATUR LEMBAB", "FAN SPEED", ""};
    char vb[16];
    if      (currentMenuIndex == 0) snprintf(vb, 16, "%.1f\xF7""C", target_temp);
    else if (currentMenuIndex == 1) snprintf(vb, 16, "%d%%", (int)target_hum);
    else                            snprintf(vb, 16, "%d%%", target_fan_speed);
    if (strcmp(vb, cache.editStr) != 0) {
      strcpy(cache.editStr, vb);
      drawEditScreen(labels[currentMenuIndex], vb, C_EDIT_VAL, "Putar=ubah   Tekan=konfirm");
    }

  } else if (menuState == STATE_CONFIRM) {
    char vb[16];
    if      (currentMenuIndex == 0) snprintf(vb, 16, "%.1f\xF7""C", target_temp);
    else if (currentMenuIndex == 1) snprintf(vb, 16, "%d%%", (int)target_hum);
    else if (currentMenuIndex == 2) snprintf(vb, 16, "%d%%", target_fan_speed);
    else snprintf(vb, 16, "%s", servo_mode == SERVO_MODE_SWING ? "SWING" : "JADWAL");
    if (strcmp(vb, cache.editStr) != 0) {
      strcpy(cache.editStr, vb);
      drawEditScreen("SIMPAN PERUBAHAN?", vb, C_CONFIRM_VAL, "Tekan=simpan  Putar=batal");
    }
  }

  cache.menuState = (int)menuState;
}

// ── Master display update ──────────────────────────────────────
void updateDisplay() {
  updateTopbar();
  updateSensorZone();
  drawMenuZone();
}

// ═══════════════════════════════════════════════════════════════════
//  ROTARY ENCODER
// ═══════════════════════════════════════════════════════════════════

void IRAM_ATTR isr_encoder() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last > 4) {
    if (digitalRead(ROTARY_CLK_PIN) == digitalRead(ROTARY_DT_PIN)) encoderValue++;
    else                                                             encoderValue--;
    last = now;
  }
}

void IRAM_ATTR isr_button() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last > 200) { buttonPressed = true; last = now; }
}

void setupEncoder() {
  pinMode(ROTARY_CLK_PIN, INPUT_PULLUP);
  pinMode(ROTARY_DT_PIN,  INPUT_PULLUP);
  pinMode(ROTARY_SW_PIN,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK_PIN), isr_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_SW_PIN),  isr_button,  FALLING);
}

// ═══════════════════════════════════════════════════════════════════
//  MENU LOGIC
// ═══════════════════════════════════════════════════════════════════

void saveAll() {
  preferences.putDouble("t_temp", target_temp);
  preferences.putDouble("t_hum",  target_hum);
  preferences.putInt("t_fan",     target_fan_speed);
  preferences.putInt("s_mode",    servo_mode);
  preferences.putInt("s_int",     servo_interval_hours);
  preferences.putBool("s_dir",    servo_direction_cw);
}

void handleMenu() {
  // ── Button ──────────────────────────────────────────────────
  if (buttonPressed) {
    buttonPressed = false;
    if (menuState == STATE_NAVIGATE) {
      if (currentMenuIndex == 3) {
        menuState = STATE_SERVO_SUBMENU;
        currentSubmenuIndex = 0;
      } else {
        menuState = STATE_EDIT;
      }
    } else if (menuState == STATE_EDIT) {
      menuState = STATE_CONFIRM;
    } else if (menuState == STATE_SERVO_SUBMENU) {
      if (currentSubmenuIndex == 0) {       // JADWAL
        servo_mode = SERVO_MODE_JADWAL;
        menuState  = STATE_SERVO_EDIT_JADWAL;
      } else if (currentSubmenuIndex == 1) { // SWING
        servo_mode = SERVO_MODE_SWING;
        menuState  = STATE_CONFIRM;
      } else {                              // KEMBALI
        menuState = STATE_NAVIGATE;
        currentMenuIndex = 3;
      }
    } else if (menuState == STATE_SERVO_EDIT_JADWAL) {
      menuState = STATE_CONFIRM;
    } else if (menuState == STATE_CONFIRM) {
      if (currentMenuIndex == 3) {
        target_servo_pos = servo_direction_cw ? 180.0f : 0.0f;
        last_servo_mode_change = millis();
      }
      saveAll();
      menuState = STATE_NAVIGATE;
    }
  }

  // ── Encoder rotation (threshold 2 pulses to debounce) ───────
  int delta = encoderValue - lastEncoderValue;
  if (abs(delta) >= 2) {
    int dir = (delta > 0) ? 1 : -1;
    lastEncoderValue = encoderValue;

    if (menuState == STATE_NAVIGATE) {
      currentMenuIndex = (currentMenuIndex + dir + maxMenuItems) % maxMenuItems;

    } else if (menuState == STATE_SERVO_SUBMENU) {
      currentSubmenuIndex = (currentSubmenuIndex + dir + 3) % 3;

    } else if (menuState == STATE_SERVO_EDIT_JADWAL) {
      servo_interval_hours = constrain(servo_interval_hours + dir, 1, 24);

    } else if (menuState == STATE_EDIT) {
      if      (currentMenuIndex == 0) target_temp      = constrain(target_temp      + dir * 0.1, 20.0, 50.0);
      else if (currentMenuIndex == 1) target_hum       = constrain(target_hum       + dir * 1.0, 30.0, 90.0);
      else if (currentMenuIndex == 2) target_fan_speed = constrain(target_fan_speed + dir * 5,   0,   100);
      // Invalidate sensor set-value cache so they redraw in navigate too
      cache.setTemp = -999; cache.setHum = -999;

    } else if (menuState == STATE_CONFIRM) {
      // Any rotation = back to EDIT
      if (currentMenuIndex == 3)
        menuState = (servo_mode == SERVO_MODE_SWING) ? STATE_SERVO_SUBMENU : STATE_SERVO_EDIT_JADWAL;
      else
        menuState = STATE_EDIT;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════
//  SENSOR & CONTROL
// ═══════════════════════════════════════════════════════════════════

void readSensorAndControl() {
  current_temp = sht30.readTemperature();
  current_hum  = sht30.readHumidity();
  if (isnan(current_temp) || isnan(current_hum)) {
    Serial.println("[SHT30] Read fail");
    return;
  }
  // PID Heater
  myPID.run();
  int pwm = (int)heater_pwm_value;
  if (current_temp < target_temp && pwm < 15) pwm = 120;
  ledcWrite(HEATER_PWM_PIN, pwm);
  // Fan
  int fan_pwm = map(target_fan_speed, 0, 100, 0, 255);
  ledcWrite(FAN_PWM_PIN, fan_pwm);
  // Humidifier
  if      (current_hum <= target_hum - 1.0) digitalWrite(RELAY_HUM_PIN, HIGH);
  else if (current_hum >= target_hum)        digitalWrite(RELAY_HUM_PIN, LOW);

  Serial.printf("[SHT30] T=%.1f H=%.1f | Heater PWM=%d Fan=%d%%\n",
                current_temp, current_hum, pwm, target_fan_speed);
}

// ═══════════════════════════════════════════════════════════════════
//  WIFI
// ═══════════════════════════════════════════════════════════════════

void setupWifi() {
  wm.setConfigPortalBlocking(false);
  if (wm.autoConnect("ESP32_Incubator_AP"))
    Serial.println("[WiFi] Connected at boot");
  else
    Serial.println("[WiFi] Config portal active");
}

void handleWifiCheck() {
  if (WiFi.status() != WL_CONNECTED && !wm.getConfigPortalActive()) {
    Serial.println("[WiFi] Disconnected, starting portal...");
    wm.startConfigPortal("ESP32_Incubator_AP");
  }
}

// ═══════════════════════════════════════════════════════════════════
//  MQTT
// ═══════════════════════════════════════════════════════════════════

void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("[MQTT] in: "); Serial.println(msg);

  if (msg == "dev_getinfo") {
    StaticJsonDocument<200> resp;
    resp["target_temp"] = target_temp;
    resp["target_hum"]  = target_hum;
    resp["fan_speed"]   = target_fan_speed;
    resp["servo_mode"]  = servo_mode;
    char buf[200]; serializeJson(resp, buf);
    client.publish(mqtt_topic_data, buf);
    return;
  }
  StaticJsonDocument<200> doc;
  if (!deserializeJson(doc, payload, length)) {
    if (doc.containsKey("target_temp"))  { target_temp      = doc["target_temp"]; cache.setTemp = -999; }
    if (doc.containsKey("target_hum"))   { target_hum       = doc["target_hum"];  cache.setHum  = -999; }
    if (doc.containsKey("fan_speed"))    { target_fan_speed = doc["fan_speed"];   }
    saveAll();
  }
}

void reconnect() {
  unsigned long now = millis();
  if (now - lastMqttReconnectAttempt < 5000) return;
  lastMqttReconnectAttempt = now;
  Serial.print("[MQTT] Connecting...");
  String cid = "ESP32-Inc-" + String(random(0xffff), HEX);
  if (client.connect(cid.c_str(), mqtt_user, mqtt_pass)) {
    Serial.println(" OK");
    client.subscribe(mqtt_topic_con);
  } else {
    Serial.print(" FAIL rc="); Serial.println(client.state());
  }
}

void publishSensorData() {
  if (WiFi.status() == WL_CONNECTED && client.connected() && !isnan(current_temp)) {
    StaticJsonDocument<200> doc;
    doc["temperature"]  = current_temp;
    doc["humidity"]     = current_hum;
    doc["fan_speed"]    = target_fan_speed;
    doc["servo_mode"]   = servo_mode;
    doc["servo_pos"]    = current_servo_pos;
    char buf[200]; serializeJson(doc, buf);
    client.publish(mqtt_topic_data, buf);
  }
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ═══════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial.println("[BOOT] Starting...");

  // TFT Init
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);      // Landscape
  tft.setTextWrap(false);
  tft.fillScreen(C_BG);
  tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(35, 58);
  tft.print("Initializing...");
  Serial.println("[TFT] OK");

  // Static chrome layout
  drawChrome();

  // Encoder
  setupEncoder();

  // PWM & Relay
  pinMode(RELAY_HUM_PIN, OUTPUT);
  ledcAttach(FAN_PWM_PIN,    5000, 8);
  ledcAttach(HEATER_PWM_PIN, 5000, 8);

  // PID
  myPID.setBangBang(0.5);

  // SHT30
  if (!sht30.begin(0x44)) Serial.println("[SHT30] NOT FOUND");
  else                     Serial.println("[SHT30] OK");

  // NVS
  preferences.begin("incubator", false);
  target_temp      = preferences.getDouble("t_temp", 37.0);
  target_hum       = preferences.getDouble("t_hum",  60.0);
  target_fan_speed = preferences.getInt("t_fan",     100);
  if (target_fan_speed > 100) target_fan_speed = 100;

  // Servo
  setupServo();

  // WiFi
  setupWifi();

  // MQTT
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Serial.println("[BOOT] Done.");
}

void loop() {
  wm.process();

  // Encoder & menu logic
  handleMenu();

  // Display partial update (no fillScreen)
  updateDisplay();

  // Servo movement (non-blocking, every 30ms internally)
  updateServo();

  unsigned long now = millis();

  // WiFi check every 5s
  if (now - lastWifiCheck >= 5000) {
    lastWifiCheck = now;
    handleWifiCheck();
  }

  // MQTT
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) reconnect();
    else                     client.loop();
  }

  // Sensor & control every 2s
  if (now - lastSensorRead >= 2000) {
    lastSensorRead = now;
    readSensorAndControl();
  }

  // Publish MQTT every 5s
  if (now - lastMqttPublish >= 5000) {
    lastMqttPublish = now;
    publishSensorData();
  }
}
