// ═══════════════════════════════════════════════════════════════════
//  Incubator ESP32 — Single File Firmware
//  Display : ST7735S 1.8" TFT  Portrait 128×160 (Hardware SPI / HSPI)
//  Sensor  : SHT30
//  Control : PID Heater  |  Relay Humidifier  |  Fan PWM  |  Servo
//  Network : WiFiManager + MQTT (TLS)  → jalan di task core 0
//  Encoder : Quadrature state-table (akurat, tanpa delay)
// ═══════════════════════════════════════════════════════════════════

// ─── Libraries ────────────────────────────────────────────────────
#include <SPI.h>
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
#define SERVO_PIN        4

// TFT ST7735S SPI (HSPI: SCLK=14, MOSI=13)
#define TFT_SCLK        14
#define TFT_MOSI        13
#define TFT_RST         17
#define TFT_DC          16
#define TFT_CS           5

// Rotary Encoder
#define ROTARY_CLK_PIN  25
#define ROTARY_DT_PIN   26
#define ROTARY_SW_PIN   27

// ─── Encoder Config ───────────────────────────────────────────────
#define ENC_STEPS_PER_DETENT 2      // KY-040 umum = 4. Jika 1 klik loncat 2 menu → 2 atau 1
#define ENC_REVERSE          false  // true bila arah putar terbalik
#define BTN_DEBOUNCE_MS      30

// ─── TFT Layout (Portrait: 128×160) ───────────────────────────────
//
//  y=  0 ┌────────────────────────┐
//        │  INKUBATOR   WIFI:OK   │  Topbar   h=14
//  y= 14 ├────────────────────────┤
//        │  SUHU                   │
//        │          37.2 °C        │  Sensor Suhu  h=40
//  y= 54 ╞════════════════════════╡  Divider
//  y= 56 ├────────────────────────┤
//        │  LEMBABAN              │
//        │            65.1 %      │  Sensor Kelemb h=40
//  y= 96 ╞════════════════════════╡  Divider
//  y= 98 ├────────────────────────┤
//        │  > SET SUHU    37.0°C  │
//        │    SET LEMBAB  60%     │  Menu     h=50
//        │    FAN SPEED   100%    │
//        │    SERVO       SWING   │
//  y=148 ├────────────────────────┤
//        │  Putar=pilih Tekan=atur│  Hint     h=12
//  y=160 └────────────────────────┘
//
#define TFT_W           128
#define TFT_H           160
#define ROW_TOPBAR        0
#define ROW_SENSOR_SUHU  14
#define ROW_SENSOR_KELEMB 56
#define ROW_MENU         98
#define ROW_HINT        148

// ── Color palette ───────────────────────────────────────────────
#define C_BG            ST77XX_BLACK
#define C_DIV_HORIZ     0x4208
#define C_DIV_VERT      0x4208
#define C_DIV_TOPBAR    0x4208
#define C_DIV_MENU      0x4208

#define C_LABEL         0x867D
#define C_LABEL_SUHU    0xFEA0
#define C_LABEL_KELEMB  0x87FC
#define C_VALUE         ST77XX_WHITE
#define C_UNIT          ST77XX_CYAN
#define C_SET           ST77XX_YELLOW

#define C_TITLE         ST77XX_WHITE
#define C_TITLE_BG      0x1082
#define C_WIFI_OK       0x07E0
#define C_WIFI_AP       0xFBE0
#define C_WIFI_DC       0xF800

#define C_SELECTED      ST77XX_YELLOW
#define C_SEL_BG        0x1082
#define C_DIMMED        0x528A
#define C_NAV_LABEL     0xC618
#define C_EDIT_VAL      0x07FF
#define C_CONFIRM_VAL   0xFD20

#define C_HINT          0x5AEB
#define C_HINT_BG       0x0008

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

double target_temp      = 37.0;
double target_hum       = 60.0;
int    target_fan_speed = 100;     // 0–100 %
double current_temp     = NAN;     // NAN = belum terbaca, UI tampil "--.-"
double current_hum      = NAN;
double heater_pwm_value = 0.0;

int   servo_mode            = SERVO_MODE_JADWAL;
int   servo_interval_hours  = 3;
bool  servo_direction_cw    = true;
float current_servo_pos     = 0.0;
float target_servo_pos      = 0.0;
unsigned long last_servo_mode_change = 0;
unsigned long last_servo_update_time = 0;

// ─── Encoder state (ISR) ──────────────────────────────────────────
volatile int8_t  encState = 0;
volatile int32_t encAccum = 0;
portMUX_TYPE     encMux   = portMUX_INITIALIZER_UNLOCKED;

// ─── Timing ───────────────────────────────────────────────────────
unsigned long lastSensorRead            = 0;
unsigned long lastSensorRetry           = 0;
unsigned long lastMqttReconnectAttempt  = 0;
unsigned long lastDisplayUpdate         = 0;
bool          displayDirty              = true;
#define       DISPLAY_COOLDOWN_MS       10
#define       SENSOR_READ_MS            2000
#define       SENSOR_RETRY_MS           5000
#define       MQTT_RECONNECT_MS         10000

// ─── Sensor state (non-blocking) ──────────────────────────────────
bool sensorOK        = false;
uint8_t sensorFails  = 0;

// ─── Display Cache (anti-flicker) ─────────────────────────────────
struct Cache {
  float  temp     = -999;
  float  hum      = -999;
  float  setTemp  = -999;
  float  setHum   = -999;
  int    wifiSt   = -1;   // 0=DC 1=OK 2=AP
  int    menuState  = -1;
  int    subIdx     = -1;
  int    lastNavSel = -1;
  char   navVals[4][16] = {};
  char   editStr[16] = {};
} cache;

// ─── Objects ──────────────────────────────────────────────────────
SPIClass         tftSPI(HSPI);
Adafruit_ST7735  tft = Adafruit_ST7735(&tftSPI, TFT_CS, TFT_DC, TFT_RST);
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

  tft.fillRect(0, 12, TFT_W, 2, C_DIV_TOPBAR);
  tft.fillRect(0, ROW_SENSOR_KELEMB - 2, TFT_W, 2, C_DIV_HORIZ);
  tft.fillRect(0, ROW_MENU - 2, TFT_W, 2, C_DIV_MENU);

  tft.setTextSize(1);
  tft.setTextColor(C_LABEL_SUHU);
  tft.setCursor(4, ROW_SENSOR_SUHU + 2);
  tft.print("SUHU");
  tft.setTextColor(C_LABEL_KELEMB);
  tft.setCursor(4, ROW_SENSOR_KELEMB + 2);
  tft.print("LEMBABAN");
}

// ── Topbar ────────────────────────────────────────────────────
void updateTopbar() {
  int st = 0;
  if      (WiFi.status() == WL_CONNECTED)   st = 1;
  else if (wm.getConfigPortalActive())      st = 2;
  if (st == cache.wifiSt) return;
  cache.wifiSt = st;
  displayDirty = true;

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

  // ── Suhu ──
  {
    bool invalid = isnan(current_temp);
    float cmp = invalid ? -9999.0f : (float)current_temp;
    if (fabs(cmp - cache.temp) >= 0.05f) {
      cache.temp = cmp;
      char buf[12];
      if (invalid) snprintf(buf, sizeof(buf), "--.-");
      else         snprintf(buf, sizeof(buf), "%.1f", current_temp);
      tft.fillRect(2, ROW_SENSOR_SUHU + 10, 124, 24, C_BG);
      tft.setTextSize(3); tft.setTextColor(0xFD20);
      int valW = strlen(buf) * 18;
      int unitW = 8;
      int totalW = valW + unitW;
      int startX = (TFT_W - totalW) / 2;
      tft.setCursor(startX, ROW_SENSOR_SUHU + 12);
      tft.print(buf);
      tft.setTextSize(1); tft.setTextColor(0xFE60);
      tft.setCursor(startX + valW, ROW_SENSOR_SUHU + 18);
      tft.print("\xF7""C");
      changed = true;
    }
  }

  // ── Kelembapan ──
  {
    bool invalid = isnan(current_hum);
    float cmp = invalid ? -9999.0f : (float)current_hum;
    if (fabs(cmp - cache.hum) >= 0.3f) {
      cache.hum = cmp;
      char buf[12];
      if (invalid) snprintf(buf, sizeof(buf), "--.-");
      else         snprintf(buf, sizeof(buf), "%.1f", current_hum);
      tft.fillRect(2, ROW_SENSOR_KELEMB + 10, 124, 24, C_BG);
      tft.setTextSize(3); tft.setTextColor(0x041F);
      int valW = strlen(buf) * 18;
      int unitW = 6;
      int totalW = valW + unitW;
      int startX = (TFT_W - totalW) / 2;
      tft.setCursor(startX, ROW_SENSOR_KELEMB + 12);
      tft.print(buf);
      tft.setTextSize(1); tft.setTextColor(0x87FF);
      tft.setCursor(startX + valW, ROW_SENSOR_KELEMB + 18);
      tft.print("%");
      changed = true;
    }
  }

  if (changed) {
    tft.fillRect(0, 12, TFT_W, 2, C_DIV_TOPBAR);
    tft.fillRect(0, ROW_SENSOR_KELEMB - 2, TFT_W, 2, C_DIV_HORIZ);
    tft.fillRect(0, ROW_MENU - 2, TFT_W, 2, C_DIV_MENU);
    displayDirty = true;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  Anti-Flicker Helpers — row-level drawing
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
  tft.setCursor(TFT_W - tw - 4, y);
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
  if (!sel) {
    tft.fillRect(50, y, TFT_W - 50, 10, C_BG);
  }
}

void drawHint(const char* txt) {
  tft.fillRect(0, ROW_HINT, TFT_W, 12, C_HINT_BG);
  tft.setTextSize(1); tft.setTextColor(C_HINT);
  tft.setCursor(2, ROW_HINT + 2);
  tft.print(txt);
}

void drawEditScreen(const char* label, const char* valStr, uint16_t valColor, const char* hintTxt) {
  tft.fillRect(0, ROW_MENU, TFT_W, ROW_HINT - ROW_MENU, C_BG);
  tft.setTextSize(1); tft.setTextColor(C_UNIT);
  tft.setCursor(4, ROW_MENU + 2);
  tft.print(label);
  int tw = strlen(valStr) * 18;
  int xc = (TFT_W - tw) / 2; if (xc < 2) xc = 2;
  tft.setTextSize(3); tft.setTextColor(valColor);
  tft.setCursor(xc, ROW_MENU + 20);
  tft.print(valStr);
  drawHint(hintTxt);
}

void drawEditValueOnly(const char* valStr, uint16_t valColor) {
  tft.fillRect(0, ROW_MENU + 14, TFT_W, 34, C_BG);
  int tw = strlen(valStr) * 18;
  int xc = (TFT_W - tw) / 2; if (xc < 2) xc = 2;
  tft.setTextSize(3); tft.setTextColor(valColor);
  tft.setCursor(xc, ROW_MENU + 20);
  tft.print(valStr);
}

// ── Menu zone with incremental row-level updates ───────────────
void drawMenuZone() {
  bool stateChanged = ((int)menuState != cache.menuState);
  if (stateChanged) {
    tft.fillRect(0, ROW_MENU, TFT_W, ROW_HINT - ROW_MENU, C_BG);
    cache.lastNavSel = -1;
    cache.subIdx = -1;
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
    if (stateChanged || cache.lastNavSel != -2) {
      tft.fillRect(0, ROW_MENU, TFT_W, 12, C_BG);
      tft.setTextSize(1); tft.setTextColor(C_UNIT);
      tft.setCursor(4, ROW_MENU + 2);
      tft.print("MODE SERVO:");
      cache.lastNavSel = -2;
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
    if (stateChanged) {
      strcpy(cache.editStr, vb);
      drawEditScreen("INTERVAL JADWAL:", vb, C_EDIT_VAL, "Putar=ubah   Tekan=konfirm");
    } else if (strcmp(vb, cache.editStr) != 0) {
      strcpy(cache.editStr, vb);
      drawEditValueOnly(vb, C_EDIT_VAL);
    }

  } else if (menuState == STATE_EDIT) {
    const char* labels[] = {"ATUR SUHU", "ATUR LEMBAB", "FAN SPEED", ""};
    char vb[16];
    if      (currentMenuIndex == 0) snprintf(vb, 16, "%.1f\xF7""C", target_temp);
    else if (currentMenuIndex == 1) snprintf(vb, 16, "%d%%", (int)target_hum);
    else                            snprintf(vb, 16, "%d%%", target_fan_speed);
    if (stateChanged) {
      strcpy(cache.editStr, vb);
      drawEditScreen(labels[currentMenuIndex], vb, C_EDIT_VAL, "Putar=ubah   Tekan=konfirm");
    } else if (strcmp(vb, cache.editStr) != 0) {
      strcpy(cache.editStr, vb);
      drawEditValueOnly(vb, C_EDIT_VAL);
    }

  } else if (menuState == STATE_CONFIRM) {
    char vb[16];
    if      (currentMenuIndex == 0) snprintf(vb, 16, "%.1f\xF7""C", target_temp);
    else if (currentMenuIndex == 1) snprintf(vb, 16, "%d%%", (int)target_hum);
    else if (currentMenuIndex == 2) snprintf(vb, 16, "%d%%", target_fan_speed);
    else snprintf(vb, 16, "%s", servo_mode == SERVO_MODE_SWING ? "SWING" : "JADWAL");
    if (stateChanged) {
      strcpy(cache.editStr, vb);
      drawEditScreen("SIMPAN PERUBAHAN?", vb, C_CONFIRM_VAL, "Tekan=simpan  Putar=batal");
    } else if (strcmp(vb, cache.editStr) != 0) {
      strcpy(cache.editStr, vb);
      drawEditValueOnly(vb, C_CONFIRM_VAL);
    }
  }

  cache.menuState = (int)menuState;
}

// ── Master display update ──────────────────────────────────────
void updateDisplay() {
  updateTopbar();
  drawMenuZone();
}

void updateSensorDisplay() {
  updateSensorZone();
}

// ═══════════════════════════════════════════════════════════════════
//  ROTARY ENCODER — quadrature state-table (Gray code)
//  Transisi ilegal (akibat bouncing) otomatis bernilai 0,
//  sehingga tidak butuh debounce berbasis waktu.
// ═══════════════════════════════════════════════════════════════════

void IRAM_ATTR isr_encoder() {
  static const int8_t T[16] = { 0,-1, 1, 0,
                                1, 0, 0,-1,
                               -1, 0, 0, 1,
                                0, 1,-1, 0 };
  portENTER_CRITICAL_ISR(&encMux);
  encState = ((encState << 2) |
              (digitalRead(ROTARY_CLK_PIN) << 1) |
               digitalRead(ROTARY_DT_PIN)) & 0x0F;
  encAccum += T[encState];
  portEXIT_CRITICAL_ISR(&encMux);
}

void setupEncoder() {
  pinMode(ROTARY_CLK_PIN, INPUT_PULLUP);
  pinMode(ROTARY_DT_PIN,  INPUT_PULLUP);
  pinMode(ROTARY_SW_PIN,  INPUT_PULLUP);
  encState = (digitalRead(ROTARY_CLK_PIN) << 1) | digitalRead(ROTARY_DT_PIN);
  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK_PIN), isr_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_DT_PIN),  isr_encoder, CHANGE);
}

// Jumlah "klik" (detent) sejak panggilan terakhir, bertanda +/-
int readEncoderSteps() {
  portENTER_CRITICAL(&encMux);
  int steps = encAccum / ENC_STEPS_PER_DETENT;
  encAccum -= steps * ENC_STEPS_PER_DETENT;   // sisa transisi dipertahankan
  portEXIT_CRITICAL(&encMux);
  return ENC_REVERSE ? -steps : steps;
}

// Tombol: polling + debounce, tanpa lockout panjang
bool readButton() {
  static bool lastRaw = HIGH, stable = HIGH;
  static unsigned long tChange = 0;
  bool raw = digitalRead(ROTARY_SW_PIN);
  if (raw != lastRaw) { lastRaw = raw; tChange = millis(); }
  if (millis() - tChange >= BTN_DEBOUNCE_MS && raw != stable) {
    stable = raw;
    if (stable == LOW) return true;
  }
  return false;
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
  if (readButton()) {
    displayDirty = true;
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

  // ── Encoder rotation: semua step diproses, tanpa threshold ──
  int steps = readEncoderSteps();
  if (steps != 0) {
    displayDirty = true;

    if (menuState == STATE_NAVIGATE) {
      currentMenuIndex = ((currentMenuIndex + steps) % maxMenuItems + maxMenuItems) % maxMenuItems;

    } else if (menuState == STATE_SERVO_SUBMENU) {
      currentSubmenuIndex = ((currentSubmenuIndex + steps) % 3 + 3) % 3;

    } else if (menuState == STATE_SERVO_EDIT_JADWAL) {
      servo_interval_hours = constrain(servo_interval_hours + steps, 1, 24);

    } else if (menuState == STATE_EDIT) {
      if (currentMenuIndex == 0)
        target_temp = constrain(round((target_temp + steps * 0.1) * 10.0) / 10.0, 20.0, 50.0);
      else if (currentMenuIndex == 1)
        target_hum = constrain(target_hum + steps * 1.0, 30.0, 90.0);
      else if (currentMenuIndex == 2)
        target_fan_speed = constrain(target_fan_speed + steps * 5, 0, 100);
      cache.setTemp = -999; cache.setHum = -999;

    } else if (menuState == STATE_CONFIRM) {
      // Rotasi apa pun = kembali ke EDIT
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
  int fan_pwm = map(target_fan_speed, 0, 100, 0, 255);
  ledcWrite(FAN_PWM_PIN, fan_pwm);

  if (!sensorOK) {
    unsigned long now = millis();
    if (now - lastSensorRetry < SENSOR_RETRY_MS) return;
    lastSensorRetry = now;
    if (!sht30.begin(0x44)) {
      Serial.println("[SHT30] NOT FOUND, retry nanti");
      return;
    }
    Serial.println("[SHT30] found (retry OK)");
    sensorOK = true;
    sensorFails = 0;
  }

  float t = sht30.readTemperature();
  float h = sht30.readHumidity();
  if (isnan(t) || isnan(h)) {
    Serial.println("[SHT30] Read fail");
    if (++sensorFails >= 3) {
      sensorOK = false;
      sensorFails = 0;
      lastSensorRetry = millis();
      heater_pwm_value = 0;
      ledcWrite(HEATER_PWM_PIN, 0);
      digitalWrite(RELAY_HUM_PIN, LOW);
    }
    return;
  }
  sensorFails = 0;
  current_temp = t;
  current_hum  = h;
  myPID.run();
  int pwm = (int)heater_pwm_value;
  if (current_temp < target_temp && pwm < 15) pwm = 120;
  ledcWrite(HEATER_PWM_PIN, pwm);
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
  wm.setConnectTimeout(5);
  wm.setConnectRetries(1);
  wm.setConfigPortalTimeout(180);
  wm.setWiFiAutoReconnect(true);
  if (wm.autoConnect("ESP32_Incubator_AP"))
    Serial.println("[WiFi] Connected at boot");
  else
    Serial.println("[WiFi] lanjut boot tanpa WiFi (portal non-blocking)");
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
  if (now - lastMqttReconnectAttempt < MQTT_RECONNECT_MS) return;
  lastMqttReconnectAttempt = now;
  if (WiFi.status() != WL_CONNECTED) return;
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
  if (!sensorOK) return;   // jangan kirim NAN
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
//  NETWORK TASK (core 0) — WiFi/MQTT/TLS tidak boleh menahan UI
// ═══════════════════════════════════════════════════════════════════

void networkTask(void* pv) {
  setupWifi();

  espClient.setInsecure();
  espClient.setTimeout(3);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setSocketTimeout(3);
  client.setKeepAlive(30);

  unsigned long now = millis();
  unsigned long lastWifi = now, lastPub = now;
  lastMqttReconnectAttempt = now - MQTT_RECONNECT_MS + 3000;  // coba MQTT 3 dtk lagi

  for (;;) {
    wm.process();
    now = millis();

    if (now - lastWifi >= 5000) { lastWifi = now; handleWifiCheck(); }

    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) reconnect();
      else                     client.loop();
    }

    if (now - lastPub >= 5000) { lastPub = now; publishSensorData(); }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ═══════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial.println("[BOOT] Starting...");

  // ── 1. NVS dulu → menu langsung punya nilai walau sensor/WiFi mati ──
  preferences.begin("incubator", false);
  target_temp      = preferences.getDouble("t_temp", 37.0);
  target_hum       = preferences.getDouble("t_hum",  60.0);
  target_fan_speed = preferences.getInt("t_fan",     100);
  if (target_fan_speed > 100) target_fan_speed = 100;

  // TFT Init (hardware SPI di HSPI: SCLK=14, MOSI=13)
  tftSPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);
  tft.initR(INITR_BLACKTAB);
  tft.setSPISpeed(27000000);      // turunkan ke 20000000 bila layar glitch
  tft.setRotation(0);             // Portrait
  tft.setTextWrap(false);
  tft.fillScreen(C_BG);
  tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(20, 75);
  tft.print("Initializing...");
  Serial.println("[TFT] OK");

  // Static chrome layout
  drawChrome();

  // Encoder
  setupEncoder();

  // PWM & Relay (failsafe: heater OFF, fan ikut setting)
  pinMode(RELAY_HUM_PIN, OUTPUT);
  digitalWrite(RELAY_HUM_PIN, LOW);
  ledcAttach(FAN_PWM_PIN,    5000, 8);
  ledcAttach(HEATER_PWM_PIN, 5000, 8);
  ledcWrite(HEATER_PWM_PIN, 0);
  ledcWrite(FAN_PWM_PIN, map(target_fan_speed, 0, 100, 0, 255));

  // PID
  myPID.setBangBang(0.5);

  // ── 2. UI langsung hidup, tampil "--.-" dulu tanpa nunggu sensor ──
  updateSensorDisplay();
  updateDisplay();

  // Servo (baca NVS di dalam)
  setupServo();

  // ── 3. Sensor: coba sekali saja, gagal → lanjut boot, retry di loop ──
  if (!sht30.begin(0x44)) {
    Serial.println("[SHT30] NOT FOUND at boot, lanjut (retry di loop)");
    sensorOK = false;
    lastSensorRetry = millis();
  } else {
    Serial.println("[SHT30] OK");
    sensorOK = true;
  }

  lastSensorRead = millis() - SENSOR_READ_MS + 500;  // baca pertama 0.5 dtk lagi

  // ── 4. WiFi + MQTT di task terpisah (core 0), loop() tetap di core 1 ──
  xTaskCreatePinnedToCore(networkTask, "net", 10240, NULL, 1, NULL, 0);

  Serial.println("[BOOT] Done. UI siap, sensor/WiFi/MQTT jalan background.");
}

void loop() {
  // Encoder & menu logic (prioritas utama, dipanggil tiap iterasi)
  handleMenu();

  // Sensor display (hanya redraw saat nilai berubah)
  updateSensorDisplay();

  // Display partial update
  unsigned long now = millis();
  if (displayDirty && (now - lastDisplayUpdate >= DISPLAY_COOLDOWN_MS)) {
    updateDisplay();
    lastDisplayUpdate = now;
    displayDirty = false;
  }

  // Servo movement (non-blocking, tiap 30ms internal)
  updateServo();

  // Sensor & control setiap 2 dtk (return cepat bila sensor mati)
  if (now - lastSensorRead >= SENSOR_READ_MS) {
    lastSensorRead = now;
    readSensorAndControl();
  }
}
