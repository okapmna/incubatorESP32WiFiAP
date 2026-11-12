/**
 * @file incubator_firmware.ino
 * @brief Firmware Inkubator ESP32 (Satu File)
 * 
 * Hardware: Layar ST7735S 128x160 portrait (HSPI), Sensor SHT30, Rotary Encoder KY-040, RTC DS1307
 * Kontrol: Pemanas (PID), Humidifier (Relay), Kipas (PWM), Pembalik Telur (Servo), LED Candling
 * Jaringan: WiFiManager + MQTT TLS (berjalan di Core 0 agar UI di Core 1 tidak macet)
 * Penyimpanan: Pengaturan tersimpan di NVS (Non-Volatile Storage)
 * RTC: DS1307 (I2C, bus yang sama dengan SHT30). Diset dari waktu kompilasi saat firmware
 *      pertama kali di-upload, lalu dikalibrasi ulang via NTP setiap 30 hari.
 * Candling: LED di pin 23, bisa dinyalakan dari menu LCD atau perintah MQTT.
 *           Ada auto-off agar lampu tidak memanaskan telur bila lupa dimatikan.
 *           Status LED TIDAK disimpan ke NVS (selalu mati setelah reboot).
 * Inkubasi: Pilih jenis telur (egg_profiles.h) -> suhu, kelembapan, dan target hari
 *           terisi otomatis, tetap bisa diubah manual. Hitungan hari memakai RTC dan
 *           disimpan di NVS sehingga tetap berjalan setelah mati listrik/reboot.
 * Lockdown: Beberapa hari terakhir sebelum tetas (kolom lock di egg_profiles.h) pemutaran
 *           telur dihentikan (servo diam di posisi tengah) dan setpoint suhu PID turun
 *           sebesar kolom drop. target_temp di menu tidak berubah; kembali normal otomatis
 *           bila inkubasi dihentikan.
 */

// ==============================================================================
// 1. LIBRARY & KONFIGURASI HARDWARE
// ==============================================================================
#include <SPI.h>
#include <Wire.h>
#include <time.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <RTClib.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <AutoPID.h>
#include "secret.h"
#include "icons.h"          // Bitmap ikon (WiFi) dipisah ke file tersendiri
#include "egg_profiles.h"   // Data parameter inkubasi per jenis telur

// --- Pin Aktuator ---
#define HEATER_PWM_PIN  18  // Pemanas (PWM)
#define FAN_PWM_PIN     19  // Kipas (PWM)
#define RELAY_HUM_PIN   12  // Humidifier (Relay)
#define SERVO_PIN        4  // Servo (PWM 50Hz)
#define LED_CANDLE_PIN  23  // LED candling telur

// --- Konfigurasi Candling ---
#define CANDLE_DEFAULT_SEC   120   // Auto-off default (detik)
#define CANDLE_MAX_SEC       3600  // Batas maksimum durasi (detik)
#define CANDLE_CAPTURE_SEC   30    // Auto-off saat foto kamera diambil dari dashboard

// --- Pin Layar TFT (HSPI) ---
#define TFT_SCLK        14
#define TFT_MOSI        13
#define TFT_RST         17
#define TFT_DC          16
#define TFT_CS           5

// --- Pin Rotary Encoder ---
#define ROTARY_CLK_PIN  25  // Putaran Kanan/Kiri
#define ROTARY_DT_PIN   26  // Putaran Kanan/Kiri
#define ROTARY_SW_PIN   27  // Tombol Tekan

// --- Pin I2C (SHT30 + DS1307 berbagi bus: SDA=21, SCL=22 / default ESP32) ---
// Alamat: SHT30 = 0x44, DS1307 = 0x68 (tidak bentrok)

// --- Konfigurasi RTC DS1307 ---
#define RTC_CAL_INTERVAL_SEC  (30UL * 24UL * 3600UL) // Kalibrasi ulang tiap 30 hari
#define RTC_READ_MS           1000                   // Baca RTC tiap 1 detik
#define RTC_CAL_CHECK_MS      60000                  // Cek jatuh tempo kalibrasi tiap 1 menit
#define RTC_RETRY_MS          10000                  // Coba ulang bila RTC tidak ditemukan
#define NTP_RETRY_MS          30000                  // Jeda antar percobaan NTP
#define GMT_OFFSET_SEC        (8 * 3600)             // WITA (UTC+8). Ganti 7*3600 untuk WIB, 9*3600 untuk WIT

// --- Konfigurasi Encoder ---
#define ENC_STEPS_PER_DETENT 2     // Pulsa per satu klik fisik
#define ENC_REVERSE          false // Set true jika arah putaran terbalik
#define BTN_DEBOUNCE_MS      30    // Waktu stabil tombol (milidetik)

// --- Peta Layar (128x160) & Tata Letak Baris ---
#define TFT_W           128
#define TFT_H           160
#define ROW_TOPBAR        0
#define ROW_SENSOR_SUHU  14
#define ROW_SENSOR_KELEMB 56
#define ROW_MENU         98
#define ROW_HINT        148
#define NAV_ROW_PITCH    12   // Jarak antar baris menu utama
#define NAV_VISIBLE       4   // Maksimal baris menu yang tampil; sisanya di-scroll
#define NAV_BAR_X       (TFT_W - 3)  // Posisi bilah scroll (kolom paling kanan)

// --- Palet Warna (RGB565) ---
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

// --- Konstanta PID Pemanas ---
#define KP 15.0
#define KI  0.5
#define KD 20.0

// --- Batasan Fisik & Mode Servo ---
#define SERVO_MIN_ANGLE   35.0
#define SERVO_MAX_ANGLE  145.0
#define SERVO_MIN_DUTY   1638
#define SERVO_MAX_DUTY   8192
#define SERVO_MODE_JADWAL  0
#define SERVO_MODE_SWING   1


// ==============================================================================
// 2. VARIABEL GLOBAL & OBJEK STATUS
// ==============================================================================

// --- Target & Status Sensor (NAN = Belum ada data valid) ---
double target_temp      = 37.0;
double target_hum       = 60.0;
int    target_fan_speed = 100;
double current_temp     = NAN;
double current_hum      = NAN;
double heater_pwm_value = 0.0;

// --- Status Servo ---
int   servo_mode            = SERVO_MODE_JADWAL;
int   servo_interval_hours  = 3;
bool  servo_direction_cw    = true;
float current_servo_pos     = 0.0;
float target_servo_pos      = 0.0;
unsigned long last_servo_mode_change = 0;
unsigned long last_servo_update_time = 0;
// Perintah servo dari task jaringan (Core 0) dititipkan lewat servo_cmd_*,
// dieksekusi di loop (Core 1) lewat updateServo.
volatile int8_t  servo_cmd_mode     = -1;  // -1 = tidak ada, 0 = JADWAL, 1 = SWING
volatile int16_t servo_cmd_interval = 0;   // 0 = tidak ada

// --- Status Encoder (dari ISR) ---
volatile int8_t  encState = 0;
volatile int32_t encAccum = 0;
portMUX_TYPE     encMux   = portMUX_INITIALIZER_UNLOCKED;

// --- Status Inkubasi ---
// Perintah dari task jaringan (Core 0) dititipkan lewat inc_cmd_*, dieksekusi di loop (Core 1).
int      inc_profile = EGG_CUSTOM_INDEX;  // Jenis telur yang aktif (indeks EGG_PROFILES)
int      inc_sel     = 0;                 // Pilihan sementara saat memilih jenis di menu
int      inc_days    = 21;                // Target masa inkubasi (hari)
bool     inc_active  = false;             // true = inkubasi sedang berjalan
uint32_t inc_start   = 0;                 // Waktu mulai (unixtime RTC)
int      inc_day     = 0;                 // Hari ke-berapa (1-based, 0 = tidak aktif / RTC error)
volatile int8_t  inc_cmd_profile = -1;    // -1 = tidak ada, selain itu indeks profil
volatile int16_t inc_cmd_days    = 0;     // 0 = tidak ada
volatile int8_t  inc_cmd_start   = -1;    // -1 = tidak ada, 0 = stop, 1 = mulai

// Fase lockdown: beberapa hari terakhir sebelum tetas (lihat kolom lock/drop di egg_profiles.h).
// Selama lockdown: servo berhenti memutar (diam di SERVO_REST_POS) dan setpoint suhu diturunkan.
// target_temp (nilai di menu/NVS) TIDAK diubah; PID memakai setpoint_temp.
#define SERVO_REST_POS   90.0f            // Posisi telur datar saat lockdown (tengah rentang servo)
bool     inc_lockdown  = false;           // true = sedang dalam fase lockdown
double   setpoint_temp = 37.0;            // Setpoint efektif untuk PID (= target_temp - drop saat lockdown)

enum IncAction { INC_ACT_NONE, INC_ACT_TYPE, INC_ACT_DAYS, INC_ACT_STARTSTOP };
IncAction incAction = INC_ACT_NONE;       // Aksi yang sedang dikonfirmasi di menu inkubasi

// --- Status RTC ---
// Catatan: bus I2C (Wire) HANYA diakses dari Core 1 (loop). Task jaringan di Core 0
// hanya mengambil waktu NTP lalu menitipkannya lewat variabel di bawah ini.
RTC_DS1307    rtc;
bool          rtcOK              = false;  // RTC terdeteksi & berjalan
uint16_t      rtcYear            = 0;      // Waktu terakhir yang dibaca dari RTC
uint8_t       rtcMonth           = 0;
uint8_t       rtcDay             = 0;
uint32_t      rtcUnix            = 0;      // Unixtime terakhir yang dibaca dari RTC
uint32_t      rtcLastCal         = 0;      // Waktu kalibrasi terakhir (unixtime RTC, 0 = belum pernah via NTP)
unsigned long lastRtcRead        = 0;
unsigned long lastRtcCalCheck    = 0;
unsigned long lastRtcRetry       = 0;
volatile bool rtcNeedCalibration = false;  // true = menunggu kalibrasi via NTP
volatile bool ntpResultReady     = false;  // true = hasil NTP siap ditulis ke RTC
volatile uint16_t ntpYear        = 0;
volatile uint8_t  ntpMonth = 0, ntpDay = 0, ntpHour = 0, ntpMin = 0, ntpSec = 0;
static const char BUILD_ID[]     = __DATE__ " " __TIME__; // Penanda tiap kali firmware di-upload ulang

// --- Jadwal Task (Waktu dalam ms) ---
unsigned long lastSensorRead            = 0;
unsigned long lastSensorRetry           = 0;
unsigned long lastMqttReconnectAttempt  = 0;
unsigned long lastDisplayUpdate         = 0;
bool          displayDirty              = true;

#define       DISPLAY_COOLDOWN_MS       10
#define       SENSOR_READ_MS            2000
#define       SENSOR_RETRY_MS           5000
#define       MQTT_RECONNECT_MS         10000

// --- Status Kesehatan Sensor ---
bool sensorOK        = false; // Failsafe jika lepas/gagal baca
uint8_t sensorFails  = 0;

// --- Cache Layar (Mencegah Flicker / Redraw Berlebih) ---
struct Cache {
  float  temp     = -999;
  float  hum      = -999;
  float  setTemp  = -999;
  float  setHum   = -999;
  int    wifiSt   = -1;   // 0: Terputus, 1: Tersambung, 2: Portal Aktif
  char   dateStr[12] = {}; // Tanggal yang sedang tampil di topbar
  char   incStr[8]   = {}; // Status hari inkubasi yang sedang tampil di topbar
  int    incLock     = -1; // Status lockdown yang sedang tampil di topbar (warna teks)
  int    menuState  = -1;
  int    subIdx     = -1;
  int    lastNavSel = -1;
  char   navVals[6][16] = {};
  char   editStr[16] = {};
  char   incLbl[4][24] = {}; // Label baris submenu inkubasi
} cache;

// --- Instansiasi Objek Utama ---
SPIClass         tftSPI(HSPI);
Adafruit_ST7735  tft = Adafruit_ST7735(&tftSPI, TFT_CS, TFT_DC, TFT_RST);
Adafruit_SHT31   sht30;
WiFiClientSecure espClient;
PubSubClient     client(espClient);
Preferences      preferences;
WiFiManager      wm;
AutoPID          myPID(&current_temp, &setpoint_temp, &heater_pwm_value, 0, 255, KP, KI, KD);

// --- Mesin Status Menu ---
enum MenuState {
  STATE_NAVIGATE,
  STATE_EDIT,
  STATE_SERVO_SUBMENU,
  STATE_SERVO_EDIT_JADWAL,
  STATE_INC_SUBMENU,
  STATE_INC_EDIT_TYPE,
  STATE_INC_EDIT_DAYS,
  STATE_CONFIRM
};
MenuState menuState      = STATE_NAVIGATE;
int currentMenuIndex     = 0;   // 0: Suhu, 1: Lembab, 2: Kipas, 3: Servo, 4: Candling, 5: Inkubasi
int currentSubmenuIndex  = 0;
const int maxMenuItems   = 6;

void saveAll();   // Deklarasi awal (didefinisikan di bagian encoder & menu)


// ==============================================================================
// 3. FUNGSI KONTROL SERVO
// ==============================================================================

/**
 * @brief Mengubah sudut perintah menjadi duty cycle PWM berdasarkan kalibrasi fisik.
 */
uint32_t angleToDuty(float angle) {
  float phy = SERVO_MIN_ANGLE + (angle / 180.0f) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
  return SERVO_MIN_DUTY + (uint32_t)((phy / 180.0f) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY));
}

/**
 * @brief Memulihkan konfigurasi servo dari memori NVS dan memosisikan ke titik awal.
 */
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

/**
 * @brief Memperbarui gerakan servo secara non-blocking setiap loop. 
 * Mendukung mode ayun terus (swing) dan pembalikan terjadwal (jadwal).
 */
void updateServo() {
  unsigned long now = millis();

  // Eksekusi perintah servo dari MQTT (dashboard): mode JADWAL/SWING
  if (servo_cmd_mode >= 0) {
    int m = servo_cmd_mode;
    servo_cmd_mode = -1;
    servo_mode = (m == SERVO_MODE_SWING) ? SERVO_MODE_SWING : SERVO_MODE_JADWAL;
    if (servo_mode == SERVO_MODE_JADWAL) {
      target_servo_pos = servo_direction_cw ? 180.0f : 0.0f;
    }
    last_servo_mode_change = millis();
    preferences.putInt("s_mode", servo_mode);
    displayDirty = true;
    Serial.printf("[SERVO] Mode dari MQTT: %s\n",
                  servo_mode == SERVO_MODE_SWING ? "SWING" : "JADWAL");
  }
  // Eksekusi perintah interval jadwal dari MQTT
  if (servo_cmd_interval > 0) {
    servo_interval_hours = constrain(servo_cmd_interval, 1, 24);
    servo_cmd_interval = 0;
    preferences.putInt("s_int", servo_interval_hours);
    displayDirty = true;
    Serial.printf("[SERVO] Interval dari MQTT: %d jam\n", servo_interval_hours);
  }

  // Lockdown: hentikan pemutaran, perlahan geser ke posisi datar lalu diam.
  // Setelah lockdown berakhir (mis. inkubasi dihentikan), logika normal di bawah
  // otomatis melanjutkan pemutaran dari posisi saat ini.
  if (inc_lockdown) {
    if (fabs(current_servo_pos - SERVO_REST_POS) > 0.01f && now - last_servo_update_time >= 30) {
      last_servo_update_time = now;
      current_servo_pos += (current_servo_pos < SERVO_REST_POS) ? 1.0f : -1.0f;
      ledcWrite(SERVO_PIN, angleToDuty(current_servo_pos));
    }
    last_servo_mode_change = now;   // Tunda jadwal pembalikan selama lockdown
    return;
  }

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


// ==============================================================================
// 3A. FUNGSI LED CANDLING
// ==============================================================================
// Perintah dari task jaringan (Core 0) hanya dititipkan lewat candle_cmd;
// penulisan pin dan pembaruan layar dilakukan di loop (Core 1).

bool          candle_on      = false;
unsigned long candle_start   = 0;
unsigned long candle_dur_ms  = 0;                      // 0 = tanpa auto-off
volatile int8_t   candle_cmd     = -1;                 // -1 = tidak ada, 0 = OFF, 1 = ON
volatile uint32_t candle_cmd_sec = CANDLE_DEFAULT_SEC; // Durasi auto-off untuk perintah ON

/** @brief Menyalakan/mematikan LED candling. sec = durasi auto-off (0 = tanpa batas) */
void setCandle(bool on, uint32_t sec = CANDLE_DEFAULT_SEC) {
  if (sec > CANDLE_MAX_SEC) sec = CANDLE_MAX_SEC;
  candle_on     = on;
  candle_start  = millis();
  candle_dur_ms = (unsigned long)sec * 1000UL;
  digitalWrite(LED_CANDLE_PIN, on ? HIGH : LOW);
  displayDirty  = true;
  Serial.printf("[CANDLE] %s (auto-off %us)\n", on ? "ON" : "OFF", (unsigned)sec);
}

/** @brief Dipanggil dari loop: eksekusi perintah MQTT dan cek auto-off */
void updateCandling() {
  if (candle_cmd >= 0) {
    int8_t   c = candle_cmd;
    uint32_t s = candle_cmd_sec;
    candle_cmd = -1;
    setCandle(c == 1, s);
  }
  if (candle_on && candle_dur_ms > 0 && millis() - candle_start >= candle_dur_ms) {
    setCandle(false);
    Serial.println("[CANDLE] Auto-off");
  }
}


// ==============================================================================
// 3B. FUNGSI RTC DS1307 (KALIBRASI AWAL SAAT UPLOAD + KALIBRASI TIAP 30 HARI)
// ==============================================================================

/**
 * @brief Menggeser waktu mulai inkubasi saat RTC dikoreksi (set kompilasi / NTP),
 * supaya hitungan hari tidak melompat ketika jam RTC berubah.
 */
void shiftIncubationStart(uint32_t before, uint32_t after) {
  if (!inc_active) return;
  inc_start = (uint32_t)((int64_t)inc_start + ((int64_t)after - (int64_t)before));
  preferences.putUInt("i_start", inc_start);
}

/** @brief Menyalin tanggal RTC ke buffer "DD-MM-YYYY" (atau "--/--/----" bila RTC tak tersedia) */
void formatDate(char* buf, size_t len) {
  if (!rtcOK || rtcYear < 2020) snprintf(buf, len, "--/--/----");
  else                          snprintf(buf, len, "%02d/%02d/%04d", rtcDay, rtcMonth, rtcYear);
}

/** @brief True bila RTC belum pernah dikalibrasi via NTP atau sudah lewat 30 hari */
bool isCalibrationDue(uint32_t nowUnix) {
  if (rtcLastCal == 0)         return true;
  if (nowUnix < rtcLastCal)    return true;  // Jam mundur: data tidak masuk akal
  return (nowUnix - rtcLastCal) >= RTC_CAL_INTERVAL_SEC;
}

/** @brief Membaca RTC ke variabel global; menandai layar kotor jika tanggal berganti */
void readRtcNow() {
  DateTime n = rtc.now();
  rtcUnix = n.unixtime();
  if (n.day() != rtcDay || n.month() != rtcMonth || n.year() != rtcYear) {
    rtcDay = n.day(); rtcMonth = n.month(); rtcYear = n.year();
    displayDirty = true;
  }
}

/**
 * @brief Inisialisasi RTC.
 * - Firmware baru di-upload (BUILD_ID beda) atau RTC mati (baterai habis):
 *   RTC diset dari waktu kompilasi, lalu minta kalibrasi NTP begitu WiFi tersedia.
 * - Selain itu: kalibrasi dijadwalkan bila sudah 30 hari sejak kalibrasi terakhir.
 */
bool initRtc() {
  Wire.begin();
  if (!rtc.begin()) {
    Serial.println("[RTC] DS1307 NOT FOUND");
    rtcOK = false;
    return false;
  }

  String savedBuild = preferences.getString("fw_build", "");
  bool newUpload = (savedBuild != String(BUILD_ID));
  rtcLastCal = preferences.getUInt("rtc_cal", 0);

  if (newUpload || !rtc.isrunning()) {
    bool wasRunning = rtc.isrunning();
    uint32_t before = wasRunning ? rtc.now().unixtime() : 0;
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Kalibrasi awal: waktu saat kompilasi
    if (wasRunning) shiftIncubationStart(before, rtc.now().unixtime());
    preferences.putString("fw_build", BUILD_ID);
    rtcLastCal = 0;                                  // 0 = belum dikalibrasi via NTP
    preferences.putUInt("rtc_cal", 0);
    Serial.println(newUpload ? "[RTC] Upload baru: diset dari waktu kompilasi"
                             : "[RTC] RTC mati: diset dari waktu kompilasi");
  }

  rtcOK = true;
  readRtcNow();

  if (isCalibrationDue(rtc.now().unixtime())) {
    rtcNeedCalibration = true;
    Serial.println("[RTC] Menunggu kalibrasi NTP");
  }
  Serial.printf("[RTC] OK %02d-%02d-%04d\n", rtcDay, rtcMonth, rtcYear);
  return true;
}

/** @brief Dipanggil dari loop (Core 1): baca RTC, tulis hasil NTP, dan cek jadwal 30 hari */
void updateRtc() {
  unsigned long now = millis();

  // RTC belum ditemukan: coba lagi berkala
  if (!rtcOK) {
    if (now - lastRtcRetry >= RTC_RETRY_MS) {
      lastRtcRetry = now;
      if (initRtc()) displayDirty = true;
    }
    return;
  }

  // Terapkan hasil NTP dari task jaringan ke RTC
  if (ntpResultReady) {
    uint32_t before = rtc.now().unixtime();
    rtc.adjust(DateTime(ntpYear, ntpMonth, ntpDay, ntpHour, ntpMin, ntpSec));
    uint32_t after = rtc.now().unixtime();
    shiftIncubationStart(before, after);   // Jaga kontinuitas hitungan hari inkubasi
    rtcLastCal = after;
    preferences.putUInt("rtc_cal", rtcLastCal);
    rtcNeedCalibration = false;
    ntpResultReady = false;
    readRtcNow();
    Serial.printf("[RTC] Dikalibrasi NTP: %02d-%02d-%04d\n", rtcDay, rtcMonth, rtcYear);
  }

  // Baca waktu berkala
  if (now - lastRtcRead >= RTC_READ_MS) {
    lastRtcRead = now;
    readRtcNow();
  }

  // Cek apakah sudah waktunya kalibrasi bulanan
  if (!rtcNeedCalibration && now - lastRtcCalCheck >= RTC_CAL_CHECK_MS) {
    lastRtcCalCheck = now;
    if (isCalibrationDue(rtc.now().unixtime())) {
      rtcNeedCalibration = true;
      Serial.println("[RTC] 30 hari berlalu, jadwalkan kalibrasi NTP");
    }
  }
}


// ==============================================================================
// 3C. FUNGSI INKUBASI (JENIS TELUR, TARGET HARI, HITUNG HARI)
// ==============================================================================

/**
 * @brief Menerapkan profil telur: mengisi target suhu, kelembapan, dan hari.
 * Profil "KUSTOM" hanya memilih jenis tanpa menimpa nilai yang sudah diatur.
 * Nilai tetap bisa diubah manual lewat menu SET SUHU / SET LEMBAB / INKUBASI.
 */
void applyProfile(int idx) {
  if (idx < 0 || idx >= EGG_PROFILE_COUNT) return;
  inc_profile = idx;
  const EggProfile& p = EGG_PROFILES[idx];
  if (!p.custom) {
    target_temp = p.temp;
    target_hum  = p.hum;
    inc_days    = p.days;
  }
  saveAll();
  displayDirty = true;
  Serial.printf("[INC] Profil: %s (%d hari, %.1f C, %d%%)\n",
                p.name, inc_days, target_temp, (int)target_hum);
}

/** @brief Memulai hitungan inkubasi dari sekarang (butuh RTC aktif) */
void startIncubation() {
  if (!rtcOK || rtcUnix == 0) {
    Serial.println("[INC] Gagal mulai: RTC tidak tersedia");
    return;
  }
  inc_active = true;
  inc_start  = rtcUnix;
  inc_day    = 1;
  saveAll();
  displayDirty = true;
  Serial.println("[INC] Inkubasi dimulai");
}

/** @brief Menghentikan hitungan inkubasi */
void stopIncubation() {
  inc_active = false;
  inc_day    = 0;
  saveAll();
  displayDirty = true;
  Serial.println("[INC] Inkubasi dihentikan");
}

/** @brief Dipanggil dari loop: eksekusi perintah MQTT dan hitung hari ke-berapa */
void updateIncubation() {
  if (inc_cmd_profile >= 0) {
    int p = inc_cmd_profile;
    inc_cmd_profile = -1;
    applyProfile(p);
  }
  if (inc_cmd_days > 0) {
    int d = inc_cmd_days;
    inc_cmd_days = 0;
    inc_days = constrain(d, EGG_DAYS_MIN, EGG_DAYS_MAX);
    saveAll();
    displayDirty = true;
  }
  if (inc_cmd_start >= 0) {
    int8_t c = inc_cmd_start;
    inc_cmd_start = -1;
    if (c == 1) { if (!inc_active) startIncubation(); }
    else        { if (inc_active)  stopIncubation();  }
  }

  int d = 0;
  if (inc_active && rtcOK) {
    d = (rtcUnix >= inc_start) ? (int)((rtcUnix - inc_start) / 86400UL) + 1 : 1;
  }
  if (d != inc_day) {
    inc_day = d;
    displayDirty = true;
  }

  // Fase lockdown: hari ke-N dari (inc_days - lock + 1) sampai tetas.
  // Bila RTC sedang error saat inkubasi berjalan, status lockdown dipertahankan.
  bool lock = inc_lockdown;
  if (!inc_active) {
    lock = false;
  } else if (rtcOK && inc_day > 0) {
    const EggProfile& p = EGG_PROFILES[inc_profile];
    int ld = min((int)p.lock, inc_days - 1);   // Minimal 1 hari tetap normal
    lock = (ld > 0) && (inc_day > inc_days - ld);
  }
  if (lock != inc_lockdown) {
    inc_lockdown = lock;
    displayDirty = true;
    Serial.println(lock ? "[INC] LOCKDOWN: pemutaran dihentikan, suhu diturunkan"
                        : "[INC] Lockdown selesai");
  }

  // Setpoint efektif untuk PID
  double sp = target_temp - (inc_lockdown ? EGG_PROFILES[inc_profile].drop : 0.0);
  setpoint_temp = constrain(sp, 20.0, 50.0);
}

/** @brief Teks status untuk topbar: "H5/21", "TETAS!" atau kosong bila tidak aktif */
void formatIncStatus(char* buf, size_t len) {
  if (!inc_active)                snprintf(buf, len, "");
  else if (inc_day <= 0)          snprintf(buf, len, "H--/%d", inc_days);
  else if (inc_day > inc_days)    snprintf(buf, len, "TETAS!");
  else                            snprintf(buf, len, "H%d/%d", inc_day, inc_days);
}


// ==============================================================================
// 4. FUNGSI TAMPILAN (TFT DISPLAY)
// ==============================================================================

/** @brief Menulis teks rata kiri dan menghapus latar zonanya */
void drawZoneText(int x, int y, int zoneW, int zoneH, const char* txt, uint16_t color, uint8_t sz, uint16_t bg = C_BG) {
  tft.fillRect(x, y, zoneW, zoneH, bg);
  tft.setTextSize(sz);
  tft.setTextColor(color);
  tft.setCursor(x, y);
  tft.print(txt);
}

/** @brief Menulis teks rata kanan dan menghapus latar zonanya */
void drawZoneTextRight(int x, int y, int zoneW, int zoneH, const char* txt, uint16_t color, uint8_t sz, uint16_t bg = C_BG) {
  tft.fillRect(x, y, zoneW, zoneH, bg);
  tft.setTextSize(sz);
  tft.setTextColor(color);
  int tw = strlen(txt) * 6 * sz;
  tft.setCursor(x + zoneW - tw, y);
  tft.print(txt);
}

/** @brief Menggambar bitmap monokrom dengan skala integer (mis. 16x16 -> 32x32). */
void drawBitmapScaled(int16_t x, int16_t y, const uint8_t *bitmap,
                      int16_t w, int16_t h, uint16_t color, uint8_t scale) {
  int16_t byteW = (w + 7) / 8;
  for (int16_t j = 0; j < h; j++) {
    for (int16_t i = 0; i < w; i++) {
      if (pgm_read_byte(&bitmap[j * byteW + i / 8]) & (0x80 >> (i & 7))) {
        if (scale == 1) tft.drawPixel(x + i, y + j, color);
        else            tft.fillRect(x + i * scale, y + j * scale, scale, scale, color);
      }
    }
  }
}

// Ikon sensor 24x24 digambar 1:1 agar setinggi angka textSize 3 (24px)
#define SENSOR_ICON_SCALE 1
#define SENSOR_ICON_X     4
#define SENSOR_VAL_X      (SENSOR_ICON_X + SENSOR_ICON_W * SENSOR_ICON_SCALE + 6)

/** @brief Menggambar bingkai UI statis sekali saat booting */
void drawChrome() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 12, TFT_W, 2, C_DIV_TOPBAR);
  tft.fillRect(0, ROW_SENSOR_KELEMB - 2, TFT_W, 2, C_DIV_HORIZ);
  tft.fillRect(0, ROW_MENU - 2, TFT_W, 2, C_DIV_MENU);

  // Ikon di depan (kiri), seukuran angka: 24x24 sejajar dengan zona nilai
  drawBitmapScaled(SENSOR_ICON_X, ROW_SENSOR_SUHU + 10,
                   icon_temperature, SENSOR_ICON_W, SENSOR_ICON_H,
                   C_LABEL_SUHU, SENSOR_ICON_SCALE);
  drawBitmapScaled(SENSOR_ICON_X, ROW_SENSOR_KELEMB + 10,
                   icon_humidity, SENSOR_ICON_W, SENSOR_ICON_H,
                   C_LABEL_KELEMB, SENSOR_ICON_SCALE);
}

/** @brief Memperbarui bilah status atas (saat status WiFi, tanggal, atau hari inkubasi berubah) */
void updateTopbar() {
  int st = 0;
  if      (WiFi.status() == WL_CONNECTED)   st = 1;
  else if (wm.getConfigPortalActive())      st = 2;

  char dateStr[12];
  formatDate(dateStr, sizeof(dateStr));

  char incStr[8];
  formatIncStatus(incStr, sizeof(incStr));

  bool wifiChanged = (st != cache.wifiSt);
  bool dateChanged = (strcmp(dateStr, cache.dateStr) != 0);
  bool incChanged  = (strcmp(incStr, cache.incStr) != 0) || (cache.incLock != (int)inc_lockdown);
  if (!wifiChanged && !dateChanged && !incChanged) return;

  cache.incLock = (int)inc_lockdown;
  cache.wifiSt = st;
  strcpy(cache.dateStr, dateStr);
  strcpy(cache.incStr, incStr);
  displayDirty = true;

  tft.fillRect(0, ROW_TOPBAR, TFT_W, 13, C_TITLE_BG);
  tft.setTextSize(1);
  tft.setTextColor(C_TITLE);
  tft.setCursor(2, 3);
  tft.print(dateStr);   // Tanggal-bulan-tahun di pojok kiri (menggantikan teks INCUBATOR)

  // Hari inkubasi di tengah topbar ("H5/21"); cyan saat lockdown, oranye bila waktunya menetas
  if (incStr[0]) {
    uint16_t incColor = C_SET;
    if (inc_day > inc_days)  incColor = C_CONFIRM_VAL;
    else if (inc_lockdown)   incColor = C_EDIT_VAL;
    tft.setTextColor(incColor);
    tft.setCursor(66, 3);
    tft.print(incStr);
  }

  // Ikon WiFi di pojok kanan (bitmap dari icons.h)
  const int ICON_X = TFT_W - ICON_W - 3;
  const int ICON_Y = 1;
  if (st == 1) {            // Terhubung: hijau
    tft.drawBitmap(ICON_X, ICON_Y, ICON_WIFI, ICON_W, ICON_H, C_WIFI_OK);
  } else if (st == 2) {     // Portal konfigurasi aktif: oranye
    tft.drawBitmap(ICON_X, ICON_Y, ICON_WIFI, ICON_W, ICON_H, C_WIFI_AP);
  } else {                  // Terputus: abu-abu + coretan merah
    tft.drawBitmap(ICON_X, ICON_Y, ICON_WIFI, ICON_W, ICON_H, C_DIMMED);
    tft.drawBitmap(ICON_X, ICON_Y, ICON_WIFI_SLASH, ICON_W, ICON_H, C_WIFI_DC);
  }
}

/** @brief Menggambar pembaruan sensor, me-redraw hanya bila melewati batas deadband */
void updateSensorZone() {
  bool changed = false;

  // Render Suhu (ikon 32x32 di kiri, angka rata-kiri tepat di depannya)
  {
    bool invalid = isnan(current_temp);
    float cmp = invalid ? -9999.0f : (float)current_temp;
    if (fabs(cmp - cache.temp) >= 0.05f) {
      cache.temp = cmp;
      char buf[12];
      if (invalid) snprintf(buf, sizeof(buf), "--.-");
      else         snprintf(buf, sizeof(buf), "%.1f", current_temp);
      
      tft.fillRect(SENSOR_VAL_X, ROW_SENSOR_SUHU + 10, TFT_W - SENSOR_VAL_X - 2, 24, C_BG);
      tft.setTextSize(3); tft.setTextColor(0xFD20);
      int valW = strlen(buf) * 18;
      int startX = SENSOR_VAL_X;
      
      tft.setCursor(startX, ROW_SENSOR_SUHU + 12);
      tft.print(buf);
      tft.setTextSize(1); tft.setTextColor(0xFE60);
      tft.setCursor(startX + valW, ROW_SENSOR_SUHU + 18);
      tft.print("\xF7""C");
      changed = true;
    }
  }

  // Render Kelembapan (ikon 32x32 di kiri, angka rata-kiri tepat di depannya)
  {
    bool invalid = isnan(current_hum);
    float cmp = invalid ? -9999.0f : (float)current_hum;
    if (fabs(cmp - cache.hum) >= 0.3f) {
      cache.hum = cmp;
      char buf[12];
      if (invalid) snprintf(buf, sizeof(buf), "--.-");
      else         snprintf(buf, sizeof(buf), "%.1f", current_hum);
      
      tft.fillRect(SENSOR_VAL_X, ROW_SENSOR_KELEMB + 10, TFT_W - SENSOR_VAL_X - 2, 24, C_BG);
      tft.setTextSize(3); tft.setTextColor(0x041F);
      int valW = strlen(buf) * 18;
      int startX = SENSOR_VAL_X;
      
      tft.setCursor(startX, ROW_SENSOR_KELEMB + 12);
      tft.print(buf);
      tft.setTextSize(1); tft.setTextColor(0x87FF);
      tft.setCursor(startX + valW, ROW_SENSOR_KELEMB + 18);
      tft.print("%");
      changed = true;
    }
  }

  // Memulihkan grid pembatas jika ada zona yang tertimpa
  if (changed) {
    tft.fillRect(0, 12, TFT_W, 2, C_DIV_TOPBAR);
    tft.fillRect(0, ROW_SENSOR_KELEMB - 2, TFT_W, 2, C_DIV_HORIZ);
    tft.fillRect(0, ROW_MENU - 2, TFT_W, 2, C_DIV_MENU);
    displayDirty = true;
  }
}

static const char* const NAV_LABELS[] = {"SET SUHU", "SET LEMBAB", "FAN SPEED", "SERVO", "CANDLING", "INKUBASI"};
static const char* const SERVO_OPTS[] = {"JADWAL", "SWING", "KEMBALI"};

void formatMenuVal(int idx, char* buf, size_t len) {
  if      (idx == 0) snprintf(buf, len, "%.1f\xF7""C", target_temp);
  else if (idx == 1) snprintf(buf, len, "%d%%", (int)target_hum);
  else if (idx == 2) snprintf(buf, len, "%d%%", target_fan_speed);
  else if (idx == 3) snprintf(buf, len, "%s", servo_mode == SERVO_MODE_SWING ? "SWING" : "JADWAL");
  else if (idx == 4) snprintf(buf, len, "%s", candle_on ? "ON" : "OFF");
  else {
    // Inkubasi: tampilkan "H5/21" bila berjalan, selain itu nama jenis telur
    if (inc_active && inc_day > 0) snprintf(buf, len, "H%d/%d", inc_day, inc_days);
    else                           snprintf(buf, len, "%s", EGG_PROFILES[inc_profile].name);
  }
}

int navScroll = 0;   // Indeks menu pertama yang tampil di jendela scroll

/** @brief Menjaga item terpilih tetap di dalam jendela 4 baris. True bila jendela bergeser. */
bool updateNavScroll() {
  int old = navScroll;
  if (currentMenuIndex < navScroll)                      navScroll = currentMenuIndex;
  if (currentMenuIndex >= navScroll + NAV_VISIBLE)       navScroll = currentMenuIndex - NAV_VISIBLE + 1;
  navScroll = constrain(navScroll, 0, maxMenuItems - NAV_VISIBLE);
  return navScroll != old;
}

/** @brief Bilah scroll tipis di tepi kanan area menu (hanya bila item lebih dari jendela) */
void drawNavScrollbar() {
  const int trackY = ROW_MENU;
  const int trackH = NAV_VISIBLE * NAV_ROW_PITCH;
  tft.fillRect(NAV_BAR_X, trackY, 2, trackH, C_BG);
  if (maxMenuItems <= NAV_VISIBLE) return;
  tft.fillRect(NAV_BAR_X, trackY, 2, trackH, C_SEL_BG);                   // Jalur
  int thumbH = max(6, trackH * NAV_VISIBLE / maxMenuItems);
  int range  = maxMenuItems - NAV_VISIBLE;
  int thumbY = trackY + (trackH - thumbH) * navScroll / range;
  tft.fillRect(NAV_BAR_X, thumbY, 2, thumbH, C_SELECTED);                 // Penunjuk
}

void drawNavRow(int idx, bool sel) {
  int slot = idx - navScroll;
  if (slot < 0 || slot >= NAV_VISIBLE) return;   // Di luar jendela scroll: tidak digambar
  int y = ROW_MENU + 2 + slot * NAV_ROW_PITCH;
  // Lebar dikurangi 4px agar bilah scroll di kanan tidak tertimpa
  tft.fillRect(0, y - 2, TFT_W - 4, NAV_ROW_PITCH, sel ? C_SEL_BG : C_BG);
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
  if (!sel) tft.fillRect(50, y, TFT_W - 50, 10, C_BG);
}

/** @brief Label baris submenu inkubasi (dinamis mengikuti status) */
void incOptLabel(int idx, char* buf, size_t len) {
  if      (idx == 0) snprintf(buf, len, "JENIS: %s", EGG_PROFILES[inc_profile].name);
  else if (idx == 1) snprintf(buf, len, "%s", inc_active ? "STOP INKUBASI"
                                              : (rtcOK ? "MULAI SEKARANG" : "MULAI (RTC ERR)"));
  else if (idx == 2) snprintf(buf, len, "TARGET: %d HARI", inc_days);
  else               snprintf(buf, len, "KEMBALI");
}

void drawIncOptRow(int idx, bool sel, const char* label) {
  int y = ROW_MENU + 12 + idx * 9;
  tft.fillRect(0, y - 1, TFT_W, 9, sel ? C_SEL_BG : C_BG);
  tft.setTextSize(1);
  tft.setCursor(8, y);
  tft.setTextColor(sel ? C_SELECTED : C_NAV_LABEL);
  tft.print(sel ? "> " : "  ");
  tft.print(label);
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
  int xc = max((TFT_W - tw) / 2, 2);
  tft.setTextSize(3); tft.setTextColor(valColor);
  tft.setCursor(xc, ROW_MENU + 20);
  tft.print(valStr);
  drawHint(hintTxt);
}

void drawEditValueOnly(const char* valStr, uint16_t valColor) {
  tft.fillRect(0, ROW_MENU + 14, TFT_W, 34, C_BG);
  int tw = strlen(valStr) * 18;
  int xc = max((TFT_W - tw) / 2, 2);
  tft.setTextSize(3); tft.setTextColor(valColor);
  tft.setCursor(xc, ROW_MENU + 20);
  tft.print(valStr);
}

/** @brief Dispatcher rendering menu berdasarkan status mesin saat ini. */
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
    bool scrolled = updateNavScroll();
    if (cache.lastNavSel == -1 || scrolled) {
      // Gambar ulang seluruh jendela (awal masuk menu atau saat daftar bergeser)
      tft.fillRect(0, ROW_MENU, TFT_W, ROW_HINT - ROW_MENU, C_BG);
      for (int i = 0; i < maxMenuItems; i++) {
        if (i >= navScroll && i < navScroll + NAV_VISIBLE) drawNavRow(i, i == newSel);
        formatMenuVal(i, cache.navVals[i], sizeof(cache.navVals[i]));
      }
      drawNavScrollbar();
      if (cache.lastNavSel == -1) drawHint("Putar=pilih  Tekan=atur");
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

  } else if (menuState == STATE_INC_SUBMENU) {
    if (stateChanged || cache.lastNavSel != -3) {
      tft.fillRect(0, ROW_MENU, TFT_W, 12, C_BG);
      tft.setTextSize(1); tft.setTextColor(C_UNIT);
      tft.setCursor(4, ROW_MENU + 2);
      tft.print("INKUBASI:");
      cache.lastNavSel = -3;
      cache.subIdx = -1;
      memset(cache.incLbl, 0, sizeof(cache.incLbl));
      drawHint("Putar=pilih  Tekan=pilih");
    }
    bool selChanged = (currentSubmenuIndex != cache.subIdx);
    for (int i = 0; i < 4; i++) {
      char lb[24];
      incOptLabel(i, lb, sizeof(lb));
      bool lblChanged = (strcmp(lb, cache.incLbl[i]) != 0);
      if (lblChanged || (selChanged && (i == currentSubmenuIndex || i == cache.subIdx))) {
        strcpy(cache.incLbl[i], lb);
        drawIncOptRow(i, i == currentSubmenuIndex, lb);
      }
    }
    cache.subIdx = currentSubmenuIndex;

  } else if (menuState == STATE_INC_EDIT_TYPE) {
    const EggProfile& p = EGG_PROFILES[inc_sel];
    if (stateChanged || strcmp(p.name, cache.editStr) != 0) {
      strncpy(cache.editStr, p.name, sizeof(cache.editStr) - 1);
      cache.editStr[sizeof(cache.editStr) - 1] = '\0';
      char info[24];
      if (p.custom) snprintf(info, sizeof(info), "ATUR MANUAL");
      else          snprintf(info, sizeof(info), "%.1f\xF7""C %d%% %dHR", p.temp, p.hum, p.days);
      drawEditScreen(info, p.name, C_EDIT_VAL, "Putar=jenis Tekan=ok");
    }

  } else if (menuState == STATE_INC_EDIT_DAYS) {
    char vb[16]; snprintf(vb, sizeof(vb), "%d HARI", inc_days);
    if (stateChanged) {
      strcpy(cache.editStr, vb);
      drawEditScreen("TARGET INKUBASI:", vb, C_EDIT_VAL, "Putar=ubah   Tekan=konfirm");
    } else if (strcmp(vb, cache.editStr) != 0) {
      strcpy(cache.editStr, vb);
      drawEditValueOnly(vb, C_EDIT_VAL);
    }

  } else if (menuState == STATE_EDIT) {
    const char* labels[] = {"ATUR SUHU", "ATUR LEMBAB", "FAN SPEED", "", "LED CANDLING"};
    char vb[16];
    if      (currentMenuIndex == 0) snprintf(vb, 16, "%.1f\xF7""C", target_temp);
    else if (currentMenuIndex == 1) snprintf(vb, 16, "%d%%", (int)target_hum);
    else if (currentMenuIndex == 2) snprintf(vb, 16, "%d%%", target_fan_speed);
    else                            snprintf(vb, 16, "%s", candle_on ? "ON" : "OFF");
    
    if (stateChanged) {
      strcpy(cache.editStr, vb);
      drawEditScreen(labels[currentMenuIndex], vb, C_EDIT_VAL, "Putar=ubah   Tekan=konfirm");
    } else if (strcmp(vb, cache.editStr) != 0) {
      strcpy(cache.editStr, vb);
      drawEditValueOnly(vb, C_EDIT_VAL);
    }

  } else if (menuState == STATE_CONFIRM) {
    char vb[16];
    const char* lbl = "SIMPAN PERUBAHAN?";
    if      (currentMenuIndex == 0) snprintf(vb, 16, "%.1f\xF7""C", target_temp);
    else if (currentMenuIndex == 1) snprintf(vb, 16, "%d%%", (int)target_hum);
    else if (currentMenuIndex == 2) snprintf(vb, 16, "%d%%", target_fan_speed);
    else if (currentMenuIndex == 4) snprintf(vb, 16, "%s", candle_on ? "ON" : "OFF");
    else if (currentMenuIndex == 5) {
      if (incAction == INC_ACT_TYPE) {
        lbl = "TERAPKAN JENIS?";
        snprintf(vb, 16, "%s", EGG_PROFILES[inc_sel].name);
      } else if (incAction == INC_ACT_DAYS) {
        lbl = "SIMPAN TARGET?";
        snprintf(vb, 16, "%d HARI", inc_days);
      } else {
        lbl = inc_active ? "HENTIKAN INKUBASI?" : "MULAI INKUBASI?";
        snprintf(vb, 16, "YA");
      }
    }
    else snprintf(vb, 16, "%s", servo_mode == SERVO_MODE_SWING ? "SWING" : "JADWAL");
    
    if (stateChanged) {
      strcpy(cache.editStr, vb);
      drawEditScreen(lbl, vb, C_CONFIRM_VAL, "Tekan=simpan  Putar=batal");
    } else if (strcmp(vb, cache.editStr) != 0) {
      strcpy(cache.editStr, vb);
      drawEditValueOnly(vb, C_CONFIRM_VAL);
    }
  }

  cache.menuState = (int)menuState;
}

void updateDisplay() {
  updateTopbar();
  drawMenuZone();
}

void updateSensorDisplay() {
  updateSensorZone();
}


// ==============================================================================
// 5. FUNGSI ENCODER & MENU LOGIC
// ==============================================================================

/** @brief Interupsi membaca status quadrature encoder tanpa memerlukan debounce delay. */
void IRAM_ATTR isr_encoder() {
  static const int8_t T[16] = { 0,-1, 1, 0, 1, 0, 0,-1, -1, 0, 0, 1, 0, 1,-1, 0 };
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

/** @brief Membaca jumlah putaran encoder sejak panggilan terakhir */
int readEncoderSteps() {
  portENTER_CRITICAL(&encMux);
  int steps = encAccum / ENC_STEPS_PER_DETENT;
  encAccum -= steps * ENC_STEPS_PER_DETENT;
  portEXIT_CRITICAL(&encMux);
  return ENC_REVERSE ? -steps : steps;
}

/** @brief Membaca tombol tekan dengan debounce berbasis waktu */
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

void saveAll() {
  preferences.putDouble("t_temp", target_temp);
  preferences.putDouble("t_hum",  target_hum);
  preferences.putInt("t_fan",     target_fan_speed);
  preferences.putInt("s_mode",    servo_mode);
  preferences.putInt("s_int",     servo_interval_hours);
  preferences.putBool("s_dir",    servo_direction_cw);
  preferences.putInt("i_prof",    inc_profile);
  preferences.putInt("i_days",    inc_days);
  preferences.putBool("i_act",    inc_active);
  preferences.putUInt("i_start",  inc_start);
  // Status LED candling sengaja tidak disimpan (selalu mati setelah reboot)
}

/** @brief Mengontrol navigasi dan manipulasi variabel berdasarkan input encoder/tombol */
void handleMenu() {
  if (readButton()) {
    displayDirty = true;
    if (menuState == STATE_NAVIGATE) {
      if (currentMenuIndex == 3) {
        menuState = STATE_SERVO_SUBMENU;
        currentSubmenuIndex = 0;
      } else if (currentMenuIndex == 5) {
        menuState = STATE_INC_SUBMENU;
        currentSubmenuIndex = 0;
      } else {
        menuState = STATE_EDIT;
      }
    } else if (menuState == STATE_EDIT) {
      menuState = STATE_CONFIRM;
    } else if (menuState == STATE_SERVO_SUBMENU) {
      if (currentSubmenuIndex == 0) {
        servo_mode = SERVO_MODE_JADWAL;
        menuState  = STATE_SERVO_EDIT_JADWAL;
      } else if (currentSubmenuIndex == 1) {
        servo_mode = SERVO_MODE_SWING;
        menuState  = STATE_CONFIRM;
      } else {
        menuState = STATE_NAVIGATE;
        currentMenuIndex = 3;
      }
    } else if (menuState == STATE_SERVO_EDIT_JADWAL) {
      menuState = STATE_CONFIRM;
    } else if (menuState == STATE_INC_SUBMENU) {
      if (currentSubmenuIndex == 0) {            // Pilih jenis telur
        inc_sel   = inc_profile;
        menuState = STATE_INC_EDIT_TYPE;
      } else if (currentSubmenuIndex == 1) {     // Mulai / hentikan inkubasi
        if (inc_active || rtcOK) {               // Mulai butuh RTC aktif
          incAction = INC_ACT_STARTSTOP;
          menuState = STATE_CONFIRM;
        }
      } else if (currentSubmenuIndex == 2) {     // Ubah target hari
        menuState = STATE_INC_EDIT_DAYS;
      } else {                                   // Kembali
        menuState = STATE_NAVIGATE;
        currentMenuIndex = 5;
      }
    } else if (menuState == STATE_INC_EDIT_TYPE) {
      incAction = INC_ACT_TYPE;
      menuState = STATE_CONFIRM;
    } else if (menuState == STATE_INC_EDIT_DAYS) {
      incAction = INC_ACT_DAYS;
      menuState = STATE_CONFIRM;
    } else if (menuState == STATE_CONFIRM) {
      if (currentMenuIndex == 3) {
        target_servo_pos = servo_direction_cw ? 180.0f : 0.0f;
        last_servo_mode_change = millis();
      } else if (currentMenuIndex == 5) {
        if (incAction == INC_ACT_TYPE) {
          applyProfile(inc_sel);                 // Isi suhu, lembab, hari otomatis
        } else if (incAction == INC_ACT_STARTSTOP) {
          if (inc_active) stopIncubation();
          else            startIncubation();
        }
        // INC_ACT_DAYS: inc_days sudah berubah saat diputar, cukup disimpan di bawah
      }
      saveAll();
      menuState = STATE_NAVIGATE;
    }
  }

  int steps = readEncoderSteps();
  if (steps != 0) {
    displayDirty = true;
    if (menuState == STATE_NAVIGATE) {
      currentMenuIndex = ((currentMenuIndex + steps) % maxMenuItems + maxMenuItems) % maxMenuItems;
    } else if (menuState == STATE_SERVO_SUBMENU) {
      currentSubmenuIndex = ((currentSubmenuIndex + steps) % 3 + 3) % 3;
    } else if (menuState == STATE_SERVO_EDIT_JADWAL) {
      servo_interval_hours = constrain(servo_interval_hours + steps, 1, 24);
    } else if (menuState == STATE_INC_SUBMENU) {
      currentSubmenuIndex = ((currentSubmenuIndex + steps) % 4 + 4) % 4;
    } else if (menuState == STATE_INC_EDIT_TYPE) {
      inc_sel = ((inc_sel + steps) % EGG_PROFILE_COUNT + EGG_PROFILE_COUNT) % EGG_PROFILE_COUNT;
    } else if (menuState == STATE_INC_EDIT_DAYS) {
      inc_days = constrain(inc_days + steps, EGG_DAYS_MIN, EGG_DAYS_MAX);
    } else if (menuState == STATE_EDIT) {
      if (currentMenuIndex == 0)
        target_temp = constrain(round((target_temp + steps * 0.1) * 10.0) / 10.0, 20.0, 50.0);
      else if (currentMenuIndex == 1)
        target_hum = constrain(target_hum + steps * 1.0, 30.0, 90.0);
      else if (currentMenuIndex == 2)
        target_fan_speed = constrain(target_fan_speed + steps * 5, 0, 100);
      else if (currentMenuIndex == 4)
        setCandle(steps > 0);   // Putar kanan = ON, putar kiri = OFF (LED langsung berubah)
      cache.setTemp = -999; cache.setHum = -999;
    } else if (menuState == STATE_CONFIRM) {
      // Pembatalan (Cancel) dari layar confirm
      if (currentMenuIndex == 3) {
        menuState = (servo_mode == SERVO_MODE_SWING) ? STATE_SERVO_SUBMENU : STATE_SERVO_EDIT_JADWAL;
      } else if (currentMenuIndex == 5) {
        if      (incAction == INC_ACT_TYPE) menuState = STATE_INC_EDIT_TYPE;
        else if (incAction == INC_ACT_DAYS) menuState = STATE_INC_EDIT_DAYS;
        else                                menuState = STATE_INC_SUBMENU;
      } else {
        menuState = STATE_EDIT;
      }
    }
  }
}


// ==============================================================================
// 6. FUNGSI SENSOR & KONTROL PID
// ==============================================================================

/** @brief Membaca SHT30 dan menjalankan kalkulasi aktuator/PID. 
 * Kipas mengikuti pengaturan target walau sensor terputus. 
 */
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
  
  // Kickstart minimal untuk pemanas jika di bawah suhu target
  if (current_temp < setpoint_temp && pwm < 15) pwm = 120;
  
  ledcWrite(HEATER_PWM_PIN, pwm);
  if      (current_hum <= target_hum - 1.0) digitalWrite(RELAY_HUM_PIN, HIGH);
  else if (current_hum >= target_hum)        digitalWrite(RELAY_HUM_PIN, LOW);

  Serial.printf("[SHT30] T=%.1f H=%.1f | Heater PWM=%d Fan=%d%%\n",
                current_temp, current_hum, pwm, target_fan_speed);
}


// ==============================================================================
// 7. FUNGSI JARINGAN (WIFI & MQTT) - BERJALAN DI CORE 0
// ==============================================================================

void setupWifi() {
  wm.setConfigPortalBlocking(false);
  wm.setConnectTimeout(5);
  wm.setConnectRetries(1);
  wm.setConfigPortalTimeout(180);
  wm.setWiFiAutoReconnect(true);
  
  if (wm.autoConnect("ESP32_Incubator_AP")) Serial.println("[WiFi] Connected at boot");
  else Serial.println("[WiFi] lanjut boot tanpa WiFi (portal non-blocking)");
}

void handleWifiCheck() {
  if (WiFi.status() != WL_CONNECTED && !wm.getConfigPortalActive()) {
    Serial.println("[WiFi] Disconnected, starting portal...");
    wm.startConfigPortal("ESP32_Incubator_AP");
  }
}

/**
 * @brief Mengambil waktu dari NTP (zona waktu GMT_OFFSET_SEC) dan menitipkannya ke
 * variabel ntp*. Penulisan ke RTC dilakukan di loop (Core 1) agar I2C tidak bentrok.
 */
void fetchNtpTime() {
  configTime(GMT_OFFSET_SEC, 0, "pool.ntp.org", "time.google.com");
  struct tm ti;
  if (getLocalTime(&ti, 3000)) {
    ntpYear  = ti.tm_year + 1900;
    ntpMonth = ti.tm_mon + 1;
    ntpDay   = ti.tm_mday;
    ntpHour  = ti.tm_hour;
    ntpMin   = ti.tm_min;
    ntpSec   = ti.tm_sec;
    ntpResultReady = true;   // Set terakhir setelah semua field terisi
    Serial.println("[NTP] Waktu diterima");
  } else {
    Serial.println("[NTP] Gagal, coba lagi nanti");
  }
}

/**
 * @brief Menangani perintah MQTT masuk (JSON/Text)
 *
 * Perintah teks : "dev_getinfo", "candling_on", "candling_off", "inc_start", "inc_stop",
 *                 "capture"
 * Perintah JSON : {"target_temp":37.5, "target_hum":60, "fan_speed":80}
 *                 {"candling":true}                        -> ON, auto-off default
 *                 {"candling":true,"candling_sec":30}      -> ON, auto-off 30 detik
 *                 {"candling":true,"candling_sec":0}       -> ON, tanpa auto-off
 *                 {"candling":false}                       -> OFF
 *                 {"action":"capture"}                     -> ON, auto-off CANDLE_CAPTURE_SEC
 *                                                             (dashboard minta kamera ambil foto)
 *                 {"egg":"ayam"}                           -> pilih jenis telur (isi suhu/lembab/hari)
 *                 {"inc_days":25}                          -> ubah target hari inkubasi
 *                 {"inc_start":true} / {"inc_start":false} -> mulai / hentikan inkubasi
 */
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("[MQTT] in: "); Serial.println(msg);

  if (msg == "dev_getinfo") {
    StaticJsonDocument<448> resp;
    resp["target_temp"] = target_temp;
    resp["target_hum"]  = target_hum;
    resp["fan_speed"]   = target_fan_speed;
    resp["servo_mode"]  = servo_mode;
    resp["servo_interval"] = servo_interval_hours;
    resp["candling"]    = candle_on;
    resp["egg"]         = EGG_PROFILES[inc_profile].name;
    resp["inc_active"]  = inc_active;
    resp["inc_day"]     = inc_day;
    resp["inc_days"]    = inc_days;
    resp["lockdown"]    = inc_lockdown;
    resp["setpoint"]    = setpoint_temp;
    char buf[448]; serializeJson(resp, buf);
    client.publish(mqtt_topic_data, buf);
    return;
  }

  // Perintah teks sederhana
  if (msg == "candling_on")  { candle_cmd_sec = CANDLE_DEFAULT_SEC; candle_cmd = 1; return; }
  if (msg == "candling_off") { candle_cmd = 0; return; }
  if (msg == "capture")      { candle_cmd_sec = CANDLE_CAPTURE_SEC; candle_cmd = 1; return; }
  if (msg == "inc_start")    { inc_cmd_start = 1; return; }
  if (msg == "inc_stop")     { inc_cmd_start = 0; return; }

  // Fallback: deteksi perintah capture dari teks mentah.
  // Jika payload dashboard terlalu besar sehingga parsing JSON di bawah gagal,
  // LED candling tetap menyala hanya dengan mencari substring "action ... capture".
  if (msg.indexOf("\"action\"") >= 0 && msg.indexOf("capture") >= 0) {
    candle_cmd_sec = CANDLE_CAPTURE_SEC;
    candle_cmd = 1;
  }

  StaticJsonDocument<512> doc;
  if (!deserializeJson(doc, payload, length)) {
    // Capture dari dashboard (tombol "Ambil Foto Sekarang"): nyalakan LED
    // candling sesaat sebagai lampu sorot telur, mati otomatis sendiri.
    if (doc.containsKey("action")) {
      String act = doc["action"].as<const char*>(); act.toLowerCase();
      if (act == "capture") {
        candle_cmd_sec = CANDLE_CAPTURE_SEC;
        candle_cmd = 1;
      }
    }

    // Candling (dieksekusi di loop/Core 1 lewat updateCandling)
    if (doc.containsKey("candling")) {
      bool on = false;
      JsonVariant v = doc["candling"];
      if      (v.is<bool>())        on = v.as<bool>();
      else if (v.is<int>())         on = v.as<int>() != 0;
      else if (v.is<const char*>()) {
        String s = v.as<const char*>(); s.toLowerCase();
        on = (s == "on" || s == "true" || s == "1");
      }
      candle_cmd_sec = doc.containsKey("candling_sec") ? doc["candling_sec"].as<uint32_t>()
                                                       : CANDLE_DEFAULT_SEC;
      candle_cmd = on ? 1 : 0;   // Diset paling akhir setelah durasi terisi
    }

    // Inkubasi (dieksekusi di loop/Core 1 lewat updateIncubation).
    // Urutan: jenis telur dulu, lalu override hari/suhu/lembab di bawah bila ada.
    if (doc.containsKey("egg")) {
      int idx = egg_find(doc["egg"].as<const char*>());
      if (idx >= 0) inc_cmd_profile = idx;
      else Serial.println("[MQTT] Jenis telur tidak dikenal");
    }
    if (doc.containsKey("inc_days")) {
      int d = doc["inc_days"].as<int>();
      if (d >= EGG_DAYS_MIN && d <= EGG_DAYS_MAX) inc_cmd_days = d;
    }
    if (doc.containsKey("inc_start")) {
      JsonVariant v = doc["inc_start"];
      bool go = false;
      if      (v.is<bool>()) go = v.as<bool>();
      else if (v.is<int>())  go = v.as<int>() != 0;
      inc_cmd_start = go ? 1 : 0;
    }

    // Pengaturan inkubator (hanya simpan ke NVS bila ada yang berubah).
    // Dibatasi sama seperti menu agar nilai ekstrem dari jaringan tidak berbahaya.
    bool settingsChanged = false;
    if (doc.containsKey("target_temp")) {
      double v = doc["target_temp"].as<double>();
      target_temp = constrain(v, 20.0, 50.0);
      cache.setTemp = -999; settingsChanged = true;
    }
    if (doc.containsKey("target_hum")) {
      double v = doc["target_hum"].as<double>();
      target_hum = constrain(v, 30.0, 90.0);
      cache.setHum = -999; settingsChanged = true;
    }
    if (doc.containsKey("fan_speed") || doc.containsKey("target_fan")) {
      int v = doc.containsKey("fan_speed") ? doc["fan_speed"].as<int>()
                                           : doc["target_fan"].as<int>();
      target_fan_speed = constrain(v, 0, 100);
      settingsChanged = true;
    }
    // Servo dari dashboard: mode (0=JADWAL, 1=SWING) dan interval jadwal (1-24 jam).
    // Dieksekusi di loop/Core 1 lewat updateServo lewat servo_cmd_*.
    if (doc.containsKey("servo_mode")) {
      int v = doc["servo_mode"].as<int>();
      servo_cmd_mode = (v == SERVO_MODE_SWING) ? SERVO_MODE_SWING : SERVO_MODE_JADWAL;
    }
    if (doc.containsKey("servo_interval")) {
      int v = doc["servo_interval"].as<int>();
      if (v >= 1 && v <= 24) servo_cmd_interval = v;
    }
    if (settingsChanged) saveAll();
  }
}

void reconnect() {
  unsigned long now = millis();
  if (now - lastMqttReconnectAttempt < MQTT_RECONNECT_MS) return;
  lastMqttReconnectAttempt = now;
  
  if (WiFi.status() != WL_CONNECTED) return;
  
  Serial.print("[MQTT] Connecting...");
  String cid = String(device_id);
  if (client.connect(cid.c_str(), mqtt_user, mqtt_pass)) {
    Serial.println(" OK");
    client.subscribe(mqtt_topic_con);
  } else {
    Serial.print(" FAIL rc="); Serial.println(client.state());
  }
}

void publishSensorData() {
  if (!sensorOK) return;
  if (WiFi.status() == WL_CONNECTED && client.connected() && !isnan(current_temp)) {
    StaticJsonDocument<448> doc;
    doc["temperature"]  = current_temp;
    doc["humidity"]     = current_hum;
    doc["target_temp"]  = target_temp;
    doc["target_hum"]   = target_hum;
    doc["fan_speed"]    = target_fan_speed;
    doc["servo_mode"]   = servo_mode;
    doc["servo_interval"] = servo_interval_hours;
    doc["servo_pos"]    = current_servo_pos;
    doc["candling"]     = candle_on;
    doc["egg"]          = EGG_PROFILES[inc_profile].name;
    doc["inc_active"]   = inc_active;
    doc["inc_day"]      = inc_day;
    doc["inc_days"]     = inc_days;
    doc["lockdown"]     = inc_lockdown;
    doc["setpoint"]     = setpoint_temp;
    char buf[448]; serializeJson(doc, buf);
    client.publish(mqtt_topic_data, buf);
  }
}

/** @brief Task FreeRTOS untuk menghandle network secara terpisah */
void networkTask(void* pv) {
  setupWifi();

  espClient.setInsecure();
  espClient.setTimeout(3);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setSocketTimeout(3);
  client.setKeepAlive(30);
  client.setBufferSize(512);   // Payload JSON status/dev_getinfo bisa > 256 byte (default PubSubClient)

  unsigned long now = millis();
  unsigned long lastWifi = now, lastPub = now;
  unsigned long lastNtp = now - NTP_RETRY_MS;   // Boleh langsung mencoba saat WiFi siap
  lastMqttReconnectAttempt = now - MQTT_RECONNECT_MS + 3000;

  for (;;) {
    wm.process();
    now = millis();

    if (now - lastWifi >= 5000) { lastWifi = now; handleWifiCheck(); }
    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) reconnect();
      else                     client.loop();

      // Kalibrasi RTC via NTP (awal upload / tiap 30 hari)
      if (rtcNeedCalibration && !ntpResultReady && now - lastNtp >= NTP_RETRY_MS) {
        lastNtp = now;
        fetchNtpTime();
      }
    }
    if (now - lastPub >= 5000) { lastPub = now; publishSensorData(); }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


// ==============================================================================
// 8. SETUP & LOOP UTAMA
// ==============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("[BOOT] Starting...");

  // Cek & tampilkan ID perangkat di awal (dari secret.h)
  initDeviceId();
  Serial.printf("[ID] Device ID : %s\n", device_id);
  Serial.printf("[ID] Topic data: %s\n", mqtt_topic_data);
  Serial.printf("[ID] Topic con : %s\n", mqtt_topic_con);

  // Load memori NVS di awal untuk UI Placeholder
  preferences.begin("incubator", false);
  target_temp      = preferences.getDouble("t_temp", 37.0);
  target_hum       = preferences.getDouble("t_hum",  60.0);
  target_fan_speed = preferences.getInt("t_fan",     100);
  if (target_fan_speed > 100) target_fan_speed = 100;

  // Load status inkubasi (sebelum initRtc agar kalibrasi RTC bisa menggeser waktu mulai)
  inc_profile = constrain(preferences.getInt("i_prof", EGG_CUSTOM_INDEX), 0, EGG_PROFILE_COUNT - 1);
  inc_days    = constrain(preferences.getInt("i_days", 21), EGG_DAYS_MIN, EGG_DAYS_MAX);
  inc_start   = preferences.getUInt("i_start", 0);
  inc_active  = preferences.getBool("i_act", false) && inc_start != 0;
  Serial.printf("[INC] Jenis=%s, target=%d hari, aktif=%d\n",
                EGG_PROFILES[inc_profile].name, inc_days, inc_active);

  // Inisialisasi Layar TFT
  tftSPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);
  tft.initR(INITR_BLACKTAB);
  tft.setSPISpeed(27000000);
  tft.setRotation(0);
  tft.setTextWrap(false);
  tft.fillScreen(C_BG);
  tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(20, 75);
  tft.print("Initializing...");
  Serial.println("[TFT] OK");

  // Inisialisasi RTC (kalibrasi awal dari waktu kompilasi bila firmware baru di-upload)
  initRtc();

  drawChrome();
  setupEncoder();

  // Failsafe & Konfigurasi Pin Awal
  pinMode(RELAY_HUM_PIN, OUTPUT);
  digitalWrite(RELAY_HUM_PIN, LOW);
  pinMode(LED_CANDLE_PIN, OUTPUT);       // LED candling: selalu mati saat boot
  digitalWrite(LED_CANDLE_PIN, LOW);
  ledcAttach(FAN_PWM_PIN,    5000, 8);
  ledcAttach(HEATER_PWM_PIN, 5000, 8);
  ledcWrite(HEATER_PWM_PIN, 0);
  ledcWrite(FAN_PWM_PIN, map(target_fan_speed, 0, 100, 0, 255));

  myPID.setBangBang(0.5);

  // Hitung hari inkubasi awal supaya langsung tampil di layar pertama
  updateIncubation();

  // Update layar pertama kali tanpa menunggu data sensor
  updateSensorDisplay();
  updateDisplay();
  setupServo();

  // Tes Sensor Awal
  if (!sht30.begin(0x44)) {
    Serial.println("[SHT30] NOT FOUND at boot, lanjut (retry di loop)");
    sensorOK = false;
    lastSensorRetry = millis();
  } else {
    Serial.println("[SHT30] OK");
    sensorOK = true;
  }

  lastSensorRead = millis() - SENSOR_READ_MS + 500;

  // Mengisolasi Network ke Core 0 (Loop tetap di Core 1)
  xTaskCreatePinnedToCore(networkTask, "net", 10240, NULL, 1, NULL, 0);

  Serial.println("[BOOT] Done. UI siap, sensor/WiFi/MQTT jalan background.");
}

void loop() {
  handleMenu();
  updateCandling();
  updateIncubation();
  updateSensorDisplay();
  updateRtc();

  unsigned long now = millis();
  
  if (displayDirty && (now - lastDisplayUpdate >= DISPLAY_COOLDOWN_MS)) {
    updateDisplay();
    lastDisplayUpdate = now;
    displayDirty = false;
  }

  updateServo();

  if (now - lastSensorRead >= SENSOR_READ_MS) {
    lastSensorRead = now;
    readSensorAndControl();
  }
}
