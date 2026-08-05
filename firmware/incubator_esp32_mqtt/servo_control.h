#pragma once
#include "config.h"



// ================= SERVO PARAMETERS CONFIG =================
// Batasan gerakan sudut fisik servo (Awal: 20, Tengah: 90, Akhir: 160)
#define SERVO_MIN_ANGLE   35.0
#define SERVO_MAX_ANGLE   145.0

// Batasan sinyal pulsa (duty cycle 16-bit pada 50Hz)
// - Sinyal 0.5ms - 2.5ms (Duty 1638 - 8192) seringkali membuat servo murah (SG90/MG996R) mentok fisik dan bergetar/dengung.
// - Sinyal standar industri adalah 1.0ms - 2.0ms (Duty 3277 - 6554).
// Anda bisa menyesuaikan nilai ini jika servo Anda berdengung atau tidak merespons perubahan sudut.
#define SERVO_MIN_DUTY    1638  // Set ke 3277 jika servo mentok/berdengung di batas bawah
#define SERVO_MAX_DUTY    8192  // Set ke 6554 jika servo mentok/berdengung di batas atas
// ===========================================================

uint32_t angleToDuty(float angle) {
  // Memetakan sudut input virtual (0 - 180) ke rentang sudut fisik yang dibatasi (SERVO_MIN_ANGLE - SERVO_MAX_ANGLE)
  float physical_angle = SERVO_MIN_ANGLE + (angle / 180.0) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
  
  // Memetakan sudut fisik ke duty cycle yang sesuai
  return SERVO_MIN_DUTY + (uint32_t)((physical_angle / 180.0) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY));
}

void setupServo() {
  ledcAttach(SERVO_PIN, 50, 16); // 50Hz, 16-bit resolution
  
  // Load preferences
  preferences.begin("incubator", false);
  servo_mode = preferences.getInt("s_mode", SERVO_MODE_JADWAL);
  servo_interval_hours = preferences.getInt("s_int", 3);
  servo_direction_cw = preferences.getBool("s_dir", true);
  
  if (servo_mode == SERVO_MODE_JADWAL) {
    current_servo_pos = servo_direction_cw ? 180.0 : 0.0;
    target_servo_pos = current_servo_pos;
  } else {
    current_servo_pos = 0.0;
    target_servo_pos = 180.0;
  }
  
  ledcWrite(SERVO_PIN, angleToDuty(current_servo_pos));
  last_servo_mode_change = millis();
}

void updateServo() {
  unsigned long now = millis();
  
  if (servo_mode == SERVO_MODE_SWING) {
    if (now - last_servo_update_time >= 30) {
      last_servo_update_time = now;
      if (servo_direction_cw) {
        current_servo_pos += 1.0;
        if (current_servo_pos >= 180.0) {
          current_servo_pos = 180.0;
          servo_direction_cw = false;
        }
      } else {
        current_servo_pos -= 1.0;
        if (current_servo_pos <= 0.0) {
          current_servo_pos = 0.0;
          servo_direction_cw = true;
        }
      }
      ledcWrite(SERVO_PIN, angleToDuty(current_servo_pos));
    }
  } else if (servo_mode == SERVO_MODE_JADWAL) {
    // Scheduled mode: toggles direction every few hours
    unsigned long interval_ms = (unsigned long)servo_interval_hours * 3600ULL * 1000ULL;
    if (now - last_servo_mode_change >= interval_ms) {
      last_servo_mode_change = now;
      servo_direction_cw = !servo_direction_cw;
      
      // Save direction to survive reboot during sweep
      preferences.begin("incubator", false);
      preferences.putBool("s_dir", servo_direction_cw);
      
      target_servo_pos = servo_direction_cw ? 180.0 : 0.0;
    }
    
    // Slow movement to target position
    if (abs(current_servo_pos - target_servo_pos) > 0.01) {
      if (now - last_servo_update_time >= 30) {
        last_servo_update_time = now;
        if (current_servo_pos < target_servo_pos) {
          current_servo_pos += 1.0;
          if (current_servo_pos > target_servo_pos) current_servo_pos = target_servo_pos;
        } else {
          current_servo_pos -= 1.0;
          if (current_servo_pos < target_servo_pos) current_servo_pos = target_servo_pos;
        }
        ledcWrite(SERVO_PIN, angleToDuty(current_servo_pos));
      }
    }
  }
}
