#pragma once
#include "config.h"

enum MenuState {
  STATE_NAVIGATE,
  STATE_EDIT,
  STATE_CONFIRM,
  STATE_SERVO_SUBMENU,
  STATE_SERVO_EDIT_JADWAL
};

MenuState menuState = STATE_NAVIGATE;
int currentMenuIndex = 0; // 0: Set Target Temp, 1: Set Target Hum, 2: Set Fan Speed, 3: Servo
const int maxMenuItems = 4;
int currentSubmenuIndex = 0; // 0: JADWAL, 1: SWING, 2: KEMBALI
int lastEncoderValue = 0;
unsigned long lastDisplayUpdate = 0;

void IRAM_ATTR isr_encoder() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 5) {
    if (digitalRead(ROTARY_CLK_PIN) == digitalRead(ROTARY_DT_PIN)) {
      encoderValue++;
    } else {
      encoderValue--;
    }
    lastInterruptTime = interruptTime;
  }
}

void IRAM_ATTR isr_button() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) {
    buttonPressed = true;
    lastInterruptTime = interruptTime;
  }
}

void setupMenu() {
  pinMode(ROTARY_CLK_PIN, INPUT_PULLUP);
  pinMode(ROTARY_DT_PIN, INPUT_PULLUP);
  pinMode(ROTARY_SW_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK_PIN), isr_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_SW_PIN), isr_button, FALLING);
}

// Function to center text
void printCenteredText(String text, int y, int size) {
  oled.setTextSize(size);
  int16_t x1, y1;
  uint16_t w, h;
  oled.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (128 - w) / 2;
  if (x < 0) x = 0;
  oled.setCursor(x, y);
  oled.print(text);
}

void updateOLEDDisplay() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  
  if (menuState == STATE_NAVIGATE) {
    // WiFi Status (Top Right)
    oled.setTextSize(1);
    oled.setCursor(80, 0); 
    if (WiFi.status() == WL_CONNECTED) {
      oled.print("WIFI: OK");
    } else if (wm.getConfigPortalActive()) {
      oled.print("WIFI: AP");
    } else {
      oled.print("WIFI: DC");
    }

    if (currentMenuIndex == 0) {
      // Hovered: Size 2
      oled.setTextSize(2);
      oled.setCursor(0, 16);
      oled.print("> SET TEMP");
      // Unhovered: Size 1
      oled.setTextSize(1);
      oled.setCursor(20, 36);
      oled.print(" SET HUMI");
    } else if (currentMenuIndex == 1) {
      // Hovered: Size 2
      oled.setTextSize(2);
      oled.setCursor(0, 16);
      oled.print("> SET HUMI");
      // Unhovered: Size 1
      oled.setTextSize(1);
      oled.setCursor(20, 36);
      oled.print(" FAN SPD");
    } else if (currentMenuIndex == 2) {
      // Hovered: Size 2
      oled.setTextSize(2);
      oled.setCursor(0, 16);
      oled.print("> FAN SPD");
      // Unhovered: Size 1
      oled.setTextSize(1);
      oled.setCursor(20, 36);
      oled.print(" SERVO");
    } else {
      // Hovered: Size 2
      oled.setTextSize(2);
      oled.setCursor(0, 16);
      oled.print("> SERVO");
      // Unhovered: Size 1
      oled.setTextSize(1);
      oled.setCursor(20, 36);
      oled.print(" SET TEMP");
    }
    printCenteredText("Putar untuk memilih", 56, 1);
    
  } else if (menuState == STATE_EDIT) {
    if (currentMenuIndex == 0) {
      printCenteredText("TEMPERATURE", 0, 1);
    } else if (currentMenuIndex == 1) {
      printCenteredText("HUMIDIFIER", 0, 1);
    } else {
      printCenteredText("FAN SPEED", 0, 1);
    }
    
    String valStr = "";
    if (currentMenuIndex == 0) {
      valStr += String(target_temp, 1) + (char)248 + "C";
    } else if (currentMenuIndex == 1) {
      valStr += String(target_hum, 0) + "%";
    } else {
      valStr += String(target_fan_speed) + "%";
    }
    
    printCenteredText(valStr, 22, 3);
    printCenteredText("ATUR TINGKAT", 56, 1);
    
  } else if (menuState == STATE_SERVO_SUBMENU) {
    printCenteredText("SERVO MENU", 0, 1);
    if (currentSubmenuIndex == 0) {
      oled.setTextSize(2);
      oled.setCursor(0, 16);
      oled.print("> JADWAL");
      oled.setTextSize(1);
      oled.setCursor(20, 36);
      oled.print(" SWING");
    } else if (currentSubmenuIndex == 1) {
      oled.setTextSize(2);
      oled.setCursor(0, 16);
      oled.print("> SWING");
      oled.setTextSize(1);
      oled.setCursor(20, 36);
      oled.print(" KEMBALI");
    } else {
      oled.setTextSize(2);
      oled.setCursor(0, 16);
      oled.print("> KEMBALI");
      oled.setTextSize(1);
      oled.setCursor(20, 36);
      oled.print(" JADWAL");
    }
    printCenteredText("Pilih mode servo", 56, 1);

  } else if (menuState == STATE_SERVO_EDIT_JADWAL) {
    printCenteredText("INTERVAL JADWAL", 0, 1);
    String valStr = String(servo_interval_hours) + " JAM";
    printCenteredText(valStr, 22, 3);
    printCenteredText("ATUR JAM", 56, 1);
    
  } else if (menuState == STATE_CONFIRM) {
    printCenteredText("APAKAH BENAR?", 0, 1);
    
    String valStr = "";
    if (currentMenuIndex == 0) {
      valStr += String(target_temp, 1) + (char)248 + "C";
    } else if (currentMenuIndex == 1) {
      valStr += String(target_hum, 0) + "%";
    } else if (currentMenuIndex == 2) {
      valStr += String(target_fan_speed) + "%";
    } else {
      if (servo_mode == SERVO_MODE_SWING) {
        valStr += "SWING";
      } else {
        valStr += String(servo_interval_hours) + " JAM";
      }
    }
    
    printCenteredText(valStr, 22, 3);
    printCenteredText("TEKAN LAGI UNTUK OK", 56, 1);
  }

  oled.display();
}

void handleMenu() {
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
      if (currentSubmenuIndex == 0) {
        servo_mode = SERVO_MODE_JADWAL;
        menuState = STATE_SERVO_EDIT_JADWAL;
      } else if (currentSubmenuIndex == 1) {
        servo_mode = SERVO_MODE_SWING;
        menuState = STATE_CONFIRM;
      } else {
        menuState = STATE_NAVIGATE;
        currentMenuIndex = 3;
      }
    } else if (menuState == STATE_SERVO_EDIT_JADWAL) {
      menuState = STATE_CONFIRM;
    } else if (menuState == STATE_CONFIRM) {
      // Save changes to NVS
      if (currentMenuIndex == 0 || currentMenuIndex == 1 || currentMenuIndex == 2) {
        preferences.putDouble("t_temp", target_temp);
        preferences.putDouble("t_hum", target_hum);
        preferences.putInt("t_fan", target_fan_speed);
      } else if (currentMenuIndex == 3) {
        preferences.putInt("s_mode", servo_mode);
        preferences.putInt("s_int", servo_interval_hours);
        
        // Re-initialize scheduled timer / target pos when mode is saved
        preferences.putBool("s_dir", servo_direction_cw);
        if (servo_mode == SERVO_MODE_JADWAL) {
          target_servo_pos = servo_direction_cw ? 180.0 : 0.0;
          last_servo_mode_change = millis();
        } else {
          target_servo_pos = 180.0;
        }
      }
      menuState = STATE_NAVIGATE;
    }
    updateOLEDDisplay();
  }

  if (abs(encoderValue - lastEncoderValue) >= 2) {
    // Reverse direction by subtracting encoderValue from lastEncoderValue
    int diff = lastEncoderValue - encoderValue; 
    lastEncoderValue = encoderValue;
    
    if (menuState == STATE_NAVIGATE) {
      currentMenuIndex += (diff > 0) ? 1 : -1;
      if (currentMenuIndex < 0) currentMenuIndex = maxMenuItems - 1;
      if (currentMenuIndex >= maxMenuItems) currentMenuIndex = 0;
    } else if (menuState == STATE_SERVO_SUBMENU) {
      currentSubmenuIndex += (diff > 0) ? 1 : -1;
      if (currentSubmenuIndex < 0) currentSubmenuIndex = 2;
      if (currentSubmenuIndex > 2) currentSubmenuIndex = 0;
    } else if (menuState == STATE_SERVO_EDIT_JADWAL) {
      servo_interval_hours += (diff > 0) ? 1 : -1;
      if (servo_interval_hours < 1) servo_interval_hours = 1;
      if (servo_interval_hours > 24) servo_interval_hours = 24;
    } else if (menuState == STATE_EDIT || menuState == STATE_CONFIRM) {
      // If rotated during CONFIRM state, go back to EDIT state
      if (menuState == STATE_CONFIRM) {
        if (currentMenuIndex == 3) {
          menuState = (servo_mode == SERVO_MODE_SWING) ? STATE_SERVO_SUBMENU : STATE_SERVO_EDIT_JADWAL;
        } else {
          menuState = STATE_EDIT;
        }
      } else { // STATE_EDIT
        if (currentMenuIndex == 0) {
          target_temp += (diff > 0) ? 0.1 : -0.1;
          if (target_temp < 20.0) target_temp = 20.0;
          if (target_temp > 50.0) target_temp = 50.0;
        } else if (currentMenuIndex == 1) {
          target_hum += (diff > 0) ? 1.0 : -1.0;
          if (target_hum < 30.0) target_hum = 30.0;
          if (target_hum > 90.0) target_hum = 90.0;
        } else if (currentMenuIndex == 2) {
          target_fan_speed += (diff > 0) ? 5 : -5;
          if (target_fan_speed < 0) target_fan_speed = 0;
          if (target_fan_speed > 100) target_fan_speed = 100;
        }
      }
    }
    updateOLEDDisplay();
  }
  
  unsigned long now = millis();
  if (now - lastDisplayUpdate >= 1000) {
    lastDisplayUpdate = now;
    updateOLEDDisplay();
  }
}
