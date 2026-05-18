#pragma once
#include "config.h"

enum MenuState {
  STATE_NAVIGATE,
  STATE_EDIT,
  STATE_CONFIRM
};

MenuState menuState = STATE_NAVIGATE;
int currentMenuIndex = 0; // 0: Set Target Temp, 1: Set Target Hum
const int maxMenuItems = 2;
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
      oled.print(">SET TEMP");
      // Unhovered: Size 1
      oled.setTextSize(1);
      oled.setCursor(20, 36);
      oled.print(">SET HUMID");
    } else {
      // Hovered: Size 2
      oled.setTextSize(2);
      oled.setCursor(0, 16);
      oled.print(">SET HUMID");
      // Unhovered: Size 1
      oled.setTextSize(1);
      oled.setCursor(20, 36);
      oled.print(">SET TEMP");
    }
    printCenteredText("Putar untuk memilih", 56, 1);
    
  } else if (menuState == STATE_EDIT) {
    printCenteredText(currentMenuIndex == 0 ? "TEMPERATURE" : "HUMIDIFIER", 0, 1);
    
    String valStr = "";
    if (currentMenuIndex == 0) {
      valStr += String(target_temp, 1) + (char)248 + "C";
    } else {
      valStr += String(target_hum, 0) + "%";
    }
    
    printCenteredText(valStr, 22, 3);
    printCenteredText("ATUR TINGKAT", 56, 1);
    
  } else if (menuState == STATE_CONFIRM) {
    printCenteredText("APAKAH BENAR?", 0, 1);
    
    String valStr = "";
    if (currentMenuIndex == 0) {
      valStr += String(target_temp, 1) + (char)248 + "C";
    } else {
      valStr += String(target_hum, 0) + "%";
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
      menuState = STATE_EDIT;
    } else if (menuState == STATE_EDIT) {
      menuState = STATE_CONFIRM;
    } else if (menuState == STATE_CONFIRM) {
      // Save changes to NVS
      preferences.putDouble("t_temp", target_temp);
      preferences.putDouble("t_hum", target_hum);
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
    } else if (menuState == STATE_EDIT || menuState == STATE_CONFIRM) {
      // If rotated during CONFIRM state, go back to EDIT state
      if (menuState == STATE_CONFIRM) {
        menuState = STATE_EDIT;
      }
      
      if (currentMenuIndex == 0) {
        target_temp += (diff > 0) ? 0.1 : -0.1;
        if (target_temp < 20.0) target_temp = 20.0;
        if (target_temp > 50.0) target_temp = 50.0;
      } else {
        target_hum += (diff > 0) ? 1.0 : -1.0;
        if (target_hum < 30.0) target_hum = 30.0;
        if (target_hum > 90.0) target_hum = 90.0;
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
