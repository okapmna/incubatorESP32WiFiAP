#pragma once
#include "config.h"

// 0: Set Target Temp, 1: Set Target Hum
int currentMenuIndex = 0; 
const int maxMenuItems = 2;
bool isEditing = false;
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

void updateOLEDDisplay() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  
  // WiFi Status (Top Right)
  oled.setTextSize(1);
  oled.setCursor(80, 0); 
  if (WiFi.status() == WL_CONNECTED) {
    oled.print("WiFi:OK");
  } else if (wm.getConfigPortalActive()) {
    oled.print("WiFi:AP");
  } else {
    oled.print("WiFi:DC");
  }

  // Menu Title
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("--- MENU ---");

  // Display Menus
  
  // Menu 1: Set Target Temp
  if (isEditing && currentMenuIndex == 0) {
    oled.setTextSize(2);
    oled.setCursor(0, 20);
    oled.print(">T:");
    oled.print(target_temp, 1);
  } else {
    oled.setTextSize(1);
    oled.setCursor(0, 24);
    if (currentMenuIndex == 0) oled.print("> ");
    else oled.print("  ");
    oled.print("Set Temp: ");
    oled.print(target_temp, 1);
    oled.print(" C");
  }
  
  // Menu 2: Set Target Hum
  if (isEditing && currentMenuIndex == 1) {
    oled.setTextSize(2);
    oled.setCursor(0, 40);
    oled.print(">H:");
    oled.print(target_hum, 0);
  } else {
    oled.setTextSize(1);
    oled.setCursor(0, 40);
    if (currentMenuIndex == 1) oled.print("> ");
    else oled.print("  ");
    oled.print("Set Hum: ");
    oled.print(target_hum, 0);
    oled.print(" %");
  }

  // Instruction (if editing)
  if (isEditing) {
    oled.setTextSize(1);
    oled.setCursor(0, 56);
    oled.print("Putar & Tekan (Save)");
  }

  oled.display();
}

void handleMenu() {
  // Check button press
  if (buttonPressed) {
    buttonPressed = false;
    isEditing = !isEditing;
    if (!isEditing) {
      // Save changes to NVS on edit end
      preferences.putDouble("t_temp", target_temp);
      preferences.putDouble("t_hum", target_hum);
    }
    updateOLEDDisplay();
  }

  // Check encoder rotation
  if (encoderValue != lastEncoderValue) {
    int diff = encoderValue - lastEncoderValue;
    lastEncoderValue = encoderValue;
    
    if (isEditing) {
      if (currentMenuIndex == 0) {
        // Edit Temperature
        target_temp += (diff > 0) ? 0.1 : -0.1;
        // Limit bounds if necessary, e.g. 20 to 50
        if (target_temp < 20.0) target_temp = 20.0;
        if (target_temp > 50.0) target_temp = 50.0;
      } else if (currentMenuIndex == 1) {
        // Edit Humidity
        target_hum += (diff > 0) ? 1.0 : -1.0;
        // Limit bounds, e.g. 30 to 90
        if (target_hum < 30.0) target_hum = 30.0;
        if (target_hum > 90.0) target_hum = 90.0;
      }
    } else {
      // Navigate Menu
      currentMenuIndex += (diff > 0) ? 1 : -1;
      if (currentMenuIndex < 0) currentMenuIndex = maxMenuItems - 1;
      if (currentMenuIndex >= maxMenuItems) currentMenuIndex = 0;
    }
    updateOLEDDisplay();
  }
  
  // Refresh display periodically to update WiFi status
  unsigned long now = millis();
  if (now - lastDisplayUpdate >= 1000) {
    lastDisplayUpdate = now;
    updateOLEDDisplay();
  }
}
