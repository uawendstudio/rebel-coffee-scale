#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2_timer(U8G2_R0, 17, 16, 20);
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2_weight(U8G2_R0, 21, 14, 15);

NAU7802 myScale;

const int TARE_BTN = 2;
const int TIME_BTN = 3;
const int AUTO_BTN = 22;
const int BATTERY_PIN = 26; 

float calibrationFactor = 1091.26; 
float displayWeight = 0.0;
unsigned long timerStartMillis = 0;
bool timerRunning = false;
bool autoStartEnabled = false;
bool isCalibrating = false;

// --- BATTERY VARIABLES ---
float smoothedBatteryV = 8.0; 
const float VOLTAGE_CORRECTION = 1.063; 
unsigned long lastBatteryCheck = 0; 

unsigned long autoBtnPressStart = 0;
bool autoBtnHeld = false;

unsigned long lastDisplayUpdate = 0;
const int DISPLAY_INTERVAL = 120;

void setup() {
  delay(50); 
  
  // --- LOAD CALIBRATION FROM FLASH ---
  EEPROM.begin(256);
  float storedFactor = 0;
  EEPROM.get(0, storedFactor);
  // Protection from corrupt memory data
  if (!isnan(storedFactor) && abs(storedFactor) > 100.0) {
    calibrationFactor = storedFactor;
  }

  Wire.setSDA(0); Wire.setSCL(1);
  Wire.begin(); 
  Wire.setClock(400000); 

  SPI.begin();
  u8g2_timer.setBusClock(4000000); u8g2_weight.setBusClock(4000000);
  u8g2_timer.begin(); u8g2_weight.begin();

  if (myScale.begin(Wire)) {
    myScale.setLDO(NAU7802_LDO_2V7);
    myScale.setGain(NAU7802_GAIN_128);
    myScale.setSampleRate(NAU7802_SPS_40);
    myScale.calibrateAFE();
        
    delay(500);
    while(myScale.available()) myScale.getReading(); 
    
    myScale.calculateZeroOffset(15);
  }

  pinMode(TARE_BTN, INPUT_PULLUP);
  pinMode(TIME_BTN, INPUT_PULLUP);
  pinMode(AUTO_BTN, INPUT_PULLUP);
  
  analogReadResolution(12); 
  
  float raw = analogRead(BATTERY_PIN);
  smoothedBatteryV = (raw / 4095.0) * 3.3 * (147.0 / 47.0) * VOLTAGE_CORRECTION;
}

void loop() {
  unsigned long now = millis();

  // --- BATTERY READ ---
  if (now - lastBatteryCheck >= 10000) {
    lastBatteryCheck = now;
    float raw = analogRead(BATTERY_PIN); 
    float currentV = (raw / 4095.0) * 3.3 * (147.0 / 47.0) * VOLTAGE_CORRECTION;
    if (currentV > 5.0) {
      smoothedBatteryV = (smoothedBatteryV * 0.8) + (currentV * 0.2);
    }
  }

  // --- BUTTONS ---
  bool autoBtnState = (digitalRead(AUTO_BTN) == LOW);
  if (autoBtnState) {
    if (autoBtnPressStart == 0) autoBtnPressStart = now;
    if (now - autoBtnPressStart > 2000 && !autoBtnHeld) {
      isCalibrating = !isCalibrating;
      autoBtnHeld = true;
      if (isCalibrating) { 
        myScale.calculateZeroOffset(5); 
        timerRunning = false; 
        autoStartEnabled = false; 
      }
    }
  } else {
    if (autoBtnPressStart != 0 && !autoBtnHeld && !isCalibrating) {
      autoStartEnabled = !autoStartEnabled;
      if (autoStartEnabled) { 
        myScale.calculateZeroOffset(5); 
        displayWeight = 0; 
        timerRunning = false; 
      } else { 
        timerRunning = false; 
        timerStartMillis = 0; 
      }
    }
    autoBtnPressStart = 0; 
    autoBtnHeld = false;
  }

// --- CALIBRATION ---
  if (isCalibrating && digitalRead(TARE_BTN) == LOW) {
      u8g2_weight.clearBuffer(); 
      u8g2_weight.drawStr(0, 30, "1. EMPTY & TARE"); 
      u8g2_weight.sendBuffer();
      
      delay(2000);
      
      long rawZero = 0;
      for(int i=0; i<32; i++) {
        while(!myScale.available()) delay(1);
        rawZero += myScale.getReading(); 
      }
      rawZero /= 32;
      
      myScale.setZeroOffset(rawZero);

      u8g2_weight.clearBuffer(); 
      u8g2_weight.drawStr(0, 30, "2. PUT 100.8G"); 
      u8g2_weight.sendBuffer();
      
      delay(8000);

      u8g2_weight.clearBuffer(); 
      u8g2_weight.drawStr(0, 30, "MEASURING..."); 
      u8g2_weight.sendBuffer();

      long rawWithWeight = 0;
      for(int i=0; i<32; i++) {
        while(!myScale.available()) delay(1);
        rawWithWeight += myScale.getReading(); 
      }
      rawWithWeight /= 32;


      float newFactor = (float)abs(rawWithWeight - rawZero) / 100.8;
      

      if (newFactor > 50.0) {
        calibrationFactor = newFactor;
        EEPROM.put(0, calibrationFactor);
        EEPROM.commit();
        u8g2_weight.clearBuffer(); 
        u8g2_weight.drawStr(0, 30, "SAVED!"); 
      } else {
        u8g2_weight.clearBuffer(); 
        u8g2_weight.drawStr(0, 30, "ERROR: LOW SIGNAL"); 
      }
      
      u8g2_weight.sendBuffer();
      delay(2000);
      isCalibrating = false;
      displayWeight = 100.8;
  } else if (!isCalibrating) {

    if (digitalRead(TARE_BTN) == LOW) { 
      myScale.calculateZeroOffset(15); 
      displayWeight = 0; 
      delay(250); 
    }
    if (digitalRead(TIME_BTN) == LOW) { 
      timerRunning = !timerRunning; 
      if(timerRunning) timerStartMillis = millis(); 
      delay(250); 
    }
  }

  // --- WEIGHT READING (STICKY 0.1G FILTER) ---
  if (myScale.available()) {
    long rawValue = myScale.getReading(); // Direct reading for speed
    float currentRaw = (float)(rawValue - myScale.getZeroOffset()) / calibrationFactor;
    float diff = abs(currentRaw - displayWeight);

    // 1. HYSTERESIS LOGIC (The "Sticking" part)
    // If the change is very small (less than 0.07g), we ignore it.
    // This stops the 100.7 <-> 100.8 flickering.
    if (diff < 0.07) { 
        // Do nothing, keep displayWeight exactly as it is
    } 
    // 2. STABILIZING ZONE (Small changes)
    else if (diff < 0.5) {
        // Slow move: we only move 10% towards the new weight
        displayWeight = (currentRaw * 0.1) + (displayWeight * 0.9);
    } 
    // 3. FAST RESPONSE (Pouring or removing cup)
    else {
        // Instant update for big changes
        displayWeight = currentRaw;
    }

    // Snap to Zero
    if (abs(displayWeight) < 0.12) displayWeight = 0.0;

    // Timer Auto-start
    if (autoStartEnabled && !timerRunning && !isCalibrating && displayWeight > 3.0) {
      timerRunning = true; 
      timerStartMillis = now;
    }
  }

  // --- SCREENS UPDATE ---
  if (now - lastDisplayUpdate > DISPLAY_INTERVAL) {
    lastDisplayUpdate = now;

    // TIMER SCREEN
    u8g2_timer.clearBuffer();
    u8g2_timer.setFont(u8g2_font_6x12_tr);
    u8g2_timer.drawStr(0, 10, isCalibrating ? "CALIBRATION" : "WENDSTUDIO");
    
    unsigned long elapsed = (timerRunning) ? (now - timerStartMillis)/1000 : 0;
    char tStr[10]; sprintf(tStr, "%02u:%02u", (unsigned int)(elapsed/60), (unsigned int)(elapsed%60));
    
    u8g2_timer.setFont(u8g2_font_logisoso32_tn); 
    int tWidth = u8g2_timer.getStrWidth(tStr);
    u8g2_timer.drawStr((128 - tWidth) / 2, 55, tStr); 
    u8g2_timer.sendBuffer();

    // WEIGHT SCREEN
    u8g2_weight.clearBuffer();
    u8g2_weight.setFont(u8g2_font_6x12_tr);
    if (isCalibrating) {
      u8g2_weight.drawStr(0, 10, "SET 100.8G + TARE");
    } else {
      u8g2_weight.drawStr(0, 10, autoStartEnabled ? "AUTO-MODE" : "MANUAL");
      
      int batPct = (int)constrain(((smoothedBatteryV - 6.4) / 2.0) * 100, 0, 100); 
      char bStr[10]; sprintf(bStr, "%d%%", batPct);
      u8g2_weight.drawStr(105, 10, bStr);
    }
    
    u8g2_weight.setFont(u8g2_font_logisoso32_tn); 
    char wStr[15]; dtostrf(displayWeight, 1, 1, wStr);
    int wWidth = u8g2_weight.getStrWidth(wStr);
    u8g2_weight.drawStr((128 - wWidth) / 2, 55, wStr); 
    u8g2_weight.sendBuffer();
  }
}
