#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" 

// --- DUAL SPI CONSTRUCTORS ---
// We share CLK (18), MOSI (19), DC (16), and RES (20). Only CS differs.
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2_timer(U8G2_R0, /* cs=*/ 17, /* dc=*/ 16, /* reset=*/ 20);
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2_weight(U8G2_R0, /* cs=*/ 21, /* dc=*/ 16, /* reset=*/ 20);

NAU7802 myScale;

const int TARE_BTN = 2; 
const int TIME_BTN = 3; 
const int AUTO_BTN = 22; // Moved for clearance
const int BATTERY_PIN = 26;

float calibrationFactor = 1091.26; 
float displayWeight = 0.0; 
unsigned long timerStartMillis = 0;
bool timerRunning = false; 
bool autoStartEnabled = false;
float smoothedBatteryV = 7.4; // 2S Battery starting point

unsigned long lastDisplayUpdate = 0;
const int DISPLAY_INTERVAL = 100; 

void setup() {
  delay(500);
  
  // 1. Setup I2C1 for Scale (Isolated from SPI pins)
  Wire1.setSDA(4); 
  Wire1.setSCL(5);
  Wire1.begin();
  Wire1.setClock(400000); 

  // 2. Setup SPI for Screens
  SPI.begin(); 

  u8g2_timer.begin();
  u8g2_weight.begin();

  if (myScale.begin(Wire1)) {
    myScale.setLDO(NAU7802_LDO_3V0); 
    myScale.setGain(NAU7802_GAIN_128);
    myScale.setSampleRate(NAU7802_SPS_320); 
    myScale.calibrateAFE(); 
    myScale.calculateZeroOffset(64); 
  }

  pinMode(TARE_BTN, INPUT_PULLUP);
  pinMode(TIME_BTN, INPUT_PULLUP);
  pinMode(AUTO_BTN, INPUT_PULLUP);
}

void loop() {
  // --- BUTTONS ---
  if (digitalRead(TARE_BTN) == LOW) { 
    autoStartEnabled = false; timerRunning = false;
    myScale.calculateZeroOffset(64); displayWeight = 0; delay(250); 
  }
  if (digitalRead(TIME_BTN) == LOW) {
    timerRunning = !timerRunning; 
    if(timerRunning) timerStartMillis = millis(); 
    delay(250);
  }
  if (digitalRead(AUTO_BTN) == LOW) {
    autoStartEnabled = !autoStartEnabled;
    if (autoStartEnabled) { myScale.calculateZeroOffset(64); displayWeight = 0; }
    delay(250);
  }

  // --- SCALE SENSING ---
  if (myScale.available()) {
    long rawValue = myScale.getAverage(8); 
    float currentRaw = (float)(rawValue - myScale.getZeroOffset()) / calibrationFactor;

    float diff = abs(currentRaw - displayWeight);

    if (diff < 0.20) { 
        // Locked / Static
    } else if (diff > 2.0) {
      displayWeight = (currentRaw * 0.7) + (displayWeight * 0.3); // Responsive
    } else {
      displayWeight = (currentRaw * 0.15) + (displayWeight * 0.85); // Smooth
    }

    if (abs(displayWeight) < 0.1) displayWeight = 0.0;
  }

  // --- DISPLAY REFRESH ---
  if (millis() - lastDisplayUpdate > DISPLAY_INTERVAL) {
    lastDisplayUpdate = millis();

    // Screen 1: Timer
    u8g2_timer.clearBuffer();
    u8g2_timer.setFont(u8g2_font_6x12_tr);
    u8g2_timer.drawStr(0, 10, "WENDSTUDIO");
    unsigned long elapsed = (timerRunning) ? (millis() - timerStartMillis)/1000 : 0;
    char tStr[10]; sprintf(tStr, "%02u:%02u", (unsigned int)(elapsed/60), (unsigned int)(elapsed%60));
    u8g2_timer.setFont(u8g2_font_logisoso42_tn); 
    u8g2_timer.drawStr((128 - u8g2_timer.getStrWidth(tStr))/2, 64, tStr);
    u8g2_timer.sendBuffer();

    // Screen 2: Weight
    u8g2_weight.clearBuffer();
    u8g2_weight.setFont(u8g2_font_6x12_tr);
    u8g2_weight.drawStr(0, 10, autoStartEnabled ? "AUTO-START" : "MANUAL");
    
    // Battery Logic (Adjusted for 2S 100k/47k divider)
    float currentV = (analogRead(BATTERY_PIN) / 4095.0) * 3.3 * (147.0/47.0);
    smoothedBatteryV = (smoothedBatteryV * 0.95) + (currentV * 0.05);
    char bStr[10]; sprintf(bStr, "%d%%", (int)constrain(((smoothedBatteryV - 6.4) / 2.0) * 100, 0, 100));
    u8g2_weight.drawStr(100, 10, bStr);

    u8g2_weight.setFont(u8g2_font_logisoso42_tn); 
    char wStr[15];
    dtostrf(displayWeight, 1, 1, wStr);
    u8g2_weight.drawStr((128 - u8g2_weight.getStrWidth(wStr))/2, 64, wStr);
    u8g2_weight.sendBuffer();
  }
}