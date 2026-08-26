#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include "hardware/gpio.h"
#include "hardware/xosc.h"


// SCREEN SSD1309 2.42" SPI
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R2, 17, 16, 20);


NAU7802 myScale;



// Hardware base zero offset (set at startup/calibration for overload protection)
int32_t hardwareZeroOffset = 0;


// Auto-Sleep options: 0 = 10m, 1 = 20m, 2 = 1h, 3 = Off
uint8_t autoSleepIndex = 1; // Default: 20m


// Returns timeout in milliseconds, or 0 if disabled
unsigned long getSleepTimeoutMs() {
  switch (autoSleepIndex) {
    case 0: return 600000UL;   // 10 minutes
    case 1: return 1200000UL;  // 20 minutes
    case 2: return 3600000UL;  // 1 hour
    case 3: return 0UL;        // Disabled
    default: return 1200000UL;
  }
}


// Physical button pins (0=Pin7, 1=Pin8, 2=Pin6)
const int buttonPins[3] = { 7, 8, 6 };
const int BATTERY_PIN = 28;

enum Action { ACTION_TIME = 0,
              ACTION_AUTO = 1,
              ACTION_TARE = 2 };
int userMapping[3] = { 0, 1, 2 };


enum ScaleState {
  STATE_SCALE,
  STATE_MENU_HOME,
  STATE_CONFIRM_DEFAULTS,
  STATE_MAP_TARE,
  STATE_MAP_AUTO,
  STATE_MAP_TIME,
  STATE_CONFIRM_CALIBRATION,
  STATE_CAL_STEP1_EMPTY,
  STATE_CAL_STEP2_PLACE_WT,
  STATE_CAL_STEP3_MEASURE,
  STATE_CAL_DONE
};
ScaleState currentState = STATE_SCALE;


int menuIndex = 0;  // List of 8: 0..7
bool isLbs = false;
bool isNiMH = false;
bool tareResetsTimer = true;  // TRUE = RESET (Time+Weight), FALSE = TARE (Weight only)


unsigned long lastActivityTime = 0;


void enterDeepSleep();
void setOledBrightness(uint8_t mode);
inline void resetInactivityTimer() { lastActivityTime = millis(); }


// Time and weight
const float factoryCalibrationFactor = 687.8700f;
float calibrationFactor = factoryCalibrationFactor;
float displayWeight = 0.00;
unsigned long timerStartMillis = 0;
bool timerRunning = false;
bool autoStartEnabled = false;


// Dynamic absolute weight tracking for overload protection
float absoluteWeightGrams = 0.0f;


enum BatteryDisplayMode {
  BAT_SHOW_VOLTS = 0,
  BAT_SHOW_PCT   = 1,
  BAT_SHOW_BOTH  = 2
};


uint8_t batteryDisplayMode = BAT_SHOW_BOTH;


float smoothedBatteryV = 3.00;
const float VOLTAGE_CORRECTION = 1.115;
unsigned long lastBatteryCheck = 0;
bool batteryInitialized = false;

// Function to take a clean, multi-sampled ADC reading
float readStabilizedBatteryVoltage() {
  // 1. Dummy read to clear residual charge on RP2040 ADC sample-and-hold capacitor
  analogRead(BATTERY_PIN);
  delayMicroseconds(50);

  // 2. Accumulate 32 samples
  uint32_t adcSum = 0;
  for (int i = 0; i < 32; i++) {
    adcSum += analogRead(BATTERY_PIN);
    delayMicroseconds(100); // Allow ADC capacitor to settle
  }

  float avgRaw = (float)adcSum / 32.0f;

  // 3. Convert to Volts (12-bit ADC = 4095, Divider = 147k/47k)
  return (avgRaw / 4095.0f) * 3.3f * (147.0f / 47.0f) * VOLTAGE_CORRECTION;
}


unsigned long lastDisplayUpdate = 0;
const int DISPLAY_INTERVAL = 40;


// Debounce for toggle switches
unsigned long lastDebounceTime[3] = { 0, 0, 0 };
bool buttonState[3] = { HIGH, HIGH, HIGH };
bool lastStableButtonState[3] = { HIGH, HIGH, HIGH };
const unsigned long DEBOUNCE_DELAY = 50;
unsigned long menuEnteredTime = 0;


const uint8_t contrastValues[6] = { 1, 5, 10, 20, 30, 40 };
uint8_t contrastIndex = 3;


void drawUI();
void loadSettings();
void saveSettings();


void setup() {
  set_sys_clock_khz(48000, true);


  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.begin();
  Wire.setClock(100000);   // 400kHz Fast I2C but test stability at 10
  Wire.setTimeout(20000);  // 20ms hardware timeout for I2C operations
  SPI.begin();


  u8g2.begin();


  loadSettings();
  setOledBrightness(contrastIndex);
  u8g2.setContrast(contrastValues[contrastIndex]);


  for (int i = 0; i < 3; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  analogReadResolution(12);


  if (myScale.begin(Wire)) {
    myScale.setLDO(NAU7802_LDO_3V3);
    myScale.setGain(NAU7802_GAIN_128);
    myScale.setSampleRate(NAU7802_SPS_20);


    myScale.calibrateAFE();
    myScale.calculateZeroOffset(4);
    hardwareZeroOffset = myScale.getZeroOffset(); // Capture hardware baseline zero
  }
}


void loop() {
  unsigned long now = millis();


  // Buttons reading and debounce
  bool click7 = false;
  bool click8 = false;
  bool click6 = false;


  for (int i = 0; i < 3; i++) {
    bool rawReading = digitalRead(buttonPins[i]);


    if (rawReading != buttonState[i]) {
      lastDebounceTime[i] = now;
      buttonState[i] = rawReading;
    }


    if ((now - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
      if (rawReading != lastStableButtonState[i]) {
        if (rawReading == HIGH) {
          if (buttonPins[i] == 7) click7 = true;
          if (buttonPins[i] == 8) click8 = true;
          if (buttonPins[i] == 6) click6 = true;
        }
        lastStableButtonState[i] = rawReading;
      }
    }
  }


  if (click7 || click8 || click6) {
    resetInactivityTimer();
  }


  unsigned long currentTimeout = getSleepTimeoutMs();
  if (currentTimeout > 0 && currentState == STATE_SCALE && (now - lastActivityTime >= currentTimeout)) {
    enterDeepSleep();
  }


  // Menu access + debounce
  static unsigned long btn6DownTime = 0;
  static bool waitingForRelease6 = false;


  if (digitalRead(6) == LOW) {
    if (btn6DownTime == 0 && !waitingForRelease6) {
      btn6DownTime = now;
    }


    if (currentState == STATE_SCALE && btn6DownTime > 0 && (now - btn6DownTime >= 500)) {
      currentState = STATE_MENU_HOME;
      menuIndex = 0;
      waitingForRelease6 = true;
      btn6DownTime = 0;
      menuEnteredTime = now;
      u8g2.clearBuffer();
    }
  } else {
    if (waitingForRelease6) {
      waitingForRelease6 = false;
    }
    btn6DownTime = 0;
  }


  if (waitingForRelease6) {
    click6 = false;
  }


  if (currentState != STATE_SCALE && (now - menuEnteredTime < 1000)) {
    click6 = false;
    click7 = false;
    click8 = false;
  }


  // MENU LOGIC
  if (currentState != STATE_SCALE) {
    if (currentState == STATE_MENU_HOME) {
      if (click7) {
        menuIndex = (menuIndex + 1) % 8;
      } else if (click6) {
        if (menuIndex == 0) isLbs = !isLbs;
        else if (menuIndex == 1) isNiMH = !isNiMH;
        else if (menuIndex == 2) tareResetsTimer = !tareResetsTimer;
        else if (menuIndex == 3) {
          contrastIndex = (contrastIndex + 1) % 6;
          setOledBrightness(contrastIndex);
        } else if (menuIndex == 4) {
          currentState = STATE_CONFIRM_DEFAULTS;
          u8g2.clearBuffer();
        } else if (menuIndex == 5) {
          currentState = STATE_CONFIRM_CALIBRATION;
          u8g2.clearBuffer();
        } else if (menuIndex == 6) {
          batteryDisplayMode = (batteryDisplayMode + 1) % 3;
        } else if (menuIndex == 7) {
          autoSleepIndex = (autoSleepIndex + 1) % 4;
        }
      } else if (click8) {
        saveSettings();
        currentState = STATE_SCALE;
        u8g2.clearBuffer();
      }
    } else if (currentState == STATE_CONFIRM_CALIBRATION) {
      if (click7) {
        // RESTORE FACTORY CALIBRATION FACTOR
        calibrationFactor = factoryCalibrationFactor;
        saveSettings();
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.drawStr(30, 32, "RESTORED!");
        u8g2.sendBuffer();
        delay(1000);
        currentState = STATE_MENU_HOME;
        menuEnteredTime = now;
        u8g2.clearBuffer();
      } else if (click6) {
        currentState = STATE_CAL_STEP1_EMPTY;
        u8g2.clearBuffer();
      } else if (click8) {
        currentState = STATE_MENU_HOME;
        u8g2.clearBuffer();
      }
    } else if (currentState == STATE_CONFIRM_DEFAULTS) {
      if (click7) {
        userMapping[ACTION_TARE] = -1;
        userMapping[ACTION_AUTO] = -1;
        userMapping[ACTION_TIME] = -1;
        currentState = STATE_MAP_TARE;
        u8g2.clearBuffer();
      } else if (click6) {
        userMapping[ACTION_TIME] = 0;
        userMapping[ACTION_AUTO] = 1;
        userMapping[ACTION_TARE] = 2;
        currentState = STATE_MENU_HOME;
        u8g2.clearBuffer();
      } else if (click8) {
        currentState = STATE_MENU_HOME;
        u8g2.clearBuffer();
      }
    } else if (currentState == STATE_MAP_TARE) {
      if (click7) { userMapping[ACTION_TARE] = 0; currentState = STATE_MAP_AUTO; u8g2.clearBuffer(); }
      else if (click8) { userMapping[ACTION_TARE] = 1; currentState = STATE_MAP_AUTO; u8g2.clearBuffer(); }
      else if (click6) { userMapping[ACTION_TARE] = 2; currentState = STATE_MAP_AUTO; u8g2.clearBuffer(); }
    } else if (currentState == STATE_MAP_AUTO) {
      if (click7 && userMapping[ACTION_TARE] != 0) { userMapping[ACTION_AUTO] = 0; currentState = STATE_MAP_TIME; u8g2.clearBuffer(); }
      else if (click8 && userMapping[ACTION_TARE] != 1) { userMapping[ACTION_AUTO] = 1; currentState = STATE_MAP_TIME; u8g2.clearBuffer(); }
      else if (click6 && userMapping[ACTION_TARE] != 2) { userMapping[ACTION_AUTO] = 2; currentState = STATE_MAP_TIME; u8g2.clearBuffer(); }
    } else if (currentState == STATE_MAP_TIME) {
      for (int i = 0; i < 3; i++) {
        if (userMapping[ACTION_TARE] != i && userMapping[ACTION_AUTO] != i) {
          userMapping[ACTION_TIME] = i;
        }
      }
      currentState = STATE_MENU_HOME;
      u8g2.clearBuffer();
    } else if (currentState == STATE_CAL_STEP1_EMPTY) {
      if (click6) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.drawStr(25, 32, "SETTLING 0G...");
        u8g2.sendBuffer();


        delay(800);
        while (myScale.available()) { myScale.getReading(); }


        long rawZero = 0;
        int samplesZero = 0;
        unsigned long calTimeout = millis();


        while (samplesZero < 30 && (millis() - calTimeout < 2500)) {
          if (myScale.available()) {
            rawZero += myScale.getReading();
            samplesZero++;
          }
          delay(10);
        }


        if (samplesZero > 0) {
          int32_t zOffset = rawZero / samplesZero;
          myScale.setZeroOffset(zOffset);
          hardwareZeroOffset = zOffset; // Update hardware reference base on zero calibration
        }


        currentState = STATE_CAL_STEP2_PLACE_WT;
        menuEnteredTime = now;
        u8g2.clearBuffer();
      } else if (click8) {
        currentState = STATE_MENU_HOME;
        menuEnteredTime = now;
        u8g2.clearBuffer();
      }
    } else if (currentState == STATE_CAL_STEP2_PLACE_WT) {
      if (click6) {
        currentState = STATE_CAL_STEP3_MEASURE;
        menuEnteredTime = now;
        u8g2.clearBuffer();
      } else if (click8) {
        currentState = STATE_MENU_HOME;
        menuEnteredTime = now;
        u8g2.clearBuffer();
      }
    } else if (currentState == STATE_CAL_STEP3_MEASURE) {
      static unsigned long calStepStart = 0;
      static bool samplingStarted = false;


      if (calStepStart == 0) {
        calStepStart = now;
        samplingStarted = false;
      }


      if (now - calStepStart >= 1200 && !samplingStarted) {
        samplingStarted = true;
        while (myScale.available()) { myScale.getReading(); }


        long rawWithWeight = 0;
        int samplesWeight = 0;
        unsigned long calTimeout = millis();


        while (samplesWeight < 30 && (millis() - calTimeout < 2500)) {
          if (myScale.available()) {
            rawWithWeight += myScale.getReading();
            samplesWeight++;
          }
        }


        if (samplesWeight > 0) {
          rawWithWeight /= samplesWeight;
        }


        long rawZero = myScale.getZeroOffset();
        float newFactor = (float)abs(rawWithWeight - rawZero) / 100.0f;


        if (newFactor > 10.0f) {
          calibrationFactor = newFactor;
          saveSettings();
          currentState = STATE_CAL_DONE;
        } else {
          currentState = STATE_MENU_HOME;
        }


        calStepStart = 0;
        menuEnteredTime = now;
        u8g2.clearBuffer();
      }
    } else if (currentState == STATE_CAL_DONE) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(30, 32, "CALIBRATED!");
      u8g2.sendBuffer();


      delay(1200);
      currentState = STATE_MENU_HOME;
      menuEnteredTime = now;
      u8g2.clearBuffer();
    }


    if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
      lastDisplayUpdate = now;
      drawUI();
    }
    return;
  }


  // SCALE MODE
  bool tareActionActive = (digitalRead(buttonPins[userMapping[ACTION_TARE]]) == LOW);
  bool timeActionClick = (userMapping[ACTION_TIME] == 0 ? click7 : (userMapping[ACTION_TIME] == 1 ? click8 : click6));
  bool autoActionClick = (userMapping[ACTION_AUTO] == 0 ? click7 : (userMapping[ACTION_AUTO] == 1 ? click8 : click6));


  static unsigned long tareDownTime = 0;
  if (tareActionActive) {
    if (tareDownTime == 0) tareDownTime = now;
  } else {
    if (tareDownTime > 0 && (now - tareDownTime < 800)) {
      if (tareResetsTimer) {
        timerRunning = false;
        autoStartEnabled = false;
        timerStartMillis = 0;
      }


      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(30, 35, tareResetsTimer ? "RESETTING..." : "TARING...");
      u8g2.sendBuffer();


      delay(150);
      myScale.calculateZeroOffset(8);
      displayWeight = 0.00;
    }
    tareDownTime = 0;
  }


  if (timeActionClick) {
    if (timerRunning) {
      timerRunning = false;
      autoStartEnabled = false;
    } else {
      timerStartMillis = now;
      timerRunning = true;
      autoStartEnabled = false;
    }
  }


  if (autoActionClick) {
    myScale.calculateZeroOffset(15);
    displayWeight = 0;
    timerStartMillis = now;
    timerRunning = false;
    autoStartEnabled = true;
  }


// BATTERY MEASUREMENT (Every 15 seconds)
  unsigned long batteryInterval = 15000; 
  if (now - lastBatteryCheck >= batteryInterval) {
    lastBatteryCheck = now;
    
    float newSampleV = readStabilizedBatteryVoltage();

    if (!batteryInitialized) {
      smoothedBatteryV = newSampleV;
      batteryInitialized = true;
    } else {
      // Exponential Moving Average (EMA filter)
      // 85% previous value + 15% new sample prevents sudden screen jumps
      smoothedBatteryV = (smoothedBatteryV * 0.7f) + (newSampleV * 0.3f);
    }
  }


  // WEIGHT READING from NAU7802 AND SMOOTHING
  if (myScale.available()) {
    long rawValue = 0;
    while (myScale.available()) {
      rawValue = myScale.getReading();
    }


    // Absolute load cell mass check relative to true hardware zero
    absoluteWeightGrams = (float)(rawValue - hardwareZeroOffset) / calibrationFactor;


    // Relative weight for display (relative to tared zero)
    float currentRawGrams = (float)(rawValue - myScale.getZeroOffset()) / calibrationFactor;


    float diff = abs(currentRawGrams - displayWeight);


    if (timerRunning || diff > 0.50f) {
      resetInactivityTimer();
    }


    if (abs(displayWeight) < 0.40f && diff < 0.25f) {
      displayWeight = (currentRawGrams * 0.04f) + (displayWeight * 0.96f);
    } else if (diff < 0.07f) {
      // Deadband stability locking
    } else if (diff < 0.60f) {
      displayWeight = (currentRawGrams * 0.12f) + (displayWeight * 0.88f);
    } else {
      displayWeight = currentRawGrams;
    }


    if (abs(displayWeight) < 0.08f) {
      displayWeight = 0.00f;
    }


    if (autoStartEnabled && !timerRunning && displayWeight > 5.0f) {
      timerRunning = true;
      timerStartMillis = now;
    }
  }


  if (now - lastDisplayUpdate > DISPLAY_INTERVAL) {
    lastDisplayUpdate = now;
    drawUI();
  }
}


void drawUI() {
  u8g2.clearBuffer();


  auto drawButtonFooter = [](const char* f7, const char* f8, const char* f6) {
    u8g2.setFont(u8g2_font_micropixel_tr);
    const char* sep = " - ";
    int sepWidth = u8g2.getStrWidth(sep);
    int x = 0;


    u8g2.drawStr(x, 62, f7);
    x += u8g2.getStrWidth(f7);
    u8g2.drawStr(x, 62, sep);
    x += sepWidth;
    u8g2.drawStr(x, 62, f8);
    x += u8g2.getStrWidth(f8);
    u8g2.drawStr(x, 62, sep);
    x += sepWidth;
    u8g2.drawStr(x, 62, f6);
  };


  auto getMapLabel = [](int actionIndex) {
    if (actionIndex == 0) return "Time";
    if (actionIndex == 1) return "Auto";
    if (actionIndex == 2) return "Tare";
    return "--";
  };


  if (currentState == STATE_MENU_HOME) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(5, 11, "REBEL COFFEE SCALE");


    int pageStart = menuIndex - 2;
    if (pageStart < 0) pageStart = 0;
    if (pageStart > 8 - 3) pageStart = 8 - 3;


    for (int i = 0; i < 3; i++) {
      int itemIdx = pageStart + i;
      if (itemIdx >= 8) break;


      int rowY = 24 + (i * 12);
      u8g2.drawStr(5, rowY, (itemIdx == menuIndex) ? ">" : " ");


      char menuLine[30];
      if (itemIdx == 0) sprintf(menuLine, "UNITS: %s", isLbs ? "LBS" : "GRAMS");
      else if (itemIdx == 1) sprintf(menuLine, "BATTERY: %s", isNiMH ? "NiMH" : "ALKALINE");
      else if (itemIdx == 2) sprintf(menuLine, "TARE MODE: %s", tareResetsTimer ? "RESET" : "TARE");
      else if (itemIdx == 3) sprintf(menuLine, "CONTRAST: %d", contrastValues[contrastIndex]);
      else if (itemIdx == 4) strcpy(menuLine, "BUTTON ASSIGN");
      else if (itemIdx == 5) strcpy(menuLine, "CALIBRATION");
      else if (itemIdx == 6) {
        const char* modeStr = (batteryDisplayMode == BAT_SHOW_VOLTS) ? "VOLTS" :
                              (batteryDisplayMode == BAT_SHOW_PCT)   ? "PCT" : "BOTH";
        sprintf(menuLine, "BAT DISP: %s", modeStr);
      } else if (itemIdx == 7) {
        const char* sleepStr = (autoSleepIndex == 0) ? "10M" :
                               (autoSleepIndex == 1) ? "20M" :
                               (autoSleepIndex == 2) ? "1H"  : "OFF";
        sprintf(menuLine, "AUTO SLEEP: %s", sleepStr);
      }


      u8g2.drawStr(15, rowY, menuLine);
    }


    drawButtonFooter("SCROLL", "BACK", "ENTER");
    u8g2.sendBuffer();
    return;
  } else if (currentState == STATE_CONFIRM_CALIBRATION) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(15, 22, "Sure you want to");
    u8g2.drawStr(15, 36, "calibrate?");
    drawButtonFooter("RESET", "BACK", "YES");
    u8g2.sendBuffer();
    return;
  } else if (currentState == STATE_CONFIRM_DEFAULTS) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(5, 18, "Map Buttons");
    u8g2.drawStr(5, 32, "Configuration");
    drawButtonFooter("SET", "BACK", "DEFAULT");
    u8g2.sendBuffer();
    return;
  } else if (currentState >= STATE_MAP_TARE && currentState <= STATE_MAP_TIME) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(5, 16, "PRESS BUTTON FOR:");
    if (currentState == STATE_MAP_TARE) u8g2.drawStr(5, 30, "TARE");
    else if (currentState == STATE_MAP_AUTO) u8g2.drawStr(5, 30, "AUTO");


    char l7[12], l8[12], l6[12];
    sprintf(l7, "P7:%s", getMapLabel(userMapping[0] == 0 ? 0 : (userMapping[1] == 0 ? 1 : (userMapping[2] == 0 ? 2 : -1))));
    sprintf(l8, "P8:%s", getMapLabel(userMapping[0] == 1 ? 0 : (userMapping[1] == 1 ? 1 : (userMapping[2] == 1 ? 2 : -1))));
    sprintf(l6, "P6:%s", getMapLabel(userMapping[0] == 2 ? 0 : (userMapping[1] == 2 ? 1 : (userMapping[2] == 2 ? 2 : -1))));
    drawButtonFooter(l7, l8, l6);
    u8g2.sendBuffer();
    return;
  } else if (currentState == STATE_CAL_STEP1_EMPTY) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(10, 20, "1. CLEAR PLATFORM");
    u8g2.drawStr(10, 36, "   THEN PRESS NEXT");
    drawButtonFooter("---", "CANCEL", "NEXT");
    u8g2.sendBuffer();
    return;
  } else if (currentState == STATE_CAL_STEP2_PLACE_WT) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(10, 20, "2. PLACE 100G");
    u8g2.drawStr(10, 36, "   THEN PRESS NEXT");
    drawButtonFooter("---", "CANCEL", "NEXT");
    u8g2.sendBuffer();
    return;
  } else if (currentState == STATE_CAL_STEP3_MEASURE) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(25, 32, "MEASURING...");
    u8g2.sendBuffer();
    return;
  } else if (currentState == STATE_CAL_DONE) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(30, 32, "CALIBRATED!");
    u8g2.sendBuffer();
    return;
  }






  // MAIN SCALE DISPLAY
  u8g2.setFont(u8g2_font_squeezed_b6_tr);
  
  /* draw calibration factor
  char bufferForCalibrationDisplay[12]; // Create a character buffer big enough to hold the number
 // Convert float to string with 4 decimal places
  snprintf(bufferForCalibrationDisplay, sizeof(bufferForCalibrationDisplay), "%.4f", calibrationFactor);
  u8g2.drawStr(0, 6, bufferForCalibrationDisplay); 
  */


  if (timerRunning) {
    u8g2.drawTriangle(0, 1, 0, 13, 9, 7);
  } else if (autoStartEnabled) {
    u8g2.drawStr(0, 6, "Auto start");
  }


  // BATTERY DISPLAY
  char batDisplayStr[25] = "";
  if (batteryInitialized) {
    int batPct = 0;
    if (isNiMH) {
      if (smoothedBatteryV >= 2.70f) batPct = 100;
      else if (smoothedBatteryV >= 2.55f) batPct = 90;
      else if (smoothedBatteryV >= 2.45f) batPct = 75;
      else if (smoothedBatteryV >= 2.35f) batPct = 50;
      else if (smoothedBatteryV >= 2.25f) batPct = 30;
      else batPct = 0;
    } else {
      if (smoothedBatteryV >= 3.05f) batPct = 100;
      else if (smoothedBatteryV >= 2.90f) batPct = 90;
      else if (smoothedBatteryV >= 2.75f) batPct = 75;
      else if (smoothedBatteryV >= 2.60f) batPct = 50;
      else if (smoothedBatteryV >= 2.45f) batPct = 30;
      else if (smoothedBatteryV >= 2.30f) batPct = 15;
      else if (smoothedBatteryV >= 2.15f) batPct = 5;
      else batPct = 0;
    }


    char vBuf[8];
    dtostrf(smoothedBatteryV, 1, 2, vBuf);


    if (smoothedBatteryV <= 2.25f) {
      sprintf(batDisplayStr, "LOW (%sv)", vBuf);
    } else {
      if (batteryDisplayMode == BAT_SHOW_VOLTS) {
        sprintf(batDisplayStr, "%sv", vBuf);
      } else if (batteryDisplayMode == BAT_SHOW_PCT) {
        sprintf(batDisplayStr, "%d%%", batPct);
      } else {
        sprintf(batDisplayStr, "(%sv) %d%%", vBuf, batPct);
      }
    }
  }


  int batWidth = u8g2.getStrWidth(batDisplayStr);
  u8g2.drawStr(128 - batWidth, 6, batDisplayStr);


  int mainY = 37;
  int labelY = 58;


  // LOAD CELL PROTECTION EVALUATION
  unsigned long elapsed = (timerRunning) ? (millis() - timerStartMillis) / 1000 : 0;
  unsigned int hours   = elapsed / 3600;
  unsigned int minutes = (elapsed % 3600) / 60;
  unsigned int seconds = elapsed % 60;


  bool isWarning  = (absoluteWeightGrams > 3000.0f);
  bool isOverload = (absoluteWeightGrams > 4500.0f);


  if (isOverload) {
    u8g2.setFont(u8g2_font_helvB12_tr);
    const char* heavyStr = "TOO HEAVY";
    int hWidth = u8g2.getStrWidth(heavyStr);
    u8g2.drawStr((128 - hWidth) / 2, mainY - 2, heavyStr);
  } else {
    // Render Timer / Warning
    if (isWarning) {
      u8g2.setFont(u8g2_font_7x14_tr);
      u8g2.drawStr(0, mainY - 2, "WARNING!");
    } else {
      char tStr[16];
      if (hours > 0) {
        u8g2.setFont(u8g2_font_fub11_tn);
        sprintf(tStr, "%u:%02u:%02u", hours, minutes, seconds);
      } else {
        u8g2.setFont(u8g2_font_fub17_tn);
        sprintf(tStr, "%02u:%02u", minutes, seconds);
      }
      u8g2.drawStr(0, mainY, tStr);
    }


    // Render Weight
    float calcWeight = isLbs ? (displayWeight * 0.00220462f) : displayWeight;


    if (hours > 0 || isWarning) {
      u8g2.setFont(u8g2_font_fub11_tn);
    } else {
      u8g2.setFont(u8g2_font_fub17_tn);
    }


    char wStr[15];
    if (isLbs) {
      dtostrf(calcWeight, 1, 3, wStr);
    } else if (autoStartEnabled) {
      dtostrf(calcWeight, 1, 0, wStr);
    } else {
      int decimals = (abs(displayWeight) >= 1000.0f) ? 0 : 1;
      dtostrf(calcWeight, 1, decimals, wStr);
    }


    int wWidth = u8g2.getStrWidth(wStr);
    u8g2.drawStr(128 - wWidth, mainY, wStr);
  }


  // FOOTER BUTTON LABELS
  u8g2.setFont(u8g2_font_squeezed_b6_tr);
  const char* actionNames[3] = { "Time", "Auto", "Tare" };


  int act7 = (userMapping[0] == 0) ? 0 : ((userMapping[1] == 0) ? 1 : 2);
  int act8 = (userMapping[0] == 1) ? 0 : ((userMapping[1] == 1) ? 1 : 2);
  int act6 = (userMapping[0] == 2) ? 0 : ((userMapping[1] == 2) ? 1 : 2);


  const char* sep = " - ";
  int sepWidth = u8g2.getStrWidth(sep);
  int currentX = 0;


  u8g2.drawStr(currentX, labelY, actionNames[act7]);
  currentX += u8g2.getStrWidth(actionNames[act7]);
  u8g2.drawStr(currentX, labelY, sep);
  currentX += sepWidth;
  u8g2.drawStr(currentX, labelY, actionNames[act8]);
  currentX += u8g2.getStrWidth(actionNames[act8]);
  u8g2.drawStr(currentX, labelY, sep);
  currentX += sepWidth;
  u8g2.drawStr(currentX, labelY, actionNames[act6]);


  u8g2.sendBuffer();
}


void enterDeepSleep() {
  u8g2.setPowerSave(1);
  myScale.powerDown();


  for (int i = 0; i < 3; i++) {
    gpio_set_dormant_irq_enabled(buttonPins[i], IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_LEVEL_LOW_BITS, true);
  }


  xosc_dormant();


  for (int i = 0; i < 3; i++) {
    gpio_set_dormant_irq_enabled(buttonPins[i], IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_LEVEL_LOW_BITS, false);
    gpio_acknowledge_irq(buttonPins[i], IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_LEVEL_LOW_BITS);
  }


  myScale.powerUp();
  myScale.calculateZeroOffset(4);
  u8g2.setPowerSave(0);


  resetInactivityTimer();
}


void setOledBrightness(uint8_t mode) {
  switch (mode) {
    case 0: // Level 1 (Ultra Dim)
      u8g2.sendF("ca", 0xD9, 0x11);
      u8g2.sendF("ca", 0xDB, 0x00);
      u8g2.setContrast(0);
      break;
    case 1: // Level 5 (Dim)
      u8g2.sendF("ca", 0xD9, 0x22);
      u8g2.sendF("ca", 0xDB, 0x10);
      u8g2.setContrast(5);
      break;
    case 2: // Level 10 (Low)
      u8g2.sendF("ca", 0xD9, 0x22);
      u8g2.sendF("ca", 0xDB, 0x20);
      u8g2.setContrast(15);
      break;
    case 3: // Level 20 (Medium)
      u8g2.sendF("ca", 0xD9, 0x44);
      u8g2.sendF("ca", 0xDB, 0x30);
      u8g2.setContrast(40);
      break;
    case 4: // Level 30 (High)
      u8g2.sendF("ca", 0xD9, 0x82);
      u8g2.sendF("ca", 0xDB, 0x34);
      u8g2.setContrast(120);
      break;
    case 5: // Level 40 (Bright)
      u8g2.sendF("ca", 0xD9, 0xF1);
      u8g2.sendF("ca", 0xDB, 0x34);
      u8g2.setContrast(255);
      break;
  }
}


void loadSettings() {
  EEPROM.begin(256);
  float storedFactor = 0;
  EEPROM.get(0, storedFactor);
  if (!isnan(storedFactor) && abs(storedFactor) > 10.0f) {
    calibrationFactor = storedFactor;
  }


  int validMarker = EEPROM.read(10);
  if (validMarker == 42) {
    userMapping[0] = EEPROM.read(11);
    userMapping[1] = EEPROM.read(12);
    userMapping[2] = EEPROM.read(13);
    isLbs = EEPROM.read(14);
    isNiMH = EEPROM.read(15);
    tareResetsTimer = EEPROM.read(16);


    uint8_t storedBatMode = EEPROM.read(18);
    batteryDisplayMode = (storedBatMode < 3) ? storedBatMode : BAT_SHOW_BOTH;


    uint8_t storedSleepIdx = EEPROM.read(19);
    autoSleepIndex = (storedSleepIdx < 4) ? storedSleepIdx : 1;


    uint8_t storedContrast = EEPROM.read(17);
    contrastIndex = (storedContrast < 6) ? storedContrast : 3;
  }
}


void saveSettings() {
  float currentFactor = 0;
  EEPROM.get(0, currentFactor);


  uint8_t currentMarker = EEPROM.read(10);
  uint8_t currentMap0 = EEPROM.read(11);
  uint8_t currentMap1 = EEPROM.read(12);
  uint8_t currentMap2 = EEPROM.read(13);
  bool currentLbs = EEPROM.read(14);
  bool currentNiMH = EEPROM.read(15);
  bool currentTare = EEPROM.read(16);
  uint8_t currentCont = EEPROM.read(17);
  uint8_t currentBatMode = EEPROM.read(18);
  uint8_t currentSleepIdx = EEPROM.read(19);


  bool needsUpdate = false;
  if (abs(currentFactor - calibrationFactor) > 0.001f) needsUpdate = true;
  if (currentMarker != 42) needsUpdate = true;
  if (currentBatMode != batteryDisplayMode) needsUpdate = true;
  if (currentMap0 != userMapping[0]) needsUpdate = true;
  if (currentMap1 != userMapping[1]) needsUpdate = true;
  if (currentMap2 != userMapping[2]) needsUpdate = true;
  if (currentLbs != isLbs) needsUpdate = true;
  if (currentNiMH != isNiMH) needsUpdate = true;
  if (currentTare != tareResetsTimer) needsUpdate = true;
  if (currentCont != contrastIndex) needsUpdate = true;
  if (currentSleepIdx != autoSleepIndex) needsUpdate = true;


  if (needsUpdate) {
    EEPROM.put(0, calibrationFactor);
    EEPROM.write(10, 42);
    EEPROM.write(11, userMapping[0]);
    EEPROM.write(12, userMapping[1]);
    EEPROM.write(13, userMapping[2]);
    EEPROM.write(14, isLbs);
    EEPROM.write(15, isNiMH);
    EEPROM.write(16, tareResetsTimer);
    EEPROM.write(17, contrastIndex);
    EEPROM.write(18, batteryDisplayMode);
    EEPROM.write(19, autoSleepIndex);
    EEPROM.commit();
  }
}
