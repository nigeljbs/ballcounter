#include <LiquidCrystal.h>

// ============================================================
// LCD SETUP
// ============================================================
LiquidCrystal lcd(12, 11, 5, 4, A4, A5);

// ============================================================
// PIN DEFINITIONS
// ============================================================
const byte BEAM1_DIGITAL_PIN   = 2;   // LM393 OUT 1
const byte BEAM2_DIGITAL_PIN   = 3;   // LM393 OUT 2

const byte BEAM1_ANALOG_PIN    = A0;  // Beam 1 phototransistor node
const byte BEAM2_ANALOG_PIN    = A1;  // Beam 2 phototransistor node

const byte LASER_CTRL_PIN      = 6;
const byte BACKLIGHT_CTRL_PIN  = 7;

const byte RESET_PIN           = 8;   // short press = -1 / -100, hold 2 sec = reset
const byte TOGGLE_PIN          = 9;   // short press = +1 / +100, hold 2 sec = power off

// ============================================================
// DEBUG CONFIG
// ============================================================
const bool DEBUG_MODE = true;

// ============================================================
// COMPARATOR POLARITY
// ============================================================
const bool COMP_HIGH_WHEN_BLOCKED = true;

// ============================================================
// ANALOG CLEAR THRESHOLD
// ============================================================
const int CLEAR_TH = 700;

// ============================================================
// SPEED / TIMING CONFIG
// ============================================================
const float BEAM_SPACING_IN = 1.95;

const unsigned long MIN_TIME_BETWEEN_COUNTS_MS = 100UL;
const unsigned long EVENT_TIMEOUT_US = 25000UL;
const unsigned long SENSOR_SETTLE_MS = 300UL;
const unsigned long STARTUP_IGNORE_MS = 500UL;

const unsigned long RESET_HOLD_MS       = 2000UL;
const unsigned long POWER_HOLD_MS       = 2000UL;
const unsigned long BOTH_X100_HOLD_MS   = 500UL;
const unsigned long BOTH_DEBUG_HOLD_MS  = 2000UL;

const unsigned long POWER_MSG_MS        = 700UL;
const unsigned long REJECT_MSG_MS       = 700UL;
const unsigned long MANUAL_FLASH_MS     = 350UL;
const unsigned long DEBUG_REFRESH_MS    = 200UL;

// ============================================================
// LIVE VARIABLES
// ============================================================
unsigned long ballCount = 0;

float lastRawSpeedMph = 0.0;
float lastSpeedMph = 0.0;

unsigned long lastCountMs = 0;

bool systemEnabled = true;
bool debugMode = false;
bool quickAdjustMode = false;

unsigned long enabledAtMs = 0;

bool offWakeArmed = false;
bool debugExitArmed = false;

// ============================================================
// SEQUENCE STATE MACHINE
// ============================================================
enum SeqState {
  READY_STATE,
  WAIT_FOR_B2,
  WAIT_FOR_B1,
  WAIT_FOR_CLEAR
};

SeqState seqState = READY_STATE;

// ============================================================
// ISR FLAGS AND TIMESTAMPS
// ============================================================
volatile bool beam1EdgeFlag = false;
volatile bool beam2EdgeFlag = false;

volatile unsigned long tBeam1UsISR = 0;
volatile unsigned long tBeam2UsISR = 0;

unsigned long tBeam1Us = 0;
unsigned long tBeam2Us = 0;

// ============================================================
// DISPLAY STATE
// ============================================================
enum LastDisplayState {
  DISP_NONE,
  DISP_FORWARD
};

LastDisplayState lastDisplayState = DISP_NONE;

// ============================================================
// BUTTON BOOKKEEPING
// ============================================================
bool resetWasPressed = false;
unsigned long resetPressStartMs = 0;
bool resetHoldActionDone = false;

bool toggleWasPressed = false;
unsigned long togglePressStartMs = 0;
bool toggleHoldActionDone = false;

bool bothWasPressed = false;
unsigned long bothPressStartMs = 0;
bool bothHoldActionDone = false;
bool bothX100ActionDone = false;

bool suppressSingleButtonActions = false;

unsigned long lastDebugRefreshMs = 0;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================
void updateDisplay();
void updateDebugDisplay();
void updateButtonIndicator();
void showRejectedMessage();
void flashCountAdjust(const char* adjustment);
void applySystemState();
void handleButtons();
void handleBeams();
void processForwardEvent(unsigned long t1, unsigned long t2);

float mphFromDeltaUs(unsigned long dtUs);
float calibrateSpeed(float rawMph);
float finalCorrection(float calibratedMph);

bool bothBeamsClearAnalog();
void clearEdgeFlags();
void resetButtonStates();

void dbg(const char* msg);
void dbgForward(float rawMph, float calibratedMph, float finalMph, unsigned long count, unsigned long dtUs);
void dbgTimeout(const char* stateName);

// ============================================================
// ISR FUNCTIONS
// ============================================================
void beam1ISR() {
  tBeam1UsISR = micros();
  beam1EdgeFlag = true;
}

void beam2ISR() {
  tBeam2UsISR = micros();
  beam2EdgeFlag = true;
}
// ============================================================
// >>> COUNTER-SPECIFIC CALIBRATION SECTION - SWITCH THIS ONLY <<<
// ============================================================
//
// Calibration basis:
// - Uses ONLY the new counter data provided after mounting
// - Ball Counter Speed values are mapped to radar-equivalent speed
// - Smoothed 3-segment piecewise fit
// - Wide blend zones are used to avoid sudden jumps
//
// Expected performance on this dataset:
// Overall avg % diff: ~1.36%
// 30 mph avg % diff: ~1.55%
// 45 mph avg % diff: ~1.62%
// 65 mph avg % diff: ~1.12%
//
// ============================================================

float calibrateSpeed(float rawMph) {
  const float BREAK_1 = 42.00;
  const float BREAK_2 = 61.15;

  // Wide blend zones for smooth transitions
  const float BLEND_1 = 6.0;
  const float BLEND_2 = 5.0;

  // Low-speed line, mainly 30 mph region
  float lowLine = 0.40171 * rawMph + 20.12102;

  // Mid-speed line, mainly 45 mph region
  float midLine = 0.81128 * rawMph + 4.16989;

  // High-speed line, mainly 65 mph region
  float highLine = 0.74085 * rawMph + 9.18773;

  float y;

  // Base region selection
  if (rawMph <= BREAK_1) {
    y = lowLine;
  }
  else if (rawMph <= BREAK_2) {
    y = midLine;
  }
  else {
    y = highLine;
  }

  // Smooth transition between low and mid lines
  if (rawMph >= BREAK_1 - BLEND_1 && rawMph <= BREAK_1 + BLEND_1) {
    float t = (rawMph - (BREAK_1 - BLEND_1)) / (2.0 * BLEND_1);
    t = t * t * (3.0 - 2.0 * t);  // smoothstep
    y = lowLine + (midLine - lowLine) * t;
  }

  // Smooth transition between mid and high lines
  else if (rawMph >= BREAK_2 - BLEND_2 && rawMph <= BREAK_2 + BLEND_2) {
    float t = (rawMph - (BREAK_2 - BLEND_2)) / (2.0 * BLEND_2);
    t = t * t * (3.0 - 2.0 * t);  // smoothstep
    y = midLine + (highLine - midLine) * t;
  }

  return y;
}

// No extra correction needed.
// calibrateSpeed() already maps the counter speed directly to radar-equivalent speed.
float finalCorrection(float calibratedMph) {
  return calibratedMph;
}

// ============================================================
// <<< END COUNTER-SPECIFIC CALIBRATION SECTION >>>
// ============================================================


// HELPERS
// ============================================================
bool bothBeamsClearAnalog() {
  int a0 = analogRead(BEAM1_ANALOG_PIN);
  int a1 = analogRead(BEAM2_ANALOG_PIN);

  return (a0 <= CLEAR_TH) && (a1 <= CLEAR_TH);
}

void clearEdgeFlags() {
  noInterrupts();
  beam1EdgeFlag = false;
  beam2EdgeFlag = false;
  interrupts();
}

void resetButtonStates() {
  resetWasPressed = false;
  toggleWasPressed = false;
  bothWasPressed = false;

  resetHoldActionDone = false;
  toggleHoldActionDone = false;
  bothHoldActionDone = false;
  bothX100ActionDone = false;

  suppressSingleButtonActions = false;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  if (DEBUG_MODE) {
    Serial.begin(115200);
  }

  pinMode(LASER_CTRL_PIN, OUTPUT);
  pinMode(BACKLIGHT_CTRL_PIN, OUTPUT);

  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(TOGGLE_PIN, INPUT_PULLUP);

  pinMode(BEAM1_DIGITAL_PIN, INPUT);
  pinMode(BEAM2_DIGITAL_PIN, INPUT);

  lcd.begin(16, 2);

  systemEnabled = true;
  debugMode = false;
  quickAdjustMode = false;

  digitalWrite(LASER_CTRL_PIN, HIGH);
  digitalWrite(BACKLIGHT_CTRL_PIN, HIGH);
  lcd.display();

  lcd.setCursor(0, 0);
  lcd.print("Power On        ");
  lcd.setCursor(0, 1);
  lcd.print("Starting...     ");
  delay(POWER_MSG_MS);

  enabledAtMs = millis();
  seqState = READY_STATE;
  lastDisplayState = DISP_NONE;

  clearEdgeFlags();

  if (COMP_HIGH_WHEN_BLOCKED) {
    attachInterrupt(digitalPinToInterrupt(BEAM1_DIGITAL_PIN), beam1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(BEAM2_DIGITAL_PIN), beam2ISR, RISING);
  } else {
    attachInterrupt(digitalPinToInterrupt(BEAM1_DIGITAL_PIN), beam1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BEAM2_DIGITAL_PIN), beam2ISR, FALLING);
  }

  updateDisplay();
  dbg("SMOOTH CALIBRATION READY");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  handleButtons();

  if (systemEnabled && debugMode) {
    updateDebugDisplay();
  } else if (systemEnabled && !debugMode) {
    handleBeams();
  }

  if (systemEnabled) {
    updateButtonIndicator();
  }
}

// ============================================================
// DISPLAY UPDATE
// ============================================================
void updateDisplay() {
  if (!systemEnabled) return;

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);

  lcd.print("Count:");
  lcd.setCursor(6, 0);
  lcd.print(ballCount);

  if (quickAdjustMode) {
    lcd.setCursor(12, 0);
    lcd.print("x100");
  }

  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);

  if (lastDisplayState == DISP_NONE) {
    lcd.print("No speed yet");
  } else {
    lcd.print(lastSpeedMph, 2);
    lcd.print(" mph");
  }

  updateButtonIndicator();
}

// ============================================================
// BUTTON INDICATOR
// ============================================================
void updateButtonIndicator() {
  if (!systemEnabled) return;

  bool resetPressed  = (digitalRead(RESET_PIN) == LOW);
  bool togglePressed = (digitalRead(TOGGLE_PIN) == LOW);

  char indicator = ' ';

  if (resetPressed && togglePressed) {
    indicator = 'B';
  } else if (resetPressed) {
    indicator = '-';
  } else if (togglePressed) {
    indicator = '+';
  }

  lcd.setCursor(15, 1);
  lcd.print(indicator);
}

// ============================================================
// DEBUG MODE DISPLAY
// ============================================================
void updateDebugDisplay() {
  if (!systemEnabled) return;

  unsigned long now = millis();
  if (now - lastDebugRefreshMs < DEBUG_REFRESH_MS) return;
  lastDebugRefreshMs = now;

  int a0 = analogRead(BEAM1_ANALOG_PIN);
  int a1 = analogRead(BEAM2_ANALOG_PIN);

  int d2 = digitalRead(BEAM1_DIGITAL_PIN);
  int d3 = digitalRead(BEAM2_DIGITAL_PIN);

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print("A0:");
  lcd.print(a0);
  lcd.setCursor(11, 0);
  lcd.print("D2:");
  lcd.print(d2);

  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("A1:");
  lcd.print(a1);
  lcd.setCursor(11, 1);
  lcd.print("D3:");
  lcd.print(d3);

  updateButtonIndicator();

  if (DEBUG_MODE) {
    Serial.print("DEBUG A0=");
    Serial.print(a0);
    Serial.print(" A1=");
    Serial.print(a1);
    Serial.print(" D2=");
    Serial.print(d2);
    Serial.print(" D3=");
    Serial.println(d3);
  }
}

// ============================================================
// SHOW REVERSE / REJECT MESSAGE
// ============================================================
void showRejectedMessage() {
  if (!systemEnabled || debugMode) return;

  lcd.setCursor(0, 0);
  lcd.print("Reverse Detected");
  lcd.setCursor(0, 1);
  lcd.print("Not Counted     ");
  delay(REJECT_MSG_MS);

  updateDisplay();
}

// ============================================================
// FLASH MANUAL +/- NEXT TO COUNT
// ============================================================
void flashCountAdjust(const char* adjustment) {
  if (!systemEnabled || debugMode) return;

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);

  lcd.print("Count:");
  lcd.setCursor(6, 0);
  lcd.print(ballCount);

  lcd.setCursor(12, 0);
  lcd.print(adjustment);

  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);

  if (lastDisplayState == DISP_NONE) {
    lcd.print("No speed yet");
  } else {
    lcd.print(lastSpeedMph, 2);
    lcd.print(" mph");
  }

  updateButtonIndicator();

  delay(MANUAL_FLASH_MS);
  updateDisplay();
}

// ============================================================
// APPLY SYSTEM STATE
// ============================================================
void applySystemState() {
  if (systemEnabled) {
    digitalWrite(LASER_CTRL_PIN, HIGH);
    digitalWrite(BACKLIGHT_CTRL_PIN, HIGH);
    lcd.display();

    lcd.setCursor(0, 0);
    lcd.print("Power On        ");
    lcd.setCursor(0, 1);
    lcd.print("Ready           ");
    delay(POWER_MSG_MS);

    enabledAtMs = millis();
    seqState = READY_STATE;

    clearEdgeFlags();
    resetButtonStates();

    offWakeArmed = false;
    debugExitArmed = false;

    updateDisplay();
    dbg("ON");
  } else {
    debugMode = false;
    quickAdjustMode = false;

    lcd.display();
    digitalWrite(BACKLIGHT_CTRL_PIN, HIGH);

    lcd.setCursor(0, 0);
    lcd.print("Power Off       ");
    lcd.setCursor(0, 1);
    lcd.print("Press any btn   ");
    delay(POWER_MSG_MS);

    seqState = READY_STATE;
    clearEdgeFlags();

    digitalWrite(LASER_CTRL_PIN, LOW);
    digitalWrite(BACKLIGHT_CTRL_PIN, LOW);
    lcd.noDisplay();

    resetButtonStates();

    offWakeArmed = false;

    dbg("OFF");
  }
}

// ============================================================
// BUTTON HANDLER
// ============================================================
void handleButtons() {
  unsigned long now = millis();

  bool resetPressed  = (digitalRead(RESET_PIN) == LOW);
  bool togglePressed = (digitalRead(TOGGLE_PIN) == LOW);
  bool bothPressed   = resetPressed && togglePressed;
  bool anyPressed    = resetPressed || togglePressed;

  // POWER OFF MODE
  if (!systemEnabled) {
    if (!anyPressed) {
      offWakeArmed = true;
      return;
    }

    if (anyPressed && offWakeArmed) {
      systemEnabled = true;
      applySystemState();
      return;
    }

    return;
  }

  // DEBUG MODE
  if (debugMode) {
    if (!anyPressed) {
      debugExitArmed = true;
      return;
    }

    if (anyPressed && debugExitArmed) {
      debugMode = false;
      debugExitArmed = false;

      seqState = READY_STATE;
      clearEdgeFlags();
      resetButtonStates();

      lcd.setCursor(0, 0);
      lcd.print("DEBUG OFF       ");
      lcd.setCursor(0, 1);
      lcd.print("Resuming        ");
      delay(500);

      updateDisplay();
      dbg("DEBUG OFF");
      return;
    }

    return;
  }

  // BOTH-BUTTON HANDLING
  if (bothPressed && !bothWasPressed) {
    bothWasPressed = true;
    bothPressStartMs = now;
    bothHoldActionDone = false;
    bothX100ActionDone = false;
    suppressSingleButtonActions = true;

    resetWasPressed = false;
    toggleWasPressed = false;
    resetHoldActionDone = false;
    toggleHoldActionDone = false;
  }

  if (bothPressed && bothWasPressed) {
    unsigned long bothDuration = now - bothPressStartMs;

    if (!bothX100ActionDone && bothDuration >= BOTH_X100_HOLD_MS) {
      quickAdjustMode = !quickAdjustMode;
      bothX100ActionDone = true;

      updateDisplay();
      updateButtonIndicator();

      if (quickAdjustMode) {
        dbg("QUICK ADJUST x100 ON");
      } else {
        dbg("QUICK ADJUST OFF");
      }
    }

    if (!bothHoldActionDone && bothDuration >= BOTH_DEBUG_HOLD_MS) {
      debugMode = true;
      debugExitArmed = false;
      quickAdjustMode = false;

      seqState = READY_STATE;
      clearEdgeFlags();

      lcd.setCursor(0, 0);
      lcd.print("DEBUG MODE      ");
      lcd.setCursor(0, 1);
      lcd.print("Paused          ");
      updateButtonIndicator();
      delay(500);

      dbg("DEBUG ON");

      bothHoldActionDone = true;
      resetButtonStates();
      return;
    }

    return;
  }

  if (!bothPressed && bothWasPressed) {
    bothWasPressed = false;
    bothHoldActionDone = false;
    bothX100ActionDone = false;
    suppressSingleButtonActions = false;

    resetWasPressed = false;
    toggleWasPressed = false;
    resetHoldActionDone = false;
    toggleHoldActionDone = false;

    updateDisplay();
    updateButtonIndicator();
    return;
  }

  // RESET BUTTON
  if (resetPressed && !resetWasPressed && !suppressSingleButtonActions) {
    resetWasPressed = true;
    resetPressStartMs = now;
    resetHoldActionDone = false;
  }

  if (resetPressed && resetWasPressed) {
    if (!resetHoldActionDone && (now - resetPressStartMs >= RESET_HOLD_MS)) {
      ballCount = 0;
      lastRawSpeedMph = 0.0;
      lastSpeedMph = 0.0;
      lastDisplayState = DISP_NONE;
      quickAdjustMode = false;

      seqState = READY_STATE;
      clearEdgeFlags();

      lcd.setCursor(0, 0);
      lcd.print("Count Reset     ");
      lcd.setCursor(0, 1);
      lcd.print("Speed Cleared   ");
      delay(600);

      updateDisplay();
      dbg("RESET");

      resetHoldActionDone = true;
    }
  }

  if (!resetPressed && resetWasPressed) {
    unsigned long pressDuration = now - resetPressStartMs;

    if (!resetHoldActionDone && pressDuration < RESET_HOLD_MS && !suppressSingleButtonActions) {
      if (quickAdjustMode) {
        if (ballCount >= 100) {
          ballCount -= 100;
        } else {
          ballCount = 0;
        }

        flashCountAdjust("-100");
        dbg("MANUAL -100");
      } else {
        if (ballCount > 0) {
          ballCount--;
        }

        flashCountAdjust("-1");
        dbg("MANUAL -1");
      }
    }

    resetWasPressed = false;
    resetHoldActionDone = false;
  }

  // POWER BUTTON
  if (togglePressed && !toggleWasPressed && !suppressSingleButtonActions) {
    toggleWasPressed = true;
    togglePressStartMs = now;
    toggleHoldActionDone = false;
  }

  if (togglePressed && toggleWasPressed) {
    if (!toggleHoldActionDone && (now - togglePressStartMs >= POWER_HOLD_MS)) {
      systemEnabled = false;
      applySystemState();
      toggleHoldActionDone = true;
      return;
    }
  }

  if (!togglePressed && toggleWasPressed) {
    unsigned long pressDuration = now - togglePressStartMs;

    if (!toggleHoldActionDone && pressDuration < POWER_HOLD_MS && !suppressSingleButtonActions) {
      if (quickAdjustMode) {
        ballCount += 100;

        flashCountAdjust("+100");
        dbg("MANUAL +100");
      } else {
        ballCount++;

        flashCountAdjust("+1");
        dbg("MANUAL +1");
      }
    }

    toggleWasPressed = false;
    toggleHoldActionDone = false;
  }
}

// ============================================================
// DIGITAL-EDGE 2-BEAM HANDLER
// ============================================================
void handleBeams() {
  if (!systemEnabled) return;
  if (debugMode) return;

  // ------------------------------------------------------------
  // STARTUP IGNORE WINDOW
  // ------------------------------------------------------------
  // When lasers/comparators first power on, the comparator outputs
  // may briefly toggle. Clear any false edge flags during this time
  // so the system does not count a fake ball at startup.
  if (millis() - enabledAtMs < STARTUP_IGNORE_MS) {
    clearEdgeFlags();
    seqState = READY_STATE;
    return;
  }

  bool b1 = false;
  bool b2 = false;
  unsigned long t1 = 0;
  unsigned long t2 = 0;

  noInterrupts();

  if (beam1EdgeFlag) {
    b1 = true;
    t1 = tBeam1UsISR;
    beam1EdgeFlag = false;
  }

  if (beam2EdgeFlag) {
    b2 = true;
    t2 = tBeam2UsISR;
    beam2EdgeFlag = false;
  }

  interrupts();

  unsigned long nowUs = micros();

  switch (seqState) {
    case READY_STATE:
      if (b1 && b2) {
        if (t1 < t2) {
          processForwardEvent(t1, t2);
        } else {
          showRejectedMessage();
          dbg("REV IGNORED");
        }

        seqState = WAIT_FOR_CLEAR;
      }
      else if (b1) {
        tBeam1Us = t1;
        seqState = WAIT_FOR_B2;
      }
      else if (b2) {
        tBeam2Us = t2;
        seqState = WAIT_FOR_B1;
      }
      break;

    case WAIT_FOR_B2:
      if (b2) {
        processForwardEvent(tBeam1Us, t2);
        seqState = WAIT_FOR_CLEAR;
      } else if ((nowUs - tBeam1Us) > EVENT_TIMEOUT_US) {
        dbgTimeout("B1->B2");
        seqState = WAIT_FOR_CLEAR;
      }
      break;

    case WAIT_FOR_B1:
      if (b1) {
        showRejectedMessage();
        dbg("REV IGNORED");
        seqState = WAIT_FOR_CLEAR;
      } else if ((nowUs - tBeam2Us) > EVENT_TIMEOUT_US) {
        dbgTimeout("B2->B1");
        seqState = WAIT_FOR_CLEAR;
      }
      break;

    case WAIT_FOR_CLEAR:
      if (bothBeamsClearAnalog()) {
        clearEdgeFlags();
        seqState = READY_STATE;
        dbg("READY");
      }
      break;
  }
}

// ============================================================
// FORWARD EVENT PROCESSOR
// ============================================================
void processForwardEvent(unsigned long t1, unsigned long t2) {
  unsigned long dtUs = t2 - t1;
  unsigned long nowMs = millis();

  if (nowMs - lastCountMs < MIN_TIME_BETWEEN_COUNTS_MS) {
    dbg("IGNORED <100ms");
    return;
  }

  lastRawSpeedMph = mphFromDeltaUs(dtUs);

  float calibratedSpeedMph = calibrateSpeed(lastRawSpeedMph);
  lastSpeedMph = finalCorrection(calibratedSpeedMph);

  lastDisplayState = DISP_FORWARD;
  ballCount++;
  lastCountMs = nowMs;

  updateDisplay();
  dbgForward(lastRawSpeedMph, calibratedSpeedMph, lastSpeedMph, ballCount, dtUs);
}

// ============================================================
// SPEED CALCULATION
// ============================================================
float mphFromDeltaUs(unsigned long dtUs) {
  if (dtUs == 0) return 0.0;

  float dtSec = dtUs / 1000000.0;
  float inchesPerSecond = BEAM_SPACING_IN / dtSec;
  return inchesPerSecond / 17.6;
}

// ============================================================
// DEBUG HELPERS
// ============================================================
void dbg(const char* msg) {
  if (!DEBUG_MODE) return;
  Serial.println(msg);
}

void dbgForward(float rawMph, float calibratedMph, float finalMph, unsigned long count, unsigned long dtUs) {
  if (!DEBUG_MODE) return;

  Serial.print("dtUs=");
  Serial.print(dtUs);

  Serial.print(" raw=");
  Serial.print(rawMph, 2);

  Serial.print(" cal=");
  Serial.print(calibratedMph, 2);

  Serial.print(" final=");
  Serial.print(finalMph, 2);

  Serial.print(" mph COUNT=");
  Serial.println(count);
}

void dbgTimeout(const char* stateName) {
  if (!DEBUG_MODE) return;

  Serial.print("TIMEOUT ");
  Serial.println(stateName);
}
