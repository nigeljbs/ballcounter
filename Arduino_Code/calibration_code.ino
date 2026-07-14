#include <LiquidCrystal.h>

// ============================================================
// LCD SETUP
// ============================================================
// Comparator outputs use pins 2 and 3, so LCD D6/D7 are moved to A4/A5.
LiquidCrystal lcd(12, 11, 5, 4, A4, A5);

// ============================================================
// PIN DEFINITIONS
// ============================================================
const byte BEAM1_DIGITAL_PIN   = 2;   // LM393 OUT 1
const byte BEAM2_DIGITAL_PIN   = 3;   // LM393 OUT 2

const byte BEAM1_ANALOG_PIN    = A0;  // Beam 1 analog sensor node
const byte BEAM2_ANALOG_PIN    = A1;  // Beam 2 analog sensor node

const byte LASER_CTRL_PIN      = 6;
const byte BACKLIGHT_CTRL_PIN  = 7;

// ============================================================
// SENSOR / SPEED SETTINGS
// ============================================================
// Same raw spacing basis as your main code.
const float BEAM_SPACING_IN = 1.95;

// Your comparator setup:
// clear = LOW, blocked = HIGH
const bool COMP_HIGH_WHEN_BLOCKED = true;

// Used only to confirm both beams are clear before re-arming.
const int CLEAR_TH = 700;

// Timing rules
const unsigned long SENSOR_SETTLE_MS = 300UL;
const unsigned long EVENT_TIMEOUT_US = 25000UL;
const unsigned long MIN_TIME_BETWEEN_COUNTS_MS = 100UL;

// ============================================================
// LIVE VARIABLES
// ============================================================
unsigned long ballCount = 0;
float lastRawSpeedMph = 0.0;
unsigned long lastDtUs = 0;
unsigned long lastCountMs = 0;
unsigned long enabledAtMs = 0;

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
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(LASER_CTRL_PIN, OUTPUT);
  pinMode(BACKLIGHT_CTRL_PIN, OUTPUT);

  pinMode(BEAM1_DIGITAL_PIN, INPUT);
  pinMode(BEAM2_DIGITAL_PIN, INPUT);

  digitalWrite(LASER_CTRL_PIN, HIGH);
  digitalWrite(BACKLIGHT_CTRL_PIN, HIGH);

  lcd.begin(16, 2);

  lcd.setCursor(0, 0);
  lcd.print("Raw Speed Mode  ");
  lcd.setCursor(0, 1);
  lcd.print("Starting...     ");
  delay(800);

  clearEdgeFlags();

  if (COMP_HIGH_WHEN_BLOCKED) {
    attachInterrupt(digitalPinToInterrupt(BEAM1_DIGITAL_PIN), beam1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(BEAM2_DIGITAL_PIN), beam2ISR, RISING);
  } else {
    attachInterrupt(digitalPinToInterrupt(BEAM1_DIGITAL_PIN), beam1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BEAM2_DIGITAL_PIN), beam2ISR, FALLING);
  }

  enabledAtMs = millis();
  updateDisplay();

  Serial.println("RAW SPEED CALIBRATION MODE");
  Serial.println("Columns: count, dtUs, rawMph, A0, A1");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  handleBeams();
}

// ============================================================
// DISPLAY
// ============================================================
void updateDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("Count:          ");
  lcd.setCursor(7, 0);
  lcd.print(ballCount);

  lcd.setCursor(0, 1);
  lcd.print("Raw:            ");
  lcd.setCursor(5, 1);

  if (ballCount == 0) {
    lcd.print("--.-- mph");
  } else {
    lcd.print(lastRawSpeedMph, 2);
    lcd.print(" mph");
  }
}

// ============================================================
// BEAM CLEAR CHECK
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

// ============================================================
// RAW SPEED CALCULATION
// ============================================================
float rawMphFromDeltaUs(unsigned long dtUs) {
  if (dtUs == 0) return 0.0;

  float dtSec = dtUs / 1000000.0;
  float inchesPerSecond = BEAM_SPACING_IN / dtSec;
  return inchesPerSecond / 17.6;
}

// ============================================================
// BEAM HANDLER
// ============================================================
void handleBeams() {
  if (millis() - enabledAtMs < SENSOR_SETTLE_MS) return;

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
          Serial.println("Reverse ignored");
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
        Serial.println("Timeout B1->B2");
        seqState = WAIT_FOR_CLEAR;
      }
      break;

    case WAIT_FOR_B1:
      if (b1) {
        Serial.println("Reverse ignored");
        seqState = WAIT_FOR_CLEAR;
      } else if ((nowUs - tBeam2Us) > EVENT_TIMEOUT_US) {
        Serial.println("Timeout B2->B1");
        seqState = WAIT_FOR_CLEAR;
      }
      break;

    case WAIT_FOR_CLEAR:
      if (bothBeamsClearAnalog()) {
        clearEdgeFlags();
        seqState = READY_STATE;
      }
      break;
  }
}

// ============================================================
// FORWARD EVENT
// ============================================================
void processForwardEvent(unsigned long t1, unsigned long t2) {
  unsigned long nowMs = millis();

  if (nowMs - lastCountMs < MIN_TIME_BETWEEN_COUNTS_MS) {
    Serial.println("Ignored <100ms");
    return;
  }

  unsigned long dtUs = t2 - t1;
  float rawMph = rawMphFromDeltaUs(dtUs);

  lastDtUs = dtUs;
  lastRawSpeedMph = rawMph;
  ballCount++;
  lastCountMs = nowMs;

  updateDisplay();

  int a0 = analogRead(BEAM1_ANALOG_PIN);
  int a1 = analogRead(BEAM2_ANALOG_PIN);

  Serial.print("count=");
  Serial.print(ballCount);

  Serial.print(", dtUs=");
  Serial.print(dtUs);

  Serial.print(", rawMph=");
  Serial.print(rawMph, 2);

  Serial.print(", A0=");
  Serial.print(a0);

  Serial.print(", A1=");
  Serial.println(a1);
}
