/**
 * @file TrottenDeskMod.ino
 * @brief Manual up/down control for the IKEA Trotten motorized desk mod
 * with AS5600 magnetic encoder position tracking (fully non-blocking).
 */

#include <Wire.h>

// --- I2C Pins ---
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// --- Motor Pins ---
const int R_EN = 25;
const int L_EN = 26;
const int RPWM = 32;
const int LPWM = 33;
const int UP_BUTTON = 4;
const int DOWN_BUTTON = 27;

// --- AS5600 I2C Definitions ---
#define AS5600_ADDRESS 0x36
#define RAW_ANGLE_REGISTER_MSB 0x0C

// --- State & Timers ---
int currentSpeed = 0;
unsigned long lastMotorRampTime = 0;

enum ControlState { IDLE, MANUAL_UP, MANUAL_DOWN, SEEKING };
ControlState state = IDLE;
long targetPosition = 0;

// --- Debug print throttling ---
unsigned long lastDebugPrint = 0;
const unsigned long DEBUG_PRINT_INTERVAL = 150;

// --- Button Debounce State ---
struct Button {
  int pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeTime;
  unsigned long pressStartTime;
  bool longPressFired;
  bool pressActive;
};

Button btnUp      = {UP_BUTTON,      HIGH, HIGH, 0, 0, false, false};
Button btnDown     = {DOWN_BUTTON,    HIGH, HIGH, 0, 0, false, false};
Button btnPreset1 = {PRESET1_BUTTON, HIGH, HIGH, 0, 0, false, false};
Button btnPreset2 = {PRESET2_BUTTON, HIGH, HIGH, 0, 0, false, false};

unsigned long lastEncoderReadTime = 0;
const unsigned long ENCODER_INTERVAL = 50; 

unsigned long lastMotorRampTime = 0;
const unsigned long MOTOR_RAMP_INTERVAL = 5; 

void setup() {
  Serial.begin(115200);
  
  // Motor Setup
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  ledcAttach(RPWM, 5000, 8);
  ledcAttach(LPWM, 5000, 8);
  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DOWN_BUTTON, INPUT_PULLUP);
  stopMotor();

  // Encoder Setup
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
}

void loop() {
  // Non-blocking encoder read
  if (millis() - lastEncoderReadTime >= ENCODER_INTERVAL) {
    readEncoder();
    lastEncoderReadTime = millis();
  }

  // Motor control logic
  if (digitalRead(UP_BUTTON) == LOW) {
    moveUp();
  } else if (digitalRead(DOWN_BUTTON) == LOW) {
    moveDown();
  } else {
    stopMotor();
  }

  // Manual buttons always take priority and cancel any preset seek in progress
  if (btnUp.stableState == LOW) {
    state = MANUAL_UP;
  } else if (btnDown.stableState == LOW) {
    state = MANUAL_DOWN;
  } else if (state == MANUAL_UP || state == MANUAL_DOWN) {
    state = IDLE; // manual button just released
  }

  handlePresetButton(btnPreset1, presetA);
  handlePresetButton(btnPreset2, presetB);

  driveMotor();
}

void readEncoder() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(RAW_ANGLE_REGISTER_MSB);
  Wire.endTransmission(false);
  
  Wire.requestFrom(AS5600_ADDRESS, 2);
  
  if (Wire.available() >= 2) {
    uint16_t msb = Wire.read();
    uint16_t lsb = Wire.read();
    
    uint16_t rawAngle = ((msb & 0x0F) << 8) | lsb;
    float degrees = rawAngle * (360.0 / 4096.0);
    
    Serial.printf("Desk Angle: %6.2f°\n", degrees);
  }
}

void moveUp() {
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  ledcWrite(LPWM, 0);

  // Non-blocking speed ramp
  if (currentSpeed < 255) {
    if (millis() - lastMotorRampTime >= MOTOR_RAMP_INTERVAL) {
      currentSpeed++;
      lastMotorRampTime = millis();
    }
  }

  int delta = rawAngle - lastRawAngle;
  if (delta >  TICKS_PER_REV / 2) delta -= TICKS_PER_REV; // wrapped backward
  if (delta < -TICKS_PER_REV / 2) delta += TICKS_PER_REV; // wrapped forward

  currentPosition -= delta; // inverted - confirmed correct for this mount
  lastRawAngle = rawAngle;
}

void moveDown() {
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  // Non-blocking speed ramp
  if (currentSpeed < 255) {
    if (millis() - lastMotorRampTime >= MOTOR_RAMP_INTERVAL) {
      currentSpeed++;
      lastMotorRampTime = millis();
    }
  }
}

void stopMotor() {
  currentSpeed = 0;
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  if (wasMoving) {
    prefs.putLong("lastPos", currentPosition); // persist only on actual stop events
  }
}

// ---------------- Calibration / Persistence ----------------

void savePresets() {
  prefs.putLong("presetA", presetA);
  prefs.putLong("presetB", presetB);
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'L': case 'l':
      minLimit = currentPosition;
      prefs.putLong("minLimit", minLimit);
      checkLimitsConfigured();
      Serial.printf("Lower limit set at %ld\n", minLimit);
      break;
    case 'H': case 'h':
      maxLimit = currentPosition;
      prefs.putLong("maxLimit", maxLimit);
      checkLimitsConfigured();
      Serial.printf("Upper limit set at %ld\n", maxLimit);
      break;
    case 'P': case 'p':
      Serial.printf("Position: %ld  (min=%ld max=%ld presetA=%ld presetB=%ld) configured=%d\n",
                    currentPosition, minLimit, maxLimit, presetA, presetB, limitsConfigured);
      break;
    case 'R': case 'r':
      limitsConfigured = false;
      prefs.putBool("limitsSet", false);
      Serial.println("Limits cleared.");
      break;
  }
}

void checkLimitsConfigured() {
  if (maxLimit != minLimit) {
    if (minLimit > maxLimit) { long t = minLimit; minLimit = maxLimit; maxLimit = t; }
    limitsConfigured = true;
    prefs.putBool("limitsSet", true);
  }
}
