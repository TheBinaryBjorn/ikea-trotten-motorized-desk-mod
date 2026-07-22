/**
 * @file TrottenDeskMod.ino
 * @brief Motorized IKEA Trotten desk controller with AS5600 encoder position
 * tracking, two adjustable height presets, and software travel limits.
 * Fully non-blocking.
 *
 * Encoder direction is inverted relative to raw AS5600 output - confirmed
 * empirically that this mount/gearing needs currentPosition -= delta for
 * "up" to correctly correspond to increasing position.
 *
 * Serial calibration commands (115200 baud):
 *   L  -> set current position as the LOWER travel limit
 *   H  -> set current position as the UPPER travel limit
 *   P  -> print current position / limits / presets
 *   R  -> clear stored limits (re-enter uncalibrated state)
 *
 * Preset buttons:
 *   short press  -> seek to that preset
 *   long press (>1.2s) -> save current position into that preset
 *
 * IMPORTANT: This is a clean rebuild using a new storage namespace, so no
 * limits or presets carry over from earlier testing. Recalibrate from
 * scratch: R, then L at your real bottom, H at your real top, then
 * long-press both preset buttons at the heights you want.
 */

#include <Wire.h>
#include <Preferences.h>

// --- I2C Pins ---
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// --- Motor Pins ---
const int R_EN = 25;
const int L_EN = 26;
const int RPWM = 32;
const int LPWM = 33;

// --- Button Pins ---
const int UP_BUTTON      = 4;
const int DOWN_BUTTON    = 27;
const int PRESET1_BUTTON = 14;
const int PRESET2_BUTTON = 13;

// --- AS5600 I2C Definitions ---
#define AS5600_ADDRESS 0x36
#define RAW_ANGLE_REGISTER_MSB 0x0C
const int TICKS_PER_REV = 4096;

// --- Motion Tuning ---
const int MOTOR_RAMP_INTERVAL   = 5;    // ms per PWM step while ramping
const int MAX_SPEED             = 255;
const int MIN_SEEK_SPEED        = 90;   // slowest speed while approaching a target
const long SLOWDOWN_ZONE        = 800;  // ticks from target where speed starts tapering
const long POSITION_TOLERANCE   = 15;   // ticks considered "arrived"
const unsigned long LONG_PRESS_MS = 1200;
const unsigned long DEBOUNCE_MS   = 30;

// --- Persistent Storage ---
// New namespace on purpose - guarantees a clean slate, no stale values
// from earlier direction-debugging sessions can leak in.
Preferences prefs;
long presetA = 0;
long presetB = 0;
long maxLimit = 0;
long minLimit = 0;
bool limitsConfigured = false;

// --- Encoder / Position State ---
long currentPosition = 0;   // accumulated ticks, unwrapped across revolutions
int  lastRawAngle = -1;
unsigned long lastEncoderReadTime = 0;
const unsigned long ENCODER_INTERVAL = 20;

// --- Motor State ---
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

void setup() {
  Serial.begin(115200);

  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  ledcAttach(RPWM, 5000, 8);
  ledcAttach(LPWM, 5000, 8);

  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DOWN_BUTTON, INPUT_PULLUP);
  pinMode(PRESET1_BUTTON, INPUT_PULLUP);
  pinMode(PRESET2_BUTTON, INPUT_PULLUP);

  stopMotor();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  prefs.begin("trotten2", false); // new namespace - clean slate
  presetA          = prefs.getLong("presetA", 0);
  presetB          = prefs.getLong("presetB", 0);
  maxLimit         = prefs.getLong("maxLimit", 0);
  minLimit         = prefs.getLong("minLimit", 0);
  limitsConfigured = prefs.getBool("limitsSet", false);
  currentPosition  = prefs.getLong("lastPos", 0);
  lastRawAngle = -1; // force a clean resync on first read

  Serial.println("BUILD TAG: v5-encoder-inverted-baked-in");
  if (!limitsConfigured) {
    Serial.println("No travel limits stored yet.");
    Serial.println("Move the desk manually, then send 'L' at the bottom and 'H' at the top to calibrate.");
  }
  Serial.println("Commands: L=lower limit  H=upper limit  P=status  R=reset limits");
}

void loop() {
  handleSerialCommands();

  if (millis() - lastEncoderReadTime >= ENCODER_INTERVAL) {
    readEncoder();
    lastEncoderReadTime = millis();
  }

  updateButton(btnUp);
  updateButton(btnDown);
  updateButton(btnPreset1);
  updateButton(btnPreset2);

  if (millis() - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
    Serial.printf("up=%d down=%d state=%d pos=%ld speed=%d\n",
                  btnUp.stableState, btnDown.stableState, state, currentPosition, currentSpeed);
    lastDebugPrint = millis();
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

// ---------------- Encoder ----------------

void readEncoder() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(RAW_ANGLE_REGISTER_MSB);
  if (Wire.endTransmission(false) != 0) return; // bus error, skip this cycle

  Wire.requestFrom(AS5600_ADDRESS, 2);
  if (Wire.available() < 2) return;

  uint16_t msb = Wire.read();
  uint16_t lsb = Wire.read();
  int rawAngle = ((msb & 0x0F) << 8) | lsb;

  if (lastRawAngle < 0) {
    lastRawAngle = rawAngle; // first read after boot, just sync, don't move position
    return;
  }

  int delta = rawAngle - lastRawAngle;
  if (delta >  TICKS_PER_REV / 2) delta -= TICKS_PER_REV; // wrapped backward
  if (delta < -TICKS_PER_REV / 2) delta += TICKS_PER_REV; // wrapped forward

  currentPosition -= delta; // inverted - confirmed correct for this mount
  lastRawAngle = rawAngle;
}

// ---------------- Buttons ----------------

void updateButton(Button &b) {
  bool reading = digitalRead(b.pin);
  if (reading != b.lastReading) {
    b.lastChangeTime = millis();
  }
  if ((millis() - b.lastChangeTime) > DEBOUNCE_MS && reading != b.stableState) {
    b.stableState = reading;
    if (b.stableState == LOW) {
      b.pressStartTime = millis();
      b.longPressFired = false;
      b.pressActive = true;
    }
  }
  b.lastReading = reading;
}

bool isLongPress(Button &b) {
  if (b.stableState == LOW && !b.longPressFired &&
      millis() - b.pressStartTime >= LONG_PRESS_MS) {
    b.longPressFired = true;
    return true;
  }
  return false;
}

void handlePresetButton(Button &b, long &presetSlot) {
  if (isLongPress(b)) {
    presetSlot = currentPosition;
    savePresets();
    Serial.printf("Preset saved: %ld\n", presetSlot);
  }

  if (b.stableState == HIGH && b.pressActive) {
    b.pressActive = false;
    if (!b.longPressFired) {
      // short press -> seek to preset
      targetPosition = limitsConfigured
                          ? constrain(presetSlot, minLimit, maxLimit)
                          : presetSlot;
      state = SEEKING;
    }
  }
}

// ---------------- Motor Control ----------------

void driveMotor() {
  switch (state) {
    case MANUAL_UP:
      moveDirection(true);
      break;
    case MANUAL_DOWN:
      moveDirection(false);
      break;
    case SEEKING: {
      long error = targetPosition - currentPosition;
      if (abs(error) <= POSITION_TOLERANCE) {
        stopMotor();
        state = IDLE;
      } else {
        moveDirection(error > 0);
      }
      break;
    }
    case IDLE:
    default:
      if (currentSpeed != 0) {
        stopMotor();
      }
      break;
  }
}

void moveDirection(bool up) {
  if (limitsConfigured) {
    if (up  && currentPosition >= maxLimit) { stopMotor(); return; }
    if (!up && currentPosition <= minLimit) { stopMotor(); return; }
  }

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  // Ramp speed up
  if (currentSpeed < MAX_SPEED) {
    if (millis() - lastMotorRampTime >= MOTOR_RAMP_INTERVAL) {
      currentSpeed++;
      lastMotorRampTime = millis();
    }
  }

  int speed = currentSpeed;

  // Taper speed near a seek target so it settles instead of overshooting
  if (state == SEEKING) {
    long distanceToGo = abs(targetPosition - currentPosition);
    if (distanceToGo < SLOWDOWN_ZONE) {
      int cappedSpeed = map(distanceToGo, 0, SLOWDOWN_ZONE, MIN_SEEK_SPEED, MAX_SPEED);
      speed = min(speed, cappedSpeed);
    }
  }

  if (up) {
    ledcWrite(LPWM, 0);
    ledcWrite(RPWM, speed);
  } else {
    ledcWrite(RPWM, 0);
    ledcWrite(LPWM, speed);
  }
}

void stopMotor() {
  bool wasMoving = (currentSpeed != 0);
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
