/**
 * @file TrottenDeskMod.ino
 * @brief Minimal manual up/down control for the IKEA Trotten motorized desk mod.
 *
 * Drives a BTS7960 motor driver from two momentary buttons: holding UP or
 * DOWN ramps the motor up to full speed in that direction, releasing both
 * stops it. There is no position sensing or limit detection yet -- the
 * desk's own mechanical travel stops are what prevent over-travel, so avoid
 * holding a button against the end of travel longer than necessary.
 *
 * Pin map:
 *   R_EN, L_EN   - BTS7960 driver enables
 *   RPWM, LPWM   - BTS7960 PWM inputs (RPWM drives up, LPWM drives down)
 *   UP_BUTTON    - active LOW, INPUT_PULLUP
 *   DOWN_BUTTON  - active LOW, INPUT_PULLUP
 */

const int R_EN = 25;
const int L_EN = 26;
const int RPWM = 32;
const int LPWM = 33;

const int UP_BUTTON = 4;
const int DOWN_BUTTON = 27;

/** Current PWM duty cycle (0-255), ramped up while a direction is held and reset on stop. */
int currentSpeed = 0;

/**
 * @brief Arduino entry point: configures driver and button pins and stops the motor.
 */
void setup() {
  Serial.begin(115200);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  ledcAttach(RPWM, 5000, 8);
  ledcAttach(LPWM, 5000, 8);

  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DOWN_BUTTON, INPUT_PULLUP);

  stopMotor();
}

/**
 * @brief Main control loop: polls the buttons and drives the motor accordingly.
 *
 * UP takes priority over DOWN if both are pressed simultaneously. Releasing
 * both buttons stops the motor.
 */
void loop() {
  if (digitalRead(UP_BUTTON) == LOW) {
    moveUp();
  } else if (digitalRead(DOWN_BUTTON) == LOW) {
    moveDown();
  } else {
    stopMotor();
  }
}

/**
 * @brief Drives the motor upward, ramping the duty cycle up by 1/255 per call.
 *
 * Called repeatedly from loop() while UP_BUTTON is held, so the ramp
 * advances roughly one step per loop iteration (~5ms per step here) until
 * it reaches full speed.
 */
void moveUp() {
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  ledcWrite(LPWM, 0);

  // Increment speed gradually instead of looping
  if (currentSpeed < 255) {
    currentSpeed++;
    delay(5);
  }
  ledcWrite(RPWM, currentSpeed);
}

/**
 * @brief Drives the motor downward, ramping the duty cycle up by 1/255 per call.
 *
 * Mirrors moveUp() for the opposite direction; called repeatedly from
 * loop() while DOWN_BUTTON is held.
 */
void moveDown() {
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  ledcWrite(RPWM, 0);

  // Increment speed gradually instead of looping
  if (currentSpeed < 255) {
    currentSpeed++;
    delay(5);
  }
  ledcWrite(LPWM, currentSpeed);
}

/**
 * @brief Stops the motor immediately and resets the speed ramp.
 *
 * Called whenever neither button is held, and once at startup.
 */
void stopMotor() {
  currentSpeed = 0; // Reset speed when stopped
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
}