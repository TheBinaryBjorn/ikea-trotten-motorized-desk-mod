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
  ledcWrite(RPWM, currentSpeed);
}

void moveDown() {
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  ledcWrite(RPWM, 0);

  // Non-blocking speed ramp
  if (currentSpeed < 255) {
    if (millis() - lastMotorRampTime >= MOTOR_RAMP_INTERVAL) {
      currentSpeed++;
      lastMotorRampTime = millis();
    }
  }
  ledcWrite(LPWM, currentSpeed);
}

void stopMotor() {
  currentSpeed = 0;
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
}