const int R_EN = 25;
const int L_EN = 26;
const int RPWM = 32;
const int LPWM = 33;

const int UP_BUTTON = 4;
const int DOWN_BUTTON = 27;

// Track the current speed globally
int currentSpeed = 0; 

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

void loop() {
  if (digitalRead(UP_BUTTON) == LOW) {
    moveUp();
  } else if (digitalRead(DOWN_BUTTON) == LOW) {
    moveDown();
  } else {
    stopMotor();
  }
}

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

void stopMotor() {
  currentSpeed = 0; // Reset speed when stopped
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
}