const uint8_t MOTOR1_IN1 = 13;
const uint8_t MOTOR1_IN2 = 15;

const uint8_t MOTOR2_IN1 = 17;
const uint8_t MOTOR2_IN2 = 19;
const uint8_t MOTOR1_ENABLE = 9;
const uint8_t MOTOR2_ENABLE = 10;
const uint8_t MOTOR_SPEED = 255;
const unsigned long DIRECTION_TIME_MS = 3000;
const unsigned long DIRECTION_PAUSE_MS = 50;

void setDirection(bool reverse) {
  analogWrite(MOTOR1_ENABLE, 0);
  analogWrite(MOTOR2_ENABLE, 0);

  digitalWrite(MOTOR1_IN1, reverse ? LOW : HIGH);
  digitalWrite(MOTOR1_IN2, reverse ? HIGH : LOW);
  digitalWrite(MOTOR2_IN1, reverse ? LOW : HIGH);
  digitalWrite(MOTOR2_IN2, reverse ? HIGH : LOW);

  delay(DIRECTION_PAUSE_MS);
  analogWrite(MOTOR1_ENABLE, MOTOR_SPEED);
  analogWrite(MOTOR2_ENABLE, MOTOR_SPEED);
}

void setup() {
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR1_ENABLE, OUTPUT);
  pinMode(MOTOR2_ENABLE, OUTPUT);
}

void loop() {
  setDirection(false);
  delay(DIRECTION_TIME_MS);

  setDirection(true);
  delay(DIRECTION_TIME_MS);
}
