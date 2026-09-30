/*
  L298N two DC motor test (Arduino Mega 2560)

  Wiring: ENA->D5, IN1->D7, IN2->D8, ENB->D6, IN3->D9, IN4->D10.
  Connect motor A to OUT1/OUT2 and motor B to OUT3/OUT4.
  Remove the ENA and ENB jumpers so D5/D6 can control PWM speed.
  Power the motors from a supply suited to their voltage/current; connect
  the supply ground and Arduino ground together. Do not power motors from USB.
  Keep moving parts clear. Open Serial Monitor at 115200 baud.

  Parameters to edit: pin numbers below, TEST_PWM (0..255), STEP_MS,
  REST_MS. A negative speed means the opposite direction.
*/

#include <Arduino.h>

struct MotorPins {
  const char *name;
  uint8_t enable;
  uint8_t input1;
  uint8_t input2;
};

const MotorPins MOTOR_A = {"A", 5, 7, 8};
const MotorPins MOTOR_B = {"B", 6, 9, 10};
const uint8_t TEST_PWM[] = {90, 160, 230};
const unsigned long STEP_MS = 800;
const unsigned long REST_MS = 700;

// prepareMotor: Configure one L298N channel and leave it disabled.
void prepareMotor(const MotorPins &motor) {
  pinMode(motor.enable, OUTPUT);
  pinMode(motor.input1, OUTPUT);
  pinMode(motor.input2, OUTPUT);
  analogWrite(motor.enable, 0);
  digitalWrite(motor.input1, LOW);
  digitalWrite(motor.input2, LOW);
}

// setMotor: Drive one motor at signed PWM speed (-255..255); zero stops it.
void setMotor(const MotorPins &motor, int speed) {
  speed = constrain(speed, -255, 255);
  analogWrite(motor.enable, 0);  // Disable before changing direction.
  digitalWrite(motor.input1, speed > 0 ? HIGH : LOW);
  digitalWrite(motor.input2, speed < 0 ? HIGH : LOW);
  analogWrite(motor.enable, abs(speed));
}

// stopBoth: Disable both channels and pause for REST_MS before the next test.
void stopBoth() {
  setMotor(MOTOR_A, 0);
  setMotor(MOTOR_B, 0);
  delay(REST_MS);
}

// testOneMotor: Ramp one channel forward and reverse while the other is off.
void testOneMotor(const MotorPins &motor) {
  const uint8_t steps = sizeof(TEST_PWM) / sizeof(TEST_PWM[0]);
  for (int direction = 1; direction >= -1; direction -= 2) {
    for (uint8_t i = 0; i < steps; ++i) {
      int speed = direction * TEST_PWM[i];
      Serial.print(F("Motor "));
      Serial.print(motor.name);
      Serial.print(F(" PWM: "));
      Serial.println(speed);
      setMotor(motor, speed);
      delay(STEP_MS);
    }
    stopBoth();
  }
}

// testTogether: Run both channels, first in the same direction, then opposite.
void testTogether() {
  const int speed = TEST_PWM[1];
  Serial.println(F("Both motors forward"));
  setMotor(MOTOR_A, speed);
  setMotor(MOTOR_B, speed);
  delay(STEP_MS);
  stopBoth();

  Serial.println(F("Motors in opposite directions"));
  setMotor(MOTOR_A, speed);
  setMotor(MOTOR_B, -speed);
  delay(STEP_MS);
  stopBoth();
}

// setup: Initialize the driver and Serial Monitor once at power-up.
void setup() {
  prepareMotor(MOTOR_A);
  prepareMotor(MOTOR_B);
  Serial.begin(115200);
  Serial.println(F("L298N two motor test starting"));
  delay(1500);
}

// loop: Repeat the individual and combined motor checks indefinitely.
void loop() {
  testOneMotor(MOTOR_A);
  testOneMotor(MOTOR_B);
  testTogether();
  Serial.println(F("Cycle complete; repeating in 3 seconds"));
  delay(3000);
}
