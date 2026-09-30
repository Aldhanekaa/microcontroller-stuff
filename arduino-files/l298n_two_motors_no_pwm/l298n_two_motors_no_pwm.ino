/*
  Two DC motors through an L298N, with NO PWM (Arduino Mega 2560)

  Wiring:
    Motor A: L298N OUT1/OUT2, IN1 -> Mega D7, IN2 -> Mega D8
    Motor B: L298N OUT3/OUT4, IN3 -> Mega D9, IN4 -> Mega D10
    Leave BOTH ENA and ENB jumpers installed on the L298N module.
    No Mega wire is needed for ENA or ENB.

  Connect the motor supply to the L298N and connect its ground to Mega GND.
  Follow your L298N module's instructions for its 5 V logic supply/jumper.
  Use a supply and driver rated for both motors' startup/stall current;
  do not power the motors from the Mega or USB. Keep moving parts clear.

  Parameters to edit:
    MOTOR_A and MOTOR_B: the input pins for each motor.
    RUN_MS: how long each movement lasts, in milliseconds.
    REST_MS: pause after stopping, in milliseconds.

  With ENA/ENB continuously enabled, MOTOR_STOP drives both inputs LOW,
  which electrically brakes the motor. There is no speed adjustment: each
  running motor receives the full available driver output, which is lower
  than the supply voltage because an L298N has an internal voltage drop.
  Open Serial Monitor at 115200 baud to follow the test sequence.
*/

#include <Arduino.h>

struct MotorPins {
  const char *name;
  uint8_t input1;
  uint8_t input2;
};

enum MotorDirection { MOTOR_STOP, MOTOR_FORWARD, MOTOR_REVERSE };

const MotorPins MOTOR_A = {"A", 7, 8};
const MotorPins MOTOR_B = {"B", 9, 10};
const unsigned long RUN_MS = 1500;
const unsigned long REST_MS = 750;

// prepareMotor: Set one motor's two input pins as outputs, initially stopped.
void prepareMotor(const MotorPins &motor) {
  pinMode(motor.input1, OUTPUT);
  pinMode(motor.input2, OUTPUT);
  digitalWrite(motor.input1, LOW);
  digitalWrite(motor.input2, LOW);
}

// setMotor: Select MOTOR_FORWARD, MOTOR_REVERSE, or MOTOR_STOP for one motor.
// The parameter motor selects A/B; direction selects which input goes HIGH.
void setMotor(const MotorPins &motor, MotorDirection direction) {
  // Set a known stopped state before selecting the requested direction.
  digitalWrite(motor.input1, LOW);
  digitalWrite(motor.input2, LOW);
  if (direction == MOTOR_FORWARD) {
    digitalWrite(motor.input1, HIGH);
  } else if (direction == MOTOR_REVERSE) {
    digitalWrite(motor.input2, HIGH);
  }
}

// stopBoth: Brake both motors and wait REST_MS before another movement.
void stopBoth() {
  setMotor(MOTOR_A, MOTOR_STOP);
  setMotor(MOTOR_B, MOTOR_STOP);
  delay(REST_MS);
}

// testOneMotor: Run the selected motor forward, stop, then reverse and stop.
void testOneMotor(const MotorPins &motor) {
  Serial.print(F("Motor "));
  Serial.print(motor.name);
  Serial.println(F(" forward"));
  setMotor(motor, MOTOR_FORWARD);
  delay(RUN_MS);
  stopBoth();

  Serial.print(F("Motor "));
  Serial.print(motor.name);
  Serial.println(F(" reverse"));
  setMotor(motor, MOTOR_REVERSE);
  delay(RUN_MS);
  stopBoth();
}

// testTogether: Run both motors forward, stop, then run them in opposite directions.
void testTogether() {
  Serial.println(F("Both motors forward"));
  setMotor(MOTOR_A, MOTOR_FORWARD);
  setMotor(MOTOR_B, MOTOR_FORWARD);
  delay(RUN_MS);
  stopBoth();

  Serial.println(F("Motor A forward, motor B reverse"));
  setMotor(MOTOR_A, MOTOR_FORWARD);
  setMotor(MOTOR_B, MOTOR_REVERSE);
  delay(RUN_MS);
  stopBoth();
}

// setup: Configure pins and start the serial log once after power-up.
void setup() {
  prepareMotor(MOTOR_A);
  prepareMotor(MOTOR_B);
  Serial.begin(115200);
  Serial.println(F("L298N two-motor test without PWM starting"));
  delay(1500);
}

// loop: Repeat individual and combined motor tests indefinitely.
void loop() {
  testOneMotor(MOTOR_A);
  testOneMotor(MOTOR_B);
  testTogether();
  Serial.println(F("Cycle complete; repeating in 3 seconds"));
  delay(3000);
}
