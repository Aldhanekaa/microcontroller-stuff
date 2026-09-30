/*
  25 kg position-controlled servo test (Arduino Mega 2560)

  For a standard angle/position servo only. Signal wire -> D9.
  Supply servo power from a separate supply that matches its rated voltage
  and can provide its required current; connect servo and Arduino grounds.
  Do not power a high-torque servo from the Arduino 5 V pin or USB.
  Keep the horn clear of obstructions. Open Serial Monitor at 115200 baud.

  Parameters to edit: SERVO_PIN, MIN_ANGLE, MAX_ANGLE, STEP_DEGREES,
  STEP_MS, END_PAUSE_MS. Defaults stay within 45..135 degrees to avoid
  unverified mechanical end stops. Change the limits only after checking
  the exact servo model and attached mechanism. Servo.write() requests a
  position; actual travel depends on the servo model and is not measured.
*/

#include <Arduino.h>
#include <Servo.h>

const uint8_t SERVO_PIN = 9;
const int MIN_ANGLE = 45;
const int MAX_ANGLE = 135;
const int STEP_DEGREES = 10;
const unsigned long STEP_MS = 350;
const unsigned long END_PAUSE_MS = 1000;
Servo testServo;

// moveTo: Command an angle in degrees and let the servo settle for STEP_MS.
void moveTo(int angle) {
  Serial.print(F("Target angle: "));
  Serial.println(angle);
  testServo.write(angle);
  delay(STEP_MS);
}

// sweepOnce: Step from MIN_ANGLE to MAX_ANGLE and back; pause at each end.
void sweepOnce() {
  for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += STEP_DEGREES) {
    moveTo(angle);
  }
  delay(END_PAUSE_MS);
  for (int angle = MAX_ANGLE - STEP_DEGREES; angle >= MIN_ANGLE; angle -= STEP_DEGREES) {
    moveTo(angle);
  }
  delay(END_PAUSE_MS);
}

// setup: Attach the servo and command the starting angle once.
void setup() {
  Serial.begin(115200);
  testServo.attach(SERVO_PIN);
  moveTo(MIN_ANGLE);
  Serial.println(F("25 kg position servo test starting"));
}

// loop: Repeat the angle sweep until power is removed.
void loop() {
  sweepOnce();
}
