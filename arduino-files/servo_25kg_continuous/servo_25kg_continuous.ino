/*
  25 kg continuous-rotation servo test (Arduino Mega 2560)

  Upload ONLY if the exact servo model is continuous-rotation (360-degree)
  or has been mechanically converted for continuous rotation. An ordinary
  180/270-degree position servo cannot become a motor through software.
  Signal wire -> D9. Use a separate power supply matching the servo rating
  and current demand; connect supply and Arduino grounds. Do not power it
  from the Arduino 5 V pin or USB. Keep moving parts clear.
  Open Serial Monitor at 115200 baud.

  Parameters to edit: SERVO_PIN, STOP_US (neutral pulse, usually near 1500),
  FORWARD_US and REVERSE_US (pulse widths in microseconds), RUN_MS, REST_MS.
  Start with the modest default offset from neutral. If it creeps at stop,
  adjust STOP_US a few microseconds at a time. Direction may be reversed by
  the servo's mounting or model. There is no angular position feedback here.
*/

#include <Arduino.h>
#include <Servo.h>

const uint8_t SERVO_PIN = 9;
const int STOP_US = 1500;
const int FORWARD_US = 1600;
const int REVERSE_US = 1400;
const unsigned long RUN_MS = 2000;
const unsigned long REST_MS = 1500;
Servo testServo;

// stopServo: Send neutral pulses and wait REST_MS for motion to stop.
void stopServo() {
  testServo.writeMicroseconds(STOP_US);
  Serial.print(F("Stop pulse (us): "));
  Serial.println(STOP_US);
  delay(REST_MS);
}

// runServo: Send a speed/direction pulse for RUN_MS, then stop.
// pulseUs is a pulse width in microseconds, not an angle.
void runServo(int pulseUs) {
  Serial.print(F("Run pulse (us): "));
  Serial.println(pulseUs);
  testServo.writeMicroseconds(pulseUs);
  delay(RUN_MS);
  stopServo();
}

// setup: Attach the servo, command neutral, and start Serial Monitor output.
void setup() {
  Serial.begin(115200);
  testServo.attach(SERVO_PIN);
  Serial.println(F("Continuous-rotation servo test starting"));
  stopServo();
}

// loop: Repeat short forward and reverse runs, stopping between them.
void loop() {
  runServo(FORWARD_US);
  runServo(REVERSE_US);
  Serial.println(F("Cycle complete; repeating in 3 seconds"));
  delay(3000);
}
