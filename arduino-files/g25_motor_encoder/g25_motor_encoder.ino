/*
  G25 geared DC motor + quadrature encoder test (Arduino Mega 2560)

  This assumes the motor is driven by ONE L298N channel:
  ENA->D5, IN1->D7, IN2->D8, motor->OUT1/OUT2.
  Remove the ENA jumper to enable PWM control.
  Encoder channel A->D2, channel B->D3, encoder GND->Arduino GND.
  Power the encoder with the voltage stated on its own specification.
  Its A/B outputs must be safe for the Arduino input voltage (5 V on Mega).
  Motor supply ground and Arduino ground must be connected. Use a motor
  supply suited to the motor and driver, never the Arduino USB 5 V pin.
  Open Serial Monitor at 115200 baud. Keep moving parts clear.

  Parameters to edit: pins, TEST_PWM (0..255), RUN_MS, SAMPLE_MS, REST_MS.
  ENCODER_EDGES_PER_OUTPUT_REV is the number of quadrature transitions per
  geared output-shaft revolution. Leave at 0 when unknown: raw ticks still
  test both encoder channels, and RPM is omitted. Do not use an unverified
  pulse-per-motor-revolution value as output-shaft edges/revolution.
  This test reports commanded PWM and encoder motion; it cannot measure
  electrical power, current, or torque without additional sensors.
*/

#include <Arduino.h>

const uint8_t MOTOR_ENABLE = 5;
const uint8_t MOTOR_IN1 = 7;
const uint8_t MOTOR_IN2 = 8;
const uint8_t ENCODER_A = 2;
const uint8_t ENCODER_B = 3;
const uint8_t TEST_PWM[] = {90, 160, 230};
const unsigned long RUN_MS = 3000;
const unsigned long SAMPLE_MS = 500;
const unsigned long REST_MS = 1000;
const float ENCODER_EDGES_PER_OUTPUT_REV = 0.0f;

volatile long encoderTicks = 0;
volatile unsigned long invalidTransitions = 0;
volatile uint8_t previousEncoderState = 0;

// updateEncoder: Count every valid A/B transition; called by both pin interrupts.
void updateEncoder() {
  uint8_t state = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
  uint8_t transition = (previousEncoderState << 2) | state;
  switch (transition) {
    case 0x1: case 0x7: case 0xE: case 0x8: ++encoderTicks; break;
    case 0x2: case 0xB: case 0xD: case 0x4: --encoderTicks; break;
    default:
      if (state != previousEncoderState) ++invalidTransitions;
      break;
  }
  previousEncoderState = state;
}

// readEncoder: Copy the 32-bit tick and error counters without ISR changes.
void readEncoder(long &ticks, unsigned long &errors) {
  noInterrupts();
  ticks = encoderTicks;
  errors = invalidTransitions;
  interrupts();
}

// resetEncoder: Start each motor run with fresh tick and error counts.
void resetEncoder() {
  noInterrupts();
  encoderTicks = 0;
  invalidTransitions = 0;
  interrupts();
}

// setMotor: Set signed PWM speed (-255..255); zero disables the H-bridge.
void setMotor(int speed) {
  speed = constrain(speed, -255, 255);
  analogWrite(MOTOR_ENABLE, 0);
  digitalWrite(MOTOR_IN1, speed > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_IN2, speed < 0 ? HIGH : LOW);
  analogWrite(MOTOR_ENABLE, abs(speed));
}

// runTest: Drive at signed PWM for RUN_MS and print ticks every SAMPLE_MS.
// If calibrated, print signed output-shaft RPM for each sample as well.
void runTest(int speed) {
  resetEncoder();
  Serial.print(F("Commanded PWM: "));
  Serial.println(speed);
  setMotor(speed);

  unsigned long startTime = millis();
  unsigned long lastTime = startTime;
  long lastTicks = 0;
  while (millis() - startTime < RUN_MS) {
    delay(SAMPLE_MS);
    unsigned long now = millis();
    long ticks;
    unsigned long errors;
    readEncoder(ticks, errors);
    long delta = ticks - lastTicks;
    unsigned long elapsed = now - lastTime;
    Serial.print(F("Total ticks: "));
    Serial.print(ticks);
    Serial.print(F(" | interval ticks: "));
    Serial.print(delta);
    Serial.print(F(" | invalid transitions: "));
    Serial.print(errors);
    if (ENCODER_EDGES_PER_OUTPUT_REV > 0.0f && elapsed > 0) {
      float rpm = delta * 60000.0f / (ENCODER_EDGES_PER_OUTPUT_REV * elapsed);
      Serial.print(F(" | output RPM: "));
      Serial.print(rpm, 1);
    }
    Serial.println();
    lastTicks = ticks;
    lastTime = now;
  }

  setMotor(0);
  long finalTicks;
  unsigned long errors;
  readEncoder(finalTicks, errors);
  if (finalTicks == 0) {
    Serial.println(F("No net ticks: check encoder wiring; shaft may also be stalled."));
  }
  Serial.println(F("Motor stopped"));
  delay(REST_MS);
}

// setup: Initialize motor pins, A/B interrupts, and the Serial Monitor.
void setup() {
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  setMotor(0);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  previousEncoderState = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);
  Serial.begin(115200);
  Serial.println(F("G25 motor and quadrature encoder test starting"));
  delay(1500);
}

// loop: Check three PWM levels in both directions, then repeat.
void loop() {
  const uint8_t steps = sizeof(TEST_PWM) / sizeof(TEST_PWM[0]);
  for (int direction = 1; direction >= -1; direction -= 2) {
    for (uint8_t i = 0; i < steps; ++i) {
      runTest(direction * TEST_PWM[i]);
    }
  }
  Serial.println(F("Cycle complete; repeating in 3 seconds"));
  delay(3000);
}
