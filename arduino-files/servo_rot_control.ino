/*
  Serial-controlled position servo test (Arduino Mega 2560)

  Servo signal wire -> D9. Enter an angle from 0 to 180 in Serial Monitor
  and press Enter. Select 115200 baud and enable Newline line ending.
  Power the servo from a separate supply matching its rated voltage/current;
  connect the servo and Arduino grounds.
*/

#include <Arduino.h>
#include <Servo.h>

const uint8_t SERVO_PIN = 31;
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 360;
const size_t COMMAND_LENGTH = 8;

Servo positionServo;
char command[COMMAND_LENGTH];
size_t commandLength = 0;

void moveToAngle(int angle) {
  positionServo.write(angle);
  Serial.print(F("Servo angle: "));
  Serial.println(angle);
}

void processCommand() {
  command[commandLength] = '\0';
  char *endPointer;
  long requestedAngle = strtol(command, &endPointer, 10);
  while (*endPointer == ' ' || *endPointer == '\t') {
    ++endPointer;
  }

  if (commandLength == 0 || *endPointer != '\0' || requestedAngle < MIN_ANGLE || requestedAngle > MAX_ANGLE) {
    Serial.println(F("Enter an angle from 0 to 180."));
  } else {
    moveToAngle((int) requestedAngle);
  }

  commandLength = 0;
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char received = (char) Serial.read();

    if (received == '\n' || received == '\r') {
      if (commandLength > 0) {
        processCommand();
      }
    } else if (commandLength < COMMAND_LENGTH - 1) {
      command[commandLength++] = received;
    } else {
      commandLength = 0;
      Serial.println(F("Command too long. Enter an angle from 0 to 180."));
    }
  }
}

void setup() {
  Serial.begin(115200);
  positionServo.attach(SERVO_PIN);
  moveToAngle(40);
  Serial.println(F("Enter an angle from 0 to 180, then press Enter."));
}

void loop() {
  readSerialCommand();
}
