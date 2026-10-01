/*
  25 kg continuous-rotation servo test (Arduino Mega 2560)

  Upload ONLY if the exact servo model is continuous-rotation (360-degree)
  or has been mechanically converted for continuous rotation. An ordinary
  180/270-degree position servo cannot become a motor through software.
  Signal wire -> D9. Use a separate power supply matching the servo rating
  and current demand; connect supply and Arduino grounds. Do not power it
  from the Arduino 5 V pin or USB. Keep moving parts clear.
  Open Serial Monitor at 115200 baud and set line ending to Newline.

  Commands:
    F 3000  Rotate forward for 3000 milliseconds.
    R 3000  Rotate reverse for 3000 milliseconds.
    B       Reverse the most recent movement for the same duration.
    S       Stop immediately.

  B is only an approximate return because this servo has no position feedback.
  If it creeps at stop, adjust STOP_US a few microseconds at a time.
*/

#include <Arduino.h>
#include <Servo.h>

const uint8_t SERVO_PIN = 9;
const int STOP_US = 1500;
const int FORWARD_US = 1600;
const int REVERSE_US = 1400;
const unsigned long MAX_RUN_MS = 60000;
const size_t COMMAND_LENGTH = 20;
Servo testServo;
char command[COMMAND_LENGTH];
size_t commandLength = 0;
unsigned long lastRunMs = 0;
int lastPulseUs = FORWARD_US;
unsigned long movementStartedAt = 0;
bool movementActive = false;

void stopServo() {
  movementActive = false;
  testServo.writeMicroseconds(STOP_US);
  Serial.println(F("Servo stopped"));
}

void startServo(int pulseUs, unsigned long runMs) {
  lastPulseUs = pulseUs;
  lastRunMs = runMs;
  movementStartedAt = millis();
  movementActive = true;
  testServo.writeMicroseconds(pulseUs);
  Serial.print(F("Running for "));
  Serial.print(runMs);
  Serial.println(F(" ms"));
}

void updateMovement() {
  if (movementActive && millis() - movementStartedAt >= lastRunMs) {
    stopServo();
  }
}

void processCommand() {
  command[commandLength] = '\0';
  char *commandType = strtok(command, " \t");
  char *durationText = strtok(nullptr, " \t");

  if (commandType == nullptr) {
    return;
  }

  if (strcmp(commandType, "S") == 0) {
    stopServo();
  } else if (strcmp(commandType, "B") == 0) {
    if (lastRunMs == 0) {
      Serial.println(F("No previous movement to reverse"));
    } else {
      startServo(lastPulseUs == FORWARD_US ? REVERSE_US : FORWARD_US, lastRunMs);
    }
  } else if ((strcmp(commandType, "F") == 0 || strcmp(commandType, "R") == 0) && durationText != nullptr) {
    char *endPointer;
    long requestedMs = strtol(durationText, &endPointer, 10);
    if (*endPointer != '\0' || requestedMs <= 0 || requestedMs > MAX_RUN_MS) {
      Serial.println(F("Duration must be 1 to 60000 milliseconds"));
    } else {
      int pulseUs = strcmp(commandType, "F") == 0 ? FORWARD_US : REVERSE_US;
      startServo(pulseUs, (unsigned long) requestedMs);
    }
  } else {
    Serial.println(F("Use F 3000, R 3000, B, or S"));
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
      Serial.println(F("Command too long"));
    }
  }
}

void setup() {
  Serial.begin(115200);
  testServo.attach(SERVO_PIN);
  stopServo();
  Serial.println(F("Commands: F milliseconds, R milliseconds, B, S"));
}

void loop() {
  readSerialCommand();
  updateMovement();
}
