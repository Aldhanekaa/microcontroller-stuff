#include <Servo.h>

const int rotatorPin1 = 9;
const int rotatorPin2 = 11;
const int e18_sensor = 29;
const uint8_t SERVO_PIN = 21;
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int INITIAL_SERVO_ANGLE = 30;
const int TRIGGER_SERVO_ANGLE = 0;
const unsigned long SERVO_HOLD_MS = 700;
const size_t COMMAND_LENGTH = 32;

Servo positionServo;
char command[COMMAND_LENGTH];
size_t commandLength = 0;

enum OperationMode {
  IDLE,
  MANUAL_ROTATION,
  RESET_POSITION,
  AUTOMATION
};

enum SensorState {
  WAITING_FOR_CLEAR,
  WAITING_FOR_TRIGGER
};

enum AutomationState {
  AUTOMATION_WAITING_FOR_CLEAR,
  AUTOMATION_ROTATING,
  AUTOMATION_HOLDING_SERVO,
  AUTOMATION_COMPLETE
};

OperationMode operationMode = IDLE;
SensorState sensorState;
AutomationState automationState = AUTOMATION_COMPLETE;
unsigned long manualStopTime = 0;
unsigned long servoHoldStartTime = 0;

void stopRotator() {
  digitalWrite(rotatorPin1, LOW);
  digitalWrite(rotatorPin2, LOW);
}

void startRotator() {
  digitalWrite(rotatorPin1, LOW );
  digitalWrite(rotatorPin2, HIGH);
}

void moveToAngle(int angle) {
  positionServo.write(angle);
  Serial.print(F("Servo angle: "));
  Serial.println(angle);
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  AUTO             Run the full automation cycle"));
  Serial.println(F("  RESET            Move to the sensor position"));
  Serial.println(F("  MANUAL <ms>      Rotate for the given milliseconds"));
  Serial.println(F("  STOP             Stop the rotator"));
  Serial.println(F("  <0-180>          Move the servo to an angle"));
}

bool parseMilliseconds(const char *text, unsigned long *milliseconds) {
  char *endPointer;
  unsigned long requestedMilliseconds = strtoul(text, &endPointer, 10);

  while (*endPointer == ' ' || *endPointer == '\t') {
    ++endPointer;
  }

  if (*text == '\0' || *endPointer != '\0' || requestedMilliseconds == 0) {
    return false;
  }

  *milliseconds = requestedMilliseconds;
  return true;
}

bool parseServoAngle(const char *text, int *angle) {
  char *endPointer;
  long requestedAngle = strtol(text, &endPointer, 10);

  while (*endPointer == ' ' || *endPointer == '\t') {
    ++endPointer;
  }

  if (*text == '\0' || *endPointer != '\0' || requestedAngle < MIN_ANGLE || requestedAngle > MAX_ANGLE) {
    return false;
  }

  *angle = (int) requestedAngle;
  return true;
}

void stopOperation() {
  stopRotator();
  operationMode = IDLE;
  automationState = AUTOMATION_COMPLETE;
  Serial.println(F("Rotator stopped."));
}

void startSensorOperation(OperationMode mode) {
  operationMode = mode;
  sensorState = digitalRead(e18_sensor) == LOW ? WAITING_FOR_CLEAR : WAITING_FOR_TRIGGER;
  startRotator();

  if (mode == RESET_POSITION) {
    Serial.println(F("Reset started."));
  } else {
    automationState = sensorState == WAITING_FOR_CLEAR
      ? AUTOMATION_WAITING_FOR_CLEAR
      : AUTOMATION_ROTATING;
    Serial.println(F("Automation started."));
  }
}

void startManualRotation(unsigned long milliseconds) {
  operationMode = MANUAL_ROTATION;
  manualStopTime = millis() + milliseconds;
  startRotator();
  Serial.print(F("Manual rotation started for "));
  Serial.print(milliseconds);
  Serial.println(F(" ms."));
}

void processCommand() {
  command[commandLength] = '\0';

  if (strcmp(command, "HELP") == 0) {
    printHelp();
  } else if (strcmp(command, "STOP") == 0) {
    stopOperation();
  } else if (strcmp(command, "RESET") == 0) {
    startSensorOperation(RESET_POSITION);
  } else if (strcmp(command, "AUTO") == 0) {
    startSensorOperation(AUTOMATION);
  } else if (strncmp(command, "MANUAL ", 7) == 0) {
    unsigned long milliseconds;
    if (parseMilliseconds(command + 7, &milliseconds)) {
      startManualRotation(milliseconds);
    } else {
      Serial.println(F("Usage: MANUAL <milliseconds>"));
    }
  } else {
    int angle;
    if (parseServoAngle(command, &angle)) {
      moveToAngle(angle);
    } else {
      Serial.println(F("Unknown command. Enter HELP for commands."));
    }
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
      Serial.println(F("Command too long."));
    }
  }
}

void updateManualRotation() {
  if ((long) (millis() - manualStopTime) >= 0) {
    stopOperation();
  }
}

void updateResetPosition(int currentSensorState) {
  if (sensorState == WAITING_FOR_CLEAR) {
    if (currentSensorState == HIGH) {
      sensorState = WAITING_FOR_TRIGGER;
      Serial.println(F("Sensor clear; looking for reset position."));
    }
  } else if (currentSensorState == LOW) {
    stopOperation();
    Serial.println(F("Reset position found."));
  }
}

void updateAutomation(int currentSensorState) {
  if (automationState == AUTOMATION_WAITING_FOR_CLEAR) {
    if (currentSensorState == HIGH) {
      automationState = AUTOMATION_ROTATING;
      Serial.println(F("Sensor clear; rotating to the next trigger."));
    }
  } else if (automationState == AUTOMATION_ROTATING) {
    if (currentSensorState == LOW) {
      stopRotator();
      delay(500);
      moveToAngle(TRIGGER_SERVO_ANGLE);
      servoHoldStartTime = millis();
      automationState = AUTOMATION_HOLDING_SERVO;
      Serial.println(F("Sensor triggered; holding servo at trigger angle."));
    }
  } else if (automationState == AUTOMATION_HOLDING_SERVO && millis() - servoHoldStartTime >= SERVO_HOLD_MS) {
    moveToAngle(INITIAL_SERVO_ANGLE);
    operationMode = IDLE;
    automationState = AUTOMATION_COMPLETE;
    Serial.println(F("Automation cycle complete."));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Init");

  pinMode(e18_sensor, INPUT);
  pinMode(rotatorPin1, OUTPUT);
  pinMode(rotatorPin2, OUTPUT);

  positionServo.attach(SERVO_PIN);
  moveToAngle(INITIAL_SERVO_ANGLE);
  stopRotator();
  printHelp();
}

void loop() {
  readSerialCommand();

  if (operationMode == MANUAL_ROTATION) {
    updateManualRotation();
  } else if (operationMode == RESET_POSITION) {
    updateResetPosition(digitalRead(e18_sensor));
  } else if (operationMode == AUTOMATION) {
    updateAutomation(digitalRead(e18_sensor));
  }
}
