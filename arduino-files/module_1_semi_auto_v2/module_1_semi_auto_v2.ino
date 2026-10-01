/*
  Module 1 v2 - Arduino Mega 2560

  Integrates module_1_semi_auto, taking_picture_cycle, cutting_cycle, and
  button. All motion is a nonblocking state machine so serial STOP is checked
  throughout sensor waits, servo holds, cutter motion, and the final 10 s run.

  L298N board 1 (ENA/ENB jumpers ON): roll_1 IN1/IN2 = D7/D8,
    roll_2 IN3/IN4 = D9/D10.
  L298N board 2 (ENA/ENB jumpers ON): roll_3 IN1/IN2 = D24/D25,
    motor_4 (picture rotator) IN3/IN4 = D26/D27.
  IR sensors: ir_1..ir_4 = D30..D33, ir_rotation_trigger = D34,
    cutting sensor = D43. Sensor outputs are assumed active LOW.
  Button = D41 to GND (INPUT_PULLUP); ButtonLED = D39 (HIGH = ON).
  Position servo = D40; continuous cutting servo = D42. D42 replaces the
  old D41 servo signal because button.ino uses D41 for the button.

  Use suitable external motor/servo supplies and a common Mega ground.
  Serial Monitor: 115200 baud, Newline or Both NL & CR.
  Commands: STOP, E, D, AUTO, STATUS, SENSORS, HELP,
    MOTOR-1-ON/OFF through MOTOR-4-ON/OFF, ALL-ON, ALL-OFF,
    SERVO-1-1500, SERVO-2-1500 (manual pulse widths in microseconds).
  The button starts a full run and confirms each of four photo positions.
*/

#include <Arduino.h>
#include <Servo.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

struct MotorPins { const char *name; uint8_t in1; uint8_t in2; };
struct SensorPin { const char *name; uint8_t pin; };
enum MotorDirection { MOTOR_OFF, MOTOR_FORWARD, MOTOR_REVERSE };
enum RunState {
  READY, START_LED_CUE, ROLL_TO_SENSOR, WAIT_PHOTO_BUTTON,
  PHOTO_WAIT_CLEAR, PHOTO_ROTATING, PHOTO_SERVO_HOLD,
  CUT_WAIT_SENSOR, CUT_FORWARD, CUT_HOLD, CUT_REVERSE, FINAL_RUN,
  DISABLED
};

// Pin assignments retain the v1 roll wiring. motor_4 is the picture rotator.
const MotorPins MOTORS[4] = {
  {"roll_1", 7, 8}, {"roll_2", 9, 10},
  {"roll_3", 24, 25}, {"motor_4", 26, 27}
};
const SensorPin ROLL_SENSORS[4] = {
  {"ir_1", 30}, {"ir_2", 31}, {"ir_3", 32}, {"ir_4", 33}
};
const uint8_t ROTATION_SENSOR_PIN = 34;
const uint8_t CUT_SENSOR_PIN = 43;
const uint8_t BUTTON_PIN = 41;
const uint8_t BUTTON_LED_PIN = 39;
const uint8_t POSITION_SERVO_PIN = 40;
const uint8_t CUTTING_SERVO_PIN = 42;
const uint8_t SENSOR_ACTIVE_LEVEL = LOW;

// Change these directions if physical wiring makes a motor turn the wrong way.
const MotorDirection ROLL_1_DOWN = MOTOR_FORWARD;
const MotorDirection ROLL_2_FIRST_PAIR_DOWN = MOTOR_FORWARD;
const MotorDirection ROLL_2_SECOND_PAIR_DOWN = MOTOR_FORWARD;
const MotorDirection ROLL_3_DOWN = MOTOR_FORWARD;
const MotorDirection ROTATOR_DIRECTION = MOTOR_REVERSE;  // Source: LOW, HIGH.
const MotorDirection MOTOR_4_MANUAL_DIRECTION = MOTOR_FORWARD;

// All time values are milliseconds; changing them does not require new logic.
const unsigned long BUTTON_DEBOUNCE_MS = 50UL;
const unsigned long START_LED_CUE_MS = 350UL;
const unsigned long ROLL_SENSOR_DEBOUNCE_MS = 30UL;
const unsigned long ROLL_SENSOR_TIMEOUT_MS = 60000UL;
const unsigned long ROTATION_SENSOR_TIMEOUT_MS = 60000UL;
const unsigned long CUT_SENSOR_TIMEOUT_MS = 60000UL;
const bool CUT_WAIT_FOR_SENSOR = true;  // false starts the cut immediately.
const unsigned long PICTURE_SERVO_HOLD_MS = 700UL;
const unsigned long PHOTO_LED_BLINK_MS = 300UL;
const unsigned long FINAL_RUN_MS = 10000UL;
const unsigned long FINAL_LED_BLINK_MS = 1000UL;

// Values from taking_picture_cycle.ino and cutting_cycle.ino.
const int POSITION_INITIAL_ANGLE = 30;
const int POSITION_TRIGGER_ANGLE = 0;
const int CUT_STOP_US = 1500;
const int CUT_FORWARD_US = 1600;
const int CUT_REVERSE_US = 1400;
const unsigned long CUT_INITIAL_FORWARD_MS = 1800UL;
const unsigned long CUT_INITIAL_REVERSE_MS = 1700UL;
const unsigned long CUT_DURATION_INCREMENT_MS = 50UL;
const unsigned long CUT_HOLD_MS = 250UL;
const int MANUAL_SERVO_MIN_US = 1200;
const int MANUAL_SERVO_MAX_US = 1800;

Servo positionServo;
Servo cuttingServo;
MotorDirection motorState[4] = {MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF};
RunState runState = READY;
uint8_t stationIndex = 0;  // 0..3 selects ir_1..ir_4.
unsigned long stateStartedMs = 0;
unsigned long sensorLowStartedMs = 0;
bool sensorLowTiming = false;
unsigned long cutForwardDurationMs = CUT_INITIAL_FORWARD_MS;
unsigned long cutReverseDurationMs = CUT_INITIAL_REVERSE_MS;
bool cuttingSensorWasClear = true;

bool buttonLastRaw = HIGH;
bool buttonStable = HIGH;
unsigned long buttonChangedMs = 0;
bool ledOn = false;
unsigned long ledLastToggleMs = 0;
bool stopCommandThisLoop = false;
char commandBuffer[48];
uint8_t commandLength = 0;
bool discardingCommand = false;

// stateName: Return the readable name of a run state for STATUS output.
const char *stateName(RunState state) {
  switch (state) {
    case READY: return "ready for button";
    case START_LED_CUE: return "start LED cue";
    case ROLL_TO_SENSOR: return "rolling to IR sensor";
    case WAIT_PHOTO_BUTTON: return "waiting for picture button press";
    case PHOTO_WAIT_CLEAR: return "picture rotator waiting for sensor clear";
    case PHOTO_ROTATING: return "picture rotator moving";
    case PHOTO_SERVO_HOLD: return "picture servo holding trigger angle";
    case CUT_WAIT_SENSOR: return "waiting for cutting sensor";
    case CUT_FORWARD: return "cutter forward";
    case CUT_HOLD: return "cutter hold";
    case CUT_REVERSE: return "cutter reverse";
    case FINAL_RUN: return "final 10 second roll";
    default: return "disabled";
  }
}

// setMotor: Drive one L298N channel by digital direction only, no PWM.
// index is 0..3; MOTOR_OFF brakes with both inputs LOW and EN jumper fitted.
void setMotor(uint8_t index, MotorDirection direction) {
  digitalWrite(MOTORS[index].in1, LOW);
  digitalWrite(MOTORS[index].in2, LOW);
  if (direction == MOTOR_FORWARD) digitalWrite(MOTORS[index].in1, HIGH);
  if (direction == MOTOR_REVERSE) digitalWrite(MOTORS[index].in2, HIGH);
  motorState[index] = direction;
}

// stopAllMotors: Brake all four L298N channels, including the picture rotator.
void stopAllMotors() {
  for (uint8_t i = 0; i < 4; ++i) setMotor(i, MOTOR_OFF);
}

// setButtonLed: Write and remember the ButtonLED output state.
void setButtonLed(bool on) {
  ledOn = on;
  digitalWrite(BUTTON_LED_PIN, on ? HIGH : LOW);
}

// isActive: Read one active-LOW E18 sensor pin.
bool isActive(uint8_t pin) {
  return digitalRead(pin) == SENSOR_ACTIVE_LEVEL;
}

// isPictureState: Identify states that use the normal LED blink interval.
bool isPictureState() {
  return runState == PHOTO_WAIT_CLEAR || runState == PHOTO_ROTATING ||
         runState == PHOTO_SERVO_HOLD;
}

// anyMotorRunning: Include manual runs when deciding whether READY LED is lit.
bool anyMotorRunning() {
  for (uint8_t i = 0; i < 4; ++i) {
    if (motorState[i] != MOTOR_OFF) return true;
  }
  return false;
}

// updateButtonLed: Show ready/wait states steadily, other motion off, and
// blink during picture taking or the final 10-second roll at separate rates.
void updateButtonLed() {
  unsigned long interval = 0;
  if (isPictureState()) interval = PHOTO_LED_BLINK_MS;
  else if (runState == FINAL_RUN) interval = FINAL_LED_BLINK_MS;
  if (interval > 0) {
    unsigned long now = millis();
    if (now - ledLastToggleMs >= interval) {
      ledLastToggleMs = now;
      setButtonLed(!ledOn);
    }
    return;
  }
  bool shouldBeOn = runState == WAIT_PHOTO_BUTTON || runState == START_LED_CUE ||
                    (runState == READY && !anyMotorRunning());
  if (ledOn != shouldBeOn) setButtonLed(shouldBeOn);
}

// buttonPressed: Debounce the D41 button and return true once per LOW edge.
// A held button cannot start another stage without release and another press.
bool buttonPressed() {
  bool raw = digitalRead(BUTTON_PIN);
  unsigned long now = millis();
  if (raw != buttonLastRaw) buttonChangedMs = now;
  bool pressed = false;
  if (now - buttonChangedMs >= BUTTON_DEBOUNCE_MS && raw != buttonStable) {
    buttonStable = raw;
    pressed = buttonStable == LOW;
  }
  buttonLastRaw = raw;
  return pressed;
}

// stopSystem: Cancel the run, brake every DC motor, neutralize the cutter,
// restore the position servo, and return to READY or DISABLED.
void stopSystem(bool disable) {
  stopAllMotors();
  cuttingServo.writeMicroseconds(CUT_STOP_US);
  positionServo.write(POSITION_INITIAL_ANGLE);
  sensorLowTiming = false;
  runState = disable ? DISABLED : READY;
  updateButtonLed();
  Serial.println(disable ? F("System disabled; all motion stopped.") :
                           F("STOP: cycle cancelled; all motion stopped."));
}

// startRollToStation: Start the correct roll pair toward stationIndex's IR.
// Refuse motion if its target sensor is already blocked at stage entry.
void startRollToStation() {
  stopAllMotors();
  uint8_t targetPin = ROLL_SENSORS[stationIndex].pin;
  if (isActive(targetPin)) {
    Serial.print(F("Target sensor already active: "));
    Serial.println(ROLL_SENSORS[stationIndex].name);
    stopSystem(false);
    return;
  }
  if (stationIndex < 2) {
    setMotor(0, ROLL_1_DOWN);
    setMotor(1, ROLL_2_FIRST_PAIR_DOWN);
  } else {
    setMotor(1, ROLL_2_SECOND_PAIR_DOWN);
    setMotor(2, ROLL_3_DOWN);
  }
  runState = ROLL_TO_SENSOR;
  stateStartedMs = millis();
  sensorLowTiming = false;
  setButtonLed(false);
  Serial.print(F("Rolling to "));
  Serial.println(ROLL_SENSORS[stationIndex].name);
}

// startFullAutomation: Begin after a button press (or AUTO command) with a
// short visible LED cue, then start the first motor pair.
void startFullAutomation() {
  if (runState == DISABLED) {
    Serial.println(F("Disabled. Send E before pressing the button."));
    return;
  }
  if (runState != READY) {
    Serial.println(F("Cycle already running."));
    return;
  }
  stopAllMotors();
  cuttingServo.writeMicroseconds(CUT_STOP_US);
  positionServo.write(POSITION_INITIAL_ANGLE);
  for (uint8_t i = 0; i < 4; ++i) {
    if (isActive(ROLL_SENSORS[i].pin)) {
      Serial.print(F("Clear sensor before starting: "));
      Serial.println(ROLL_SENSORS[i].name);
      return;
    }
  }
  stationIndex = 0;
  runState = START_LED_CUE;
  stateStartedMs = millis();
  setButtonLed(true);
  Serial.println(F("Full automation started by button or AUTO."));
}

// updateRollToStation: Debounce the target IR, stop both roll motors, and
// illuminate ButtonLED when it is time for a user photo confirmation.
void updateRollToStation() {
  unsigned long now = millis();
  if (isActive(ROLL_SENSORS[stationIndex].pin)) {
    if (!sensorLowTiming) {
      sensorLowTiming = true;
      sensorLowStartedMs = now;
    } else if (now - sensorLowStartedMs >= ROLL_SENSOR_DEBOUNCE_MS) {
      stopAllMotors();
      runState = WAIT_PHOTO_BUTTON;
      sensorLowTiming = false;
      setButtonLed(true);
      Serial.print(F("Reached "));
      Serial.print(ROLL_SENSORS[stationIndex].name);
      Serial.println(F(". Press button for one picture cycle."));
      return;
    }
  } else {
    sensorLowTiming = false;
  }
  if (ROLL_SENSOR_TIMEOUT_MS > 0 &&
      now - stateStartedMs >= ROLL_SENSOR_TIMEOUT_MS) {
    Serial.println(F("Roll sensor timeout."));
    stopSystem(false);
  }
}

// startPictureCycle: Port the picture sketch's reset-to-clear/trigger logic.
// motor_4 turns in the source sketch's LOW/HIGH direction until D34 triggers.
void startPictureCycle() {
  positionServo.write(POSITION_INITIAL_ANGLE);
  setMotor(3, ROTATOR_DIRECTION);
  runState = isActive(ROTATION_SENSOR_PIN) ? PHOTO_WAIT_CLEAR : PHOTO_ROTATING;
  stateStartedMs = millis();
  sensorLowTiming = false;
  setButtonLed(false);
  ledLastToggleMs = stateStartedMs;
  Serial.print(F("Picture cycle started at "));
  Serial.println(ROLL_SENSORS[stationIndex].name);
}

// finishPictureCycle: Restore the position servo and select the next phase.
// After station 3 (ir_3), run the cutter before moving toward ir_4.
void finishPictureCycle() {
  positionServo.write(POSITION_INITIAL_ANGLE);
  Serial.println(F("Picture cycle complete."));
  if (stationIndex == 2) {
    cuttingServo.writeMicroseconds(CUT_STOP_US);
    runState = CUT_WAIT_SENSOR;
    stateStartedMs = millis();
    setButtonLed(false);
    Serial.println(F("Waiting for cutting sensor before cutter cycle."));
  } else if (stationIndex == 3) {
    stopAllMotors();
    setMotor(1, ROLL_2_SECOND_PAIR_DOWN);
    setMotor(2, ROLL_3_DOWN);
    runState = FINAL_RUN;
    stateStartedMs = millis();
    setButtonLed(false);
    ledLastToggleMs = stateStartedMs;
    Serial.println(F("Final roll_2 and roll_3 run: 10000 ms."));
  } else {
    ++stationIndex;
    startRollToStation();
  }
}

// updatePictureCycle: Wait for an existing blocked position sensor to clear,
// then detect its next LOW trigger, stop motor_4, hold servo at 0 deg for
// 700 ms, and restore 30 deg. The timeout also covers waiting for clear.
void updatePictureCycle() {
  unsigned long now = millis();
  if (runState == PHOTO_WAIT_CLEAR && !isActive(ROTATION_SENSOR_PIN)) {
    runState = PHOTO_ROTATING;
    Serial.println(F("Rotation sensor cleared; awaiting next trigger."));
  } else if (runState == PHOTO_ROTATING) {
    if (isActive(ROTATION_SENSOR_PIN)) {
      if (!sensorLowTiming) {
        sensorLowTiming = true;
        sensorLowStartedMs = now;
      } else if (now - sensorLowStartedMs >= ROLL_SENSOR_DEBOUNCE_MS) {
        setMotor(3, MOTOR_OFF);
        positionServo.write(POSITION_TRIGGER_ANGLE);
        runState = PHOTO_SERVO_HOLD;
        stateStartedMs = now;
        Serial.println(F("Rotation trigger reached; holding picture servo."));
      }
    } else {
      sensorLowTiming = false;
    }
  }
  if (runState == PHOTO_SERVO_HOLD) {
    if (now - stateStartedMs >= PICTURE_SERVO_HOLD_MS) finishPictureCycle();
  } else if (ROTATION_SENSOR_TIMEOUT_MS > 0 &&
             now - stateStartedMs >= ROTATION_SENSOR_TIMEOUT_MS) {
    Serial.println(F("Picture rotation sensor timeout."));
    stopSystem(false);
  }
}

// startCuttingMotion: Port the source cutter's forward/hold/reverse cycle.
// Durations start at 1800/1700 ms and increase 50 ms after each completed cut.
void startCuttingMotion() {
  cuttingServo.writeMicroseconds(CUT_FORWARD_US);
  runState = CUT_FORWARD;
  stateStartedMs = millis();
  Serial.print(F("Cutter forward "));
  Serial.print(cutForwardDurationMs);
  Serial.print(F(" ms, reverse "));
  Serial.print(cutReverseDurationMs);
  Serial.println(F(" ms."));
}

// updateCuttingCycle: Wait for a fresh active-LOW cutter sensor event, then
// execute forward, neutral hold, reverse, neutral without blocking STOP.
void updateCuttingCycle() {
  unsigned long now = millis();
  if (runState == CUT_WAIT_SENSOR) {
    if (!CUT_WAIT_FOR_SENSOR) {
      startCuttingMotion();
      return;
    }
    if (!isActive(CUT_SENSOR_PIN)) {
      cuttingSensorWasClear = true;
    } else if (cuttingSensorWasClear) {
      cuttingSensorWasClear = false;
      startCuttingMotion();
      return;
    }
    if (CUT_SENSOR_TIMEOUT_MS > 0 && now - stateStartedMs >= CUT_SENSOR_TIMEOUT_MS) {
      Serial.println(F("Cutting sensor timeout."));
      stopSystem(false);
    }
  } else if (runState == CUT_FORWARD &&
             now - stateStartedMs >= cutForwardDurationMs) {
    cuttingServo.writeMicroseconds(CUT_STOP_US);
    runState = CUT_HOLD;
    stateStartedMs = now;
    Serial.println(F("Cutter holding before reverse."));
  } else if (runState == CUT_HOLD && now - stateStartedMs >= CUT_HOLD_MS) {
    cuttingServo.writeMicroseconds(CUT_REVERSE_US);
    runState = CUT_REVERSE;
    stateStartedMs = now;
  } else if (runState == CUT_REVERSE &&
             now - stateStartedMs >= cutReverseDurationMs) {
    cuttingServo.writeMicroseconds(CUT_STOP_US);
    cutForwardDurationMs += CUT_DURATION_INCREMENT_MS;
    cutReverseDurationMs += CUT_DURATION_INCREMENT_MS;
    Serial.println(F("Cutting cycle complete; moving to ir_4."));
    stationIndex = 3;
    startRollToStation();
  }
}

// updateFinalRun: Brake roll_2/roll_3 after 10 seconds and re-arm the button.
void updateFinalRun() {
  if (millis() - stateStartedMs >= FINAL_RUN_MS) {
    stopAllMotors();
    stationIndex = 0;
    runState = READY;
    setButtonLed(true);
    Serial.println(F("Full cycle complete. Press button to start again."));
  }
}

// printSensors: Show all six sensor levels plus the debounced button state.
void printSensors() {
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(ROLL_SENSORS[i].name);
    Serial.print(F(": "));
    Serial.println(isActive(ROLL_SENSORS[i].pin) ? F("TRIGGERED") : F("clear"));
  }
  Serial.print(F("ir_rotation_trigger: "));
  Serial.println(isActive(ROTATION_SENSOR_PIN) ? F("TRIGGERED") : F("clear"));
  Serial.print(F("ir_cut_trigger: "));
  Serial.println(isActive(CUT_SENSOR_PIN) ? F("TRIGGERED") : F("clear"));
  Serial.print(F("button: "));
  Serial.println(buttonStable == LOW ? F("PRESSED") : F("released"));
}

// printStatus: Show phase, target station, motor outputs, cutter timing, and IRs.
void printStatus() {
  Serial.print(F("State: "));
  Serial.println(stateName(runState));
  Serial.print(F("Station: "));
  Serial.println(stationIndex + 1);
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(MOTORS[i].name);
    Serial.print(F(": "));
    Serial.println(motorState[i] == MOTOR_OFF ? F("OFF") :
                   motorState[i] == MOTOR_FORWARD ? F("FORWARD") : F("REVERSE"));
  }
  Serial.print(F("ButtonLED: "));
  Serial.println(ledOn ? F("ON") : F("OFF"));
  Serial.print(F("Next cutter durations, forward/reverse ms: "));
  Serial.print(cutForwardDurationMs);
  Serial.print('/');
  Serial.println(cutReverseDurationMs);
  printSensors();
}

// printHelp: List commands inherited from v1 plus button operation.
void printHelp() {
  Serial.println(F("Button: start full run; press at each of 4 IR photo stops."));
  Serial.println(F("STOP | E | D | AUTO | AUTOMATE-FULL | STATUS | SENSORS | HELP"));
  Serial.println(F("MOTOR-1-ON/OFF .. MOTOR-4-ON/OFF | ALL-ON | ALL-OFF"));
  Serial.println(F("SERVO-1-1500 | SERVO-2-1500 (manual pulses 1200..1800 us)"));
}

// processCommand: Keep v1 serial controls; STOP and D work in every state.
// Manual motor and servo commands are allowed only when READY.
void processCommand(char *command) {
  if (strcmp(command, "STOP") == 0 || strcmp(command, "ALL-OFF") == 0) {
    stopSystem(false);
    stopCommandThisLoop = true;
    return;
  }
  if (strcmp(command, "D") == 0) {
    stopSystem(true);
    stopCommandThisLoop = true;
    return;
  }
  if (strcmp(command, "E") == 0) {
    if (runState == DISABLED) runState = READY;
    Serial.println(F("System enabled."));
    return;
  }
  if (strcmp(command, "STATUS") == 0 || strcmp(command, "STATE") == 0) {
    printStatus();
    return;
  }
  if (strcmp(command, "SENSORS") == 0) {
    printSensors();
    return;
  }
  if (strcmp(command, "HELP") == 0) {
    printHelp();
    return;
  }
  if (strcmp(command, "AUTO") == 0 || strcmp(command, "AUTOMATE-FULL") == 0) {
    startFullAutomation();
    return;
  }
  if (runState != READY) {
    Serial.println(F("Manual commands require READY. Use STOP first."));
    return;
  }
  if (strcmp(command, "ALL-ON") == 0) {
    setMotor(0, ROLL_1_DOWN);
    setMotor(1, ROLL_2_FIRST_PAIR_DOWN);
    setMotor(2, ROLL_3_DOWN);
    setMotor(3, MOTOR_4_MANUAL_DIRECTION);
    Serial.println(F("All four motors ON."));
    return;
  }
  if (strncmp(command, "MOTOR-", 6) == 0 &&
      command[6] >= '1' && command[6] <= '4' && command[7] == '-') {
    uint8_t index = command[6] - '1';
    const MotorDirection directions[4] = {
      ROLL_1_DOWN, ROLL_2_FIRST_PAIR_DOWN, ROLL_3_DOWN, MOTOR_4_MANUAL_DIRECTION
    };
    if (strcmp(command + 8, "ON") == 0) {
      setMotor(index, directions[index]);
      Serial.print(MOTORS[index].name);
      Serial.println(F(" ON"));
      return;
    }
    if (strcmp(command + 8, "OFF") == 0) {
      setMotor(index, MOTOR_OFF);
      Serial.print(MOTORS[index].name);
      Serial.println(F(" OFF"));
      return;
    }
  }
  if (strncmp(command, "SERVO-", 6) == 0 &&
      (command[6] == '1' || command[6] == '2') && command[7] == '-') {
    char *end = NULL;
    long pulseUs = strtol(command + 8, &end, 10);
    if (end != command + 8 && *end == '\0' &&
        pulseUs >= MANUAL_SERVO_MIN_US && pulseUs <= MANUAL_SERVO_MAX_US) {
      if (command[6] == '1') positionServo.writeMicroseconds((int)pulseUs);
      else cuttingServo.writeMicroseconds((int)pulseUs);
      Serial.print(F("Servo "));
      Serial.print(command[6]);
      Serial.print(F(" pulse: "));
      Serial.println(pulseUs);
      return;
    }
    Serial.println(F("Invalid servo pulse. Use 1200..1800 us."));
    return;
  }
  Serial.println(F("Unknown command. Send HELP."));
}

// pollSerial: Parse newline-terminated commands a character at a time, so
// sensor waits and timed actions never block emergency STOP reception.
void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (discardingCommand) {
        discardingCommand = false;
        commandLength = 0;
        Serial.println(F("Command too long."));
      } else if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
      continue;
    }
    if (discardingCommand) continue;
    if (commandLength >= sizeof(commandBuffer) - 1) {
      discardingCommand = true;
      continue;
    }
    if (c >= 32 && c <= 126) {
      commandBuffer[commandLength++] = (char)toupper((unsigned char)c);
    }
  }
}

// updateState: Advance only the current stage, without delay() calls.
void updateState() {
  switch (runState) {
    case START_LED_CUE:
      if (millis() - stateStartedMs >= START_LED_CUE_MS) startRollToStation();
      break;
    case ROLL_TO_SENSOR: updateRollToStation(); break;
    case PHOTO_WAIT_CLEAR:
    case PHOTO_ROTATING:
    case PHOTO_SERVO_HOLD: updatePictureCycle(); break;
    case CUT_WAIT_SENSOR:
    case CUT_FORWARD:
    case CUT_HOLD:
    case CUT_REVERSE: updateCuttingCycle(); break;
    case FINAL_RUN: updateFinalRun(); break;
    default: break;
  }
}

// setup: Configure all inputs/outputs, set servos to source start values,
// initialize the button debounce state, and show that the system is READY.
void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(MOTORS[i].in1, OUTPUT);
    pinMode(MOTORS[i].in2, OUTPUT);
    setMotor(i, MOTOR_OFF);
    pinMode(ROLL_SENSORS[i].pin, INPUT_PULLUP);
  }
  pinMode(ROTATION_SENSOR_PIN, INPUT_PULLUP);
  pinMode(CUT_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_LED_PIN, OUTPUT);
  buttonLastRaw = digitalRead(BUTTON_PIN);
  buttonStable = buttonLastRaw;
  positionServo.attach(POSITION_SERVO_PIN);
  cuttingServo.attach(CUTTING_SERVO_PIN);
  positionServo.write(POSITION_INITIAL_ANGLE);
  cuttingServo.writeMicroseconds(CUT_STOP_US);
  setButtonLed(true);
  Serial.println(F("Module 1 v2 ready. Press button to start."));
  printHelp();
}

// loop: Prioritize serial STOP, process a debounced button press, advance
// motion, and update the ButtonLED. The cutter sensor rearms when clear.
void loop() {
  stopCommandThisLoop = false;
  pollSerial();
  bool pressed = buttonPressed();
  if (!isActive(CUT_SENSOR_PIN)) cuttingSensorWasClear = true;
  if (!stopCommandThisLoop && pressed) {
    if (runState == READY) startFullAutomation();
    else if (runState == WAIT_PHOTO_BUTTON) startPictureCycle();
  }
  if (!stopCommandThisLoop) updateState();
  updateButtonLed();
}
