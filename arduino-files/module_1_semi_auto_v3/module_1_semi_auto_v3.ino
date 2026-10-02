/*
  Module 1 v3 - Arduino Mega 2560

  Each photo stop runs in this order: roll to its IR sensor, wait 800 ms,
  rotate motor_4 to the picture E18 sensor, wait for a button press, blink the
  LED during a 5-second countdown, then trigger and restore the photo servo.
  Timed actions use millis() so serial STOP is checked throughout the run.

  L298N board 1 (ENA/ENB jumpers ON): roll_1 D13/D15, roll_2 D9/D11.
  L298N board 2 (ENA/ENB jumpers ON): roll_3 D19/D17, motor_4 D5/D7.
  IR sensors: ir_1..ir_4 = D27/D23/D43/D25, picture E18 = D29,
    cutting E18 = D45 (reported only). Sensor outputs are active LOW.
  Button = D41 to GND (INPUT_PULLUP); ButtonLED = D39 (HIGH = ON).
  Position servo = D21; continuous cutting servo = D31.

  Use suitable external motor/servo supplies and a common Mega ground.
  Serial Monitor: 115200 baud, Newline or Both NL & CR.
  Commands: STOP, E, D, AUTO, STATUS, SENSORS, HELP,
    MOTOR-1-ON/OFF through MOTOR-4-ON/OFF, ALL-ON, ALL-OFF,
    SERVO-1-1500, SERVO-2-1500 (manual pulse widths in microseconds).
  The button starts a full run and starts the servo countdown at each photo.
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
  READY, START_LED_CUE, ROLL_TO_SENSOR, ROLL_TO_ROTATION_DELAY,
  PHOTO_WAIT_CLEAR, PHOTO_ROTATING, WAIT_PHOTO_BUTTON,
  PHOTO_COUNTDOWN, PHOTO_SERVO_HOLD, POST_PHOTO_PAUSE,
  CUT_WAIT_SENSOR, CUT_FORWARD, CUT_HOLD, CUT_REVERSE, FINAL_RUN,
  DISABLED
};

// Pin assignments follow the v2 sketch. motor_4 is the picture rotator.
const MotorPins MOTORS[4] = {
  {"roll_1", 13, 15}, {"roll_2", 9, 11},
  {"roll_3", 19, 17}, {"motor_4", 5, 7}
};
const SensorPin ROLL_SENSORS[4] = {
  {"ir_1", 27}, {"ir_2", 23}, {"ir_3", 43}, {"ir_4", 25}
};
const uint8_t PICTURE_CYCLE_E18_PIN = 29;    // taking_picture_cycle e18_sensor.
const uint8_t CUTTING_CYCLE_E18_PIN = 45;    // Reported only; D43 is ir_3.
const uint8_t BUTTON_PIN = 41;
const uint8_t BUTTON_LED_PIN = 39;
const uint8_t POSITION_SERVO_PIN = 21;
const uint8_t CUTTING_SERVO_PIN = 31;
const uint8_t SENSOR_ACTIVE_LEVEL = LOW;

// Change these directions if physical wiring makes a motor turn the wrong way.
const MotorDirection ROLL_1_DOWN = MOTOR_FORWARD;
const MotorDirection ROLL_2_FIRST_PAIR_DOWN = MOTOR_FORWARD;
const MotorDirection ROLL_2_SECOND_PAIR_DOWN = MOTOR_FORWARD;
const MotorDirection ROLL_3_DOWN = MOTOR_FORWARD;
const MotorDirection ROTATOR_DIRECTION = MOTOR_REVERSE;  // Source: LOW, HIGH.
const MotorDirection MOTOR_4_MANUAL_DIRECTION = MOTOR_FORWARD;

// Timing values are milliseconds except PHOTO_COUNTDOWN_SECONDS.
const unsigned long BUTTON_DEBOUNCE_MS = 50UL;
const unsigned long START_LED_CUE_MS = 350UL;
const unsigned long ROLL_SENSOR_DEBOUNCE_MS = 30UL;
const unsigned long ROLL_SENSOR_TIMEOUT_MS = 60000UL;
const unsigned long ROLL_TO_ROTATION_DELAY_MS = 800UL;
const unsigned long ROTATION_SENSOR_TIMEOUT_MS = 60000UL;
const uint8_t PHOTO_COUNTDOWN_SECONDS = 5;
const unsigned long PHOTO_COUNTDOWN_MS = PHOTO_COUNTDOWN_SECONDS * 1000UL;
const unsigned long PICTURE_SERVO_HOLD_MS = 1000UL;
const unsigned long PHOTO_LED_BLINK_MS = 300UL;
const unsigned long POST_PHOTO_PAUSE_MS = 1000UL;
const unsigned long POST_PHOTO_PAUSE_AFTER_IR_3_OR_4_MS = 800UL;
const unsigned long FINAL_RUN_MS = 10000UL;
const unsigned long FINAL_LED_BLINK_MS = 1000UL;

// Values from taking_picture_cycle.ino and cutting_cycle.ino.
const int POSITION_INITIAL_ANGLE = 90;
const int POSITION_TRIGGER_ANGLE = 40;
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
uint8_t lastCountdownSecond = PHOTO_COUNTDOWN_SECONDS;

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
    case ROLL_TO_ROTATION_DELAY: return "waiting 800 ms before picture rotation";
    case PHOTO_WAIT_CLEAR: return "picture rotator waiting for sensor clear";
    case PHOTO_ROTATING: return "picture rotator moving";
    case WAIT_PHOTO_BUTTON: return "waiting for picture button press";
    case PHOTO_COUNTDOWN: return "picture countdown";
    case PHOTO_SERVO_HOLD: return "picture servo holding trigger angle";
    case POST_PHOTO_PAUSE: return "pause after picture";
    case CUT_WAIT_SENSOR: return "starting cutter cycle";
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
         runState == PHOTO_COUNTDOWN || runState == PHOTO_SERVO_HOLD;
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

// updateRollToStation: Stop at a stable target IR and wait 800 ms before
// starting the picture rotator. The button is enabled only after rotation.
void updateRollToStation() {
  unsigned long now = millis();
  if (isActive(ROLL_SENSORS[stationIndex].pin)) {
    if (!sensorLowTiming) {
      sensorLowTiming = true;
      sensorLowStartedMs = now;
    } else if (now - sensorLowStartedMs >= ROLL_SENSOR_DEBOUNCE_MS) {
      stopAllMotors();
      runState = ROLL_TO_ROTATION_DELAY;
      stateStartedMs = now;
      sensorLowTiming = false;
      Serial.print(F("Reached "));
      Serial.print(ROLL_SENSORS[stationIndex].name);
      Serial.println(F(". Picture rotation starts in 800 ms."));
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

// startPictureRotation: Turn motor_4 until D29 clears and triggers again.
// Keep this autonomous rotation separate from the button-driven servo action.
void startPictureRotation() {
  // setMotor(3, ROTATOR_DIRECTION);
  digitalWrite(9, LOW );
  digitalWrite(11, HIGH);


  runState = isActive(PICTURE_CYCLE_E18_PIN) ? PHOTO_WAIT_CLEAR : PHOTO_ROTATING;
  stateStartedMs = millis();
  sensorLowTiming = false;
  setButtonLed(false);
  ledLastToggleMs = stateStartedMs;
  Serial.print(F("Picture rotation started at "));
  Serial.println(ROLL_SENSORS[stationIndex].name);
}

// updatePictureRotation: Wait for an initially blocked E18 to clear, then
// debounce its next LOW trigger. Stop motor_4 before accepting a photo press.
void updatePictureRotation() {
  unsigned long now = millis();
  if (runState == PHOTO_WAIT_CLEAR && !isActive(PICTURE_CYCLE_E18_PIN)) {
    runState = PHOTO_ROTATING;
    sensorLowTiming = false;
    Serial.println(F("Rotation sensor cleared; awaiting next trigger."));
  } else if (runState == PHOTO_ROTATING) {
    if (isActive(PICTURE_CYCLE_E18_PIN)) {
      if (!sensorLowTiming) {
        sensorLowTiming = true;
        sensorLowStartedMs = now;
      } else if (now - sensorLowStartedMs >= ROLL_SENSOR_DEBOUNCE_MS) {
        digitalWrite(9, LOW );
        digitalWrite(11, LOW);

        sensorLowTiming = false;
        runState = WAIT_PHOTO_BUTTON;
        setButtonLed(true);
        Serial.print(F("Rotation complete at "));
        Serial.print(ROLL_SENSORS[stationIndex].name);
        Serial.println(F(". Press button to start the 5-second photo countdown."));
        return;
      }
    } else {
      sensorLowTiming = false;
    }
  }
  if (ROTATION_SENSOR_TIMEOUT_MS > 0 &&
      now - stateStartedMs >= ROTATION_SENSOR_TIMEOUT_MS) {
    Serial.println(F("Picture rotation sensor timeout."));
    stopSystem(false);
  }
}

// startPictureCycle: The button now starts only the servo countdown.
void startPictureCycle() {
  positionServo.write(POSITION_INITIAL_ANGLE);
  runState = PHOTO_COUNTDOWN;
  stateStartedMs = millis();
  lastCountdownSecond = PHOTO_COUNTDOWN_SECONDS;
  setButtonLed(false);
  ledLastToggleMs = stateStartedMs;
  Serial.print(F("Photo in "));
  Serial.print(PHOTO_COUNTDOWN_SECONDS);
  Serial.println(F("..."));
}

// finishPictureCycle: Restore the servo, then preserve v2's station pause.
void finishPictureCycle() {
  positionServo.write(POSITION_INITIAL_ANGLE);
  Serial.println(F("Picture cycle complete."));
  runState = POST_PHOTO_PAUSE;
  stateStartedMs = millis();
  setButtonLed(false);
}

// advanceAfterPhotoPause: After station 3, cut before rolling to station 4.
void advanceAfterPhotoPause() {
  if (stationIndex == 2) {
    cuttingServo.writeMicroseconds(CUT_STOP_US);
    runState = CUT_WAIT_SENSOR;
    stateStartedMs = millis();
    setButtonLed(false);
    Serial.println(F("Starting cutter cycle without waiting for sensor."));
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

// updatePictureCycle: Blink throughout a five-second button countdown, then
// trigger the position servo, hold it, and restore it.
void updatePictureCycle() {
  unsigned long now = millis();
  if (runState == PHOTO_COUNTDOWN) {
    unsigned long elapsed = now - stateStartedMs;
    if (elapsed >= PHOTO_COUNTDOWN_MS) {
      positionServo.write(POSITION_TRIGGER_ANGLE);
      runState = PHOTO_SERVO_HOLD;
      stateStartedMs = now;
      Serial.println(F("Photo countdown complete; holding picture servo."));
    } else {
      uint8_t secondsRemaining = PHOTO_COUNTDOWN_SECONDS - (uint8_t)(elapsed / 1000UL);
      if (secondsRemaining != lastCountdownSecond) {
        lastCountdownSecond = secondsRemaining;
        Serial.print(F("Photo in "));
        Serial.print(secondsRemaining);
        Serial.println(F("..."));
      }
    }
  } else if (runState == PHOTO_SERVO_HOLD &&
             now - stateStartedMs >= PICTURE_SERVO_HOLD_MS) {
    finishPictureCycle();
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

// updateCuttingCycle: Start immediately, then execute forward, neutral hold,
// reverse, and neutral without blocking STOP.
void updateCuttingCycle() {
  unsigned long now = millis();
  if (runState == CUT_WAIT_SENSOR) {
    startCuttingMotion();
    return;
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

// printSensors: Show four roll IRs and both cycle E18 inputs.
void printSensors() {
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(ROLL_SENSORS[i].name);
    Serial.print(F(": "));
    Serial.println(isActive(ROLL_SENSORS[i].pin) ? F("TRIGGERED") : F("clear"));
  }
  Serial.print(F("picture_cycle_e18: "));
  Serial.println(isActive(PICTURE_CYCLE_E18_PIN) ? F("TRIGGERED") : F("clear"));
  Serial.print(F("cutting_cycle_e18: "));
  Serial.println(isActive(CUTTING_CYCLE_E18_PIN) ? F("TRIGGERED") : F("clear"));
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
  Serial.println(F("Button: start run; after each automatic rotation, press for a 5 s photo countdown."));
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

// updateState: Advance only the current stage without blocking STOP or LED.
void updateState() {
  switch (runState) {
    case START_LED_CUE:
      if (millis() - stateStartedMs >= START_LED_CUE_MS) startRollToStation();
      break;
    case ROLL_TO_SENSOR: updateRollToStation(); break;
    case ROLL_TO_ROTATION_DELAY:
      if (millis() - stateStartedMs >= ROLL_TO_ROTATION_DELAY_MS)
        startPictureRotation();
      break;
    case PHOTO_WAIT_CLEAR:
    case PHOTO_ROTATING: updatePictureRotation(); break;
    case PHOTO_COUNTDOWN:
    case PHOTO_SERVO_HOLD: updatePictureCycle(); break;
    case POST_PHOTO_PAUSE:
      if (millis() - stateStartedMs >=
          (stationIndex >= 2 ? POST_PHOTO_PAUSE_AFTER_IR_3_OR_4_MS :
                               POST_PHOTO_PAUSE_MS))
        advanceAfterPhotoPause();
      break;
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
    pinMode(ROLL_SENSORS[i].pin, INPUT);
  }
  pinMode(PICTURE_CYCLE_E18_PIN, INPUT);
  pinMode(CUTTING_CYCLE_E18_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_LED_PIN, OUTPUT);
  buttonLastRaw = digitalRead(BUTTON_PIN);
  buttonStable = buttonLastRaw;
  positionServo.attach(POSITION_SERVO_PIN);
  positionServo.write(POSITION_INITIAL_ANGLE);

  cuttingServo.attach(CUTTING_SERVO_PIN);
  cuttingServo.writeMicroseconds(CUT_STOP_US);
  setButtonLed(true);
  Serial.println(F("Module 1 v3 ready. Press button to start."));
  printHelp();
}

// loop: Prioritize serial STOP, process a debounced button press, advance
// motion, and update the ButtonLED.
void loop() {
  stopCommandThisLoop = false;
  pollSerial();
  bool pressed = buttonPressed();
  if (!stopCommandThisLoop && pressed) {
    if (runState == READY) startFullAutomation();
    else if (runState == WAIT_PHOTO_BUTTON) startPictureCycle();
  }
  if (!stopCommandThisLoop) updateState();
  updateButtonLed();
}
