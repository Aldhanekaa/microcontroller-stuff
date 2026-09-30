/*
  Module 1 semi-automatic controller - Arduino Mega 2560

  Four DC motors use two L298N boards without PWM. Leave all four ENA/ENB
  jumpers installed. Board 1: roll_1 uses IN1/IN2, roll_2 uses IN3/IN4.
  Board 2: roll_3 uses IN1/IN2, encoder_motor uses IN3/IN4.
  Five E18-D80NK sensors use INPUT_PULLUP and are assumed active LOW.
  Verify the output polarity and supply voltage of your exact sensors.
  The fourth motor is encoder_motor; its encoder A/B signals use Mega D2/D3.
  Power the encoder at its specified voltage and keep A/B at safe Mega levels.
  Button connects between BUTTON_PIN and GND; it is read by STATUS only.
  Two servos are command controlled only; automation does not move them.
  Power motors, sensors, and servos from supplies suited to their ratings,
  with a common ground to the Mega. Do not power motors/servos from USB.

  Serial Monitor: 115200 baud, Newline or Both NL & CR.
  Commands: E, D, AUTO, AUTOMATE-FULL, STOP, STATUS, SENSORS, ENCODER,
            ENCODER-RESET, HELP,
            MOTOR-1-ON/OFF through MOTOR-4-ON/OFF, ALL-ON, ALL-OFF,
            SERVO-1-1500, SERVO-2-1500 (pulse width in microseconds).
  Commands are case insensitive. E enables motion. STOP halts automation
  and all four motors, and sends neutral pulses to previously used servos.

  Important adjustable parameters are grouped immediately below. All
  functions describe their purpose next to their definition.
*/

#include <Arduino.h>
#include <Servo.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

struct MotorPins {
  const char *name;
  uint8_t input1;
  uint8_t input2;
};

struct SensorPin {
  const char *name;
  uint8_t pin;
};

enum MotorDirection { MOTOR_OFF, MOTOR_FORWARD, MOTOR_REVERSE };
enum AutoStage {
  AUTO_IDLE,
  AUTO_TO_IR_1,
  AUTO_TO_IR_2,
  AUTO_TO_IR_3,
  AUTO_TO_ROLL_4,
  AUTO_FINAL_RUN,
  AUTO_FINAL_ROLL_1
};

// stageName: Return a readable label for an automation stage.
const char *stageName(AutoStage stage) {
  switch (stage) {
    case AUTO_TO_IR_1: return "moving to ir_1";
    case AUTO_TO_IR_2: return "moving to ir_2";
    case AUTO_TO_IR_3: return "moving to ir_3";
    case AUTO_TO_ROLL_4: return "moving to roll_4";
    case AUTO_FINAL_RUN: return "roll_2 and roll_3 timed run";
    case AUTO_FINAL_ROLL_1: return "roll_1 timed run";
    default: return "idle";
  }
}

// Motor 1 and 2 reuse the first L298N pinout from the no-PWM test sketch.
// Motor 4 has the encoder, and is manual-control only in this version.
const MotorPins MOTORS[4] = {
  {"roll_1", 7, 8}, {"roll_2", 9, 10},
  {"roll_3", 24, 25}, {"encoder_motor", 26, 27}
};
const SensorPin SENSORS[5] = {
  {"ir_1", 30}, {"ir_2", 31}, {"ir_3", 32},
  {"roll_4", 33}, {"ir_rotation_trigger", 34}
};
const uint8_t BUTTON_PIN = 35;
const uint8_t SERVO_PINS[2] = {40, 41};
const uint8_t ENCODER_A_PIN = 2;
const uint8_t ENCODER_B_PIN = 3;
const uint8_t SENSOR_ACTIVE_LEVEL = LOW;
const uint8_t BUTTON_PRESSED_LEVEL = LOW;

// Reverse any of these constants if that motor is wired the other way.
const MotorDirection ROLL_1_DIRECTION = MOTOR_FORWARD;
const MotorDirection ROLL_2_FIRST_PAIR_DIRECTION = MOTOR_FORWARD;
const MotorDirection ROLL_2_SECOND_PAIR_DIRECTION = MOTOR_FORWARD;
const MotorDirection ROLL_3_DIRECTION = MOTOR_FORWARD;
const MotorDirection ENCODER_MOTOR_MANUAL_DIRECTION = MOTOR_FORWARD;

const unsigned long SENSOR_DEBOUNCE_MS = 30UL;
const unsigned long SENSOR_WAIT_TIMEOUT_MS = 60000UL;  // Set 0 to disable.
const unsigned long FINAL_ROLL_2_3_RUN_MS = 20000UL;
const unsigned long FINAL_ROLL_1_RUN_MS = 2000UL;
const int SERVO_NEUTRAL_US = 1500;
const int SERVO_MIN_US = 1200;
const int SERVO_MAX_US = 1800;

Servo servos[2];
bool servoAttached[2] = {false, false};
int servoPulseUs[2] = {SERVO_NEUTRAL_US, SERVO_NEUTRAL_US};
MotorDirection motorState[4] = {MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF};
bool isEnabled = false;
AutoStage autoStage = AUTO_IDLE;
unsigned long stageStartedMs = 0;
unsigned long sensorTriggeredMs = 0;
bool sensorTiming = false;
char commandBuffer[48];
uint8_t commandLength = 0;
bool discardingCommand = false;
volatile long encoderTicks = 0;
volatile unsigned long encoderInvalidTransitions = 0;
volatile uint8_t previousEncoderState = 0;

// updateEncoder: Count valid quadrature A/B edges for the fourth motor.
// This interrupt function runs when either encoder signal changes.
void updateEncoder() {
  uint8_t state = (digitalRead(ENCODER_A_PIN) << 1) |
                  digitalRead(ENCODER_B_PIN);
  uint8_t transition = (previousEncoderState << 2) | state;
  switch (transition) {
    case 0x1: case 0x7: case 0xE: case 0x8: ++encoderTicks; break;
    case 0x2: case 0xB: case 0xD: case 0x4: --encoderTicks; break;
    default:
      if (state != previousEncoderState) ++encoderInvalidTransitions;
      break;
  }
  previousEncoderState = state;
}

// readEncoder: Atomically copy the tick and invalid-transition counters.
void readEncoder(long &ticks, unsigned long &invalid) {
  noInterrupts();
  ticks = encoderTicks;
  invalid = encoderInvalidTransitions;
  interrupts();
}

// printEncoder: Show raw encoder ticks and invalid transitions on Serial.
void printEncoder() {
  long ticks;
  unsigned long invalid;
  readEncoder(ticks, invalid);
  Serial.print(F("encoder_motor ticks: "));
  Serial.print(ticks);
  Serial.print(F(" | invalid transitions: "));
  Serial.println(invalid);
}

// setMotor: Set one motor to OFF, FORWARD, or REVERSE using digital pins only.
// index is 0..3. OFF sets both L298N inputs LOW, braking the motor while EN is on.
void setMotor(uint8_t index, MotorDirection direction) {
  const MotorPins &motor = MOTORS[index];
  digitalWrite(motor.input1, LOW);
  digitalWrite(motor.input2, LOW);
  if (direction == MOTOR_FORWARD) digitalWrite(motor.input1, HIGH);
  if (direction == MOTOR_REVERSE) digitalWrite(motor.input2, HIGH);
  motorState[index] = direction;
}

// stopAllMotors: Brake all four motors, including the manual-only fourth motor.
void stopAllMotors() {
  for (uint8_t i = 0; i < 4; ++i) setMotor(i, MOTOR_OFF);
}

// neutralizeServos: Stop a continuous servo or center a position servo if used.
// Exact neutral behavior depends on the servo model and may need calibration.
void neutralizeServos() {
  for (uint8_t i = 0; i < 2; ++i) {
    if (servoAttached[i]) {
      servos[i].writeMicroseconds(SERVO_NEUTRAL_US);
      servoPulseUs[i] = SERVO_NEUTRAL_US;
    }
  }
}

// stopSystem: Cancel any automatic stage and stop all motors immediately.
// Servos that were commanded manually receive the configured neutral pulse.
void stopSystem() {
  autoStage = AUTO_IDLE;
  sensorTiming = false;
  stopAllMotors();
  neutralizeServos();
  Serial.println(F("STOP: automation cancelled; motors stopped."));
}

// sensorActive: Read one active-LOW sensor; index is 0..4.
bool sensorActive(uint8_t index) {
  return digitalRead(SENSORS[index].pin) == SENSOR_ACTIVE_LEVEL;
}

// TAKE_PICTURE_CYCLE: Placeholder called at each of the four photo positions.
void TAKE_PICTURE_CYCLE() {
  // TODO: add the tested picture-taking actions here.
  // Keep future actions nonblocking so Serial STOP remains responsive.
}

// MODULE_1_STOP: Placeholder called after the 20-second roll_2/roll_3 run.
void MODULE_1_STOP() {
  // TODO: add the later module handoff/stop actions here.
  // Keep future actions nonblocking so Serial STOP remains responsive.
}

// stageSensorIndex: Return the target sensor for a sensor-driven stage.
// -1 means the current stage is timed or idle.
int8_t stageSensorIndex(AutoStage stage) {
  switch (stage) {
    case AUTO_TO_IR_1: return 0;
    case AUTO_TO_IR_2: return 1;
    case AUTO_TO_IR_3: return 2;
    case AUTO_TO_ROLL_4: return 3;
    default: return -1;
  }
}

// enterStage: Start the selected motor pair or timed final run.
// Target sensors must be clear when their stage starts; otherwise stop safely.
void enterStage(AutoStage next) {
  stopAllMotors();
  autoStage = next;
  stageStartedMs = millis();
  sensorTiming = false;

  int8_t target = stageSensorIndex(next);
  if (target >= 0 && sensorActive((uint8_t)target)) {
    Serial.print(F("Target sensor already active: "));
    Serial.println(SENSORS[target].name);
    stopSystem();
    return;
  }

  switch (next) {
    case AUTO_TO_IR_1:
    case AUTO_TO_IR_2:
      setMotor(0, ROLL_1_DIRECTION);
      setMotor(1, ROLL_2_FIRST_PAIR_DIRECTION);
      break;
    case AUTO_TO_IR_3:
    case AUTO_TO_ROLL_4:
    case AUTO_FINAL_RUN:
      setMotor(1, ROLL_2_SECOND_PAIR_DIRECTION);
      setMotor(2, ROLL_3_DIRECTION);
      break;
    case AUTO_FINAL_ROLL_1:
      setMotor(0, ROLL_1_DIRECTION);
      break;
    case AUTO_IDLE:
      break;
  }
  if (next != AUTO_IDLE) {
    Serial.print(F("Automatic stage: "));
    Serial.println(stageName(next));
  }
}

// startAutomation: Begin the four sensor stops and two timed finishing moves.
// E must have enabled motion. All four target sensors must start untriggered.
void startAutomation() {
  if (!isEnabled) {
    Serial.println(F("Send E before AUTO."));
    return;
  }
  if (autoStage != AUTO_IDLE) {
    Serial.println(F("Automation is already running."));
    return;
  }
  // An AUTO request also ends any manual motor run before sensor checks.
  stopAllMotors();
  for (uint8_t i = 0; i < 4; ++i) {
    if (sensorActive(i)) {
      Serial.print(F("Clear sensor before AUTO: "));
      Serial.println(SENSORS[i].name);
      return;
    }
  }
  Serial.println(F("Automation started."));
  enterStage(AUTO_TO_IR_1);
}

// finishSensorStage: Stop the pair, call the empty picture function once,
// and begin the next sensor or timed stage.
void finishSensorStage() {
  AutoStage completed = autoStage;
  stopAllMotors();
  Serial.print(F("Reached "));
  Serial.println(SENSORS[stageSensorIndex(completed)].name);
  TAKE_PICTURE_CYCLE();
  if (completed == AUTO_TO_IR_1) enterStage(AUTO_TO_IR_2);
  else if (completed == AUTO_TO_IR_2) enterStage(AUTO_TO_IR_3);
  else if (completed == AUTO_TO_IR_3) enterStage(AUTO_TO_ROLL_4);
  else enterStage(AUTO_FINAL_RUN);
}

// updateAutomation: Advance the automatic sequence without blocking Serial.
// Sensor stages require a stable trigger and have a configurable timeout.
void updateAutomation() {
  if (autoStage == AUTO_IDLE) return;
  unsigned long now = millis();
  int8_t target = stageSensorIndex(autoStage);
  if (target >= 0) {
    if (sensorActive((uint8_t)target)) {
      if (!sensorTiming) {
        sensorTiming = true;
        sensorTriggeredMs = now;
      } else if (now - sensorTriggeredMs >= SENSOR_DEBOUNCE_MS) {
        finishSensorStage();
        return;
      }
    } else {
      sensorTiming = false;
    }
    if (SENSOR_WAIT_TIMEOUT_MS > 0 &&
        now - stageStartedMs >= SENSOR_WAIT_TIMEOUT_MS) {
      Serial.println(F("Sensor timeout; stopping automation."));
      stopSystem();
    }
    return;
  }

  if (autoStage == AUTO_FINAL_RUN &&
      now - stageStartedMs >= FINAL_ROLL_2_3_RUN_MS) {
    stopAllMotors();
    MODULE_1_STOP();
    enterStage(AUTO_FINAL_ROLL_1);
  } else if (autoStage == AUTO_FINAL_ROLL_1 &&
             now - stageStartedMs >= FINAL_ROLL_1_RUN_MS) {
    stopAllMotors();
    autoStage = AUTO_IDLE;
    Serial.println(F("Module 1 automation complete."));
  }
}

// printSensors: Print the trigger state of all five named IR sensors and button.
void printSensors() {
  for (uint8_t i = 0; i < 5; ++i) {
    Serial.print(SENSORS[i].name);
    Serial.print(F(": "));
    Serial.println(sensorActive(i) ? F("TRIGGERED") : F("clear"));
  }
  Serial.print(F("button: "));
  Serial.println(digitalRead(BUTTON_PIN) == BUTTON_PRESSED_LEVEL ?
                 F("PRESSED") : F("released"));
}

// printStatus: Report enable/automation state, each motor, and sensor inputs.
void printStatus() {
  Serial.print(F("Enabled: "));
  Serial.println(isEnabled ? F("YES") : F("NO"));
  Serial.print(F("Automation stage: "));
  Serial.println(stageName(autoStage));
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(MOTORS[i].name);
    Serial.print(F(": "));
    Serial.println(motorState[i] == MOTOR_OFF ? F("OFF") :
                   motorState[i] == MOTOR_FORWARD ? F("FORWARD") : F("REVERSE"));
  }
  printSensors();
  printEncoder();
  for (uint8_t i = 0; i < 2; ++i) {
    Serial.print(F("servo_"));
    Serial.print(i + 1);
    Serial.print(F(": "));
    if (servoAttached[i]) Serial.println(servoPulseUs[i]);
    else Serial.println(F("not commanded"));
  }
}

// printHelp: List all supported serial commands and their arguments.
void printHelp() {
  Serial.println(F("E | D | AUTO | AUTOMATE-FULL | STOP | STATUS | SENSORS"));
  Serial.println(F("ENCODER | ENCODER-RESET | HELP"));
  Serial.println(F("MOTOR-1-ON/OFF ... MOTOR-4-ON/OFF | ALL-ON | ALL-OFF"));
  Serial.println(F("SERVO-1-1500 | SERVO-2-1500 (pulse us; configured range 1200..1800)"));
}

// processCommand: Parse one complete, case-insensitive command line.
// Manual motor/servo commands are rejected while automation is running.
void processCommand(char *command) {
  if (strcmp(command, "STOP") == 0 || strcmp(command, "ALL-OFF") == 0) {
    stopSystem();
    return;
  }
  if (strcmp(command, "D") == 0) {
    stopSystem();
    isEnabled = false;
    Serial.println(F("System disabled."));
    return;
  }
  if (strcmp(command, "E") == 0) {
    isEnabled = true;
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
  if (strcmp(command, "ENCODER") == 0) {
    printEncoder();
    return;
  }
  if (strcmp(command, "ENCODER-RESET") == 0) {
    if (motorState[3] != MOTOR_OFF) {
      Serial.println(F("Stop motor 4 before resetting encoder counts."));
      return;
    }
    noInterrupts();
    encoderTicks = 0;
    encoderInvalidTransitions = 0;
    interrupts();
    Serial.println(F("Encoder counts reset."));
    return;
  }
  if (strcmp(command, "HELP") == 0) {
    printHelp();
    return;
  }
  if (strcmp(command, "AUTO") == 0 || strcmp(command, "AUTOMATE-FULL") == 0) {
    startAutomation();
    return;
  }
  if (!isEnabled) {
    Serial.println(F("System disabled. Send E first."));
    return;
  }
  if (autoStage != AUTO_IDLE) {
    Serial.println(F("Automation active. Use STOP before manual commands."));
    return;
  }
  if (strcmp(command, "ALL-ON") == 0) {
    setMotor(0, ROLL_1_DIRECTION);
    setMotor(1, ROLL_2_FIRST_PAIR_DIRECTION);
    setMotor(2, ROLL_3_DIRECTION);
    setMotor(3, ENCODER_MOTOR_MANUAL_DIRECTION);
    Serial.println(F("All four motors ON."));
    return;
  }
  if (strncmp(command, "MOTOR-", 6) == 0 &&
      command[6] >= '1' && command[6] <= '4' && command[7] == '-') {
    uint8_t index = command[6] - '1';
    MotorDirection onDirection[4] = {
      ROLL_1_DIRECTION, ROLL_2_FIRST_PAIR_DIRECTION,
      ROLL_3_DIRECTION, ENCODER_MOTOR_MANUAL_DIRECTION
    };
    if (strcmp(command + 8, "ON") == 0) {
      setMotor(index, onDirection[index]);
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
    long pulse = strtol(command + 8, &end, 10);
    if (end != command + 8 && *end == '\0' &&
        pulse >= SERVO_MIN_US && pulse <= SERVO_MAX_US) {
      uint8_t index = command[6] - '1';
      if (!servoAttached[index]) {
        servos[index].attach(SERVO_PINS[index]);
        servoAttached[index] = true;
      }
      servos[index].writeMicroseconds((int)pulse);
      servoPulseUs[index] = (int)pulse;
      Serial.print(F("servo_"));
      Serial.print(index + 1);
      Serial.print(F(" pulse (us): "));
      Serial.println(pulse);
      return;
    }
    Serial.println(F("Invalid servo pulse. Use 1200..1800 us."));
    return;
  }
  Serial.println(F("Unknown command. Send HELP."));
}

// pollSerial: Collect newline-terminated commands one character at a time.
// This avoids the blocking readStringUntil() used by the reference module.
void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
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

// setup: Initialize all pins in a stopped state and print command help.
void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(MOTORS[i].input1, OUTPUT);
    pinMode(MOTORS[i].input2, OUTPUT);
    setMotor(i, MOTOR_OFF);
  }
  for (uint8_t i = 0; i < 5; ++i) pinMode(SENSORS[i].pin, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  previousEncoderState = (digitalRead(ENCODER_A_PIN) << 1) |
                         digitalRead(ENCODER_B_PIN);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), updateEncoder, CHANGE);
  Serial.println(F("Module 1 semi-auto controller ready."));
  printHelp();
}

// loop: Service commands first, then advance one automatic stage if active.
void loop() {
  pollSerial();
  updateAutomation();
}
