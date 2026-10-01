#include <Arduino.h>
#include <Servo.h>

const uint8_t SERVO_PIN = 31;
const uint8_t E18_SENSOR_PIN = 43;
const int STOP_US = 1500;
const int FORWARD_US = 1600;
const int REVERSE_US = 1400;
const unsigned long INITIAL_FORWARD_MS = 1800;
const unsigned long INITIAL_REVERSE_MS = 1700;
const unsigned long FORWARD_DURATION_INCREMENT_MS = 25;
const unsigned long BACKWARD_DURATION_INCREMENT_MS = 100;

const unsigned long HOLD_BEFORE_REVERSE_MS = 250;

Servo cuttingServo;

enum CycleStage {
	IDLE,
	MOVING_FORWARD,
	HOLDING,
	MOVING_REVERSE
};

CycleStage cycleStage = IDLE;
unsigned long stageStartedAt = 0;
unsigned long forwardDurationMs = INITIAL_FORWARD_MS;
unsigned long reverseDurationMs = INITIAL_REVERSE_MS;
bool sensorWasClear = true;

void stopServo() {
	cuttingServo.writeMicroseconds(STOP_US);
}

void startCycle() {
	cuttingServo.writeMicroseconds(FORWARD_US);
	stageStartedAt = millis();
	cycleStage = MOVING_FORWARD;

	Serial.print(F("Cycle started: forward "));
	Serial.print(forwardDurationMs);
	Serial.print(F(" ms, reverse "));
	Serial.print(reverseDurationMs);
	Serial.println(F(" ms"));
}

void updateCycle() {
	unsigned long elapsedMs = millis() - stageStartedAt;

	if (cycleStage == MOVING_FORWARD && elapsedMs >= forwardDurationMs) {
		stopServo();
		stageStartedAt = millis();
		cycleStage = HOLDING;
		Serial.println(F("Holding before reverse"));
	} else if (cycleStage == HOLDING && elapsedMs >= HOLD_BEFORE_REVERSE_MS) {
		cuttingServo.writeMicroseconds(REVERSE_US);
		stageStartedAt = millis();
		cycleStage = MOVING_REVERSE;
	} else if (cycleStage == MOVING_REVERSE && elapsedMs >= reverseDurationMs) {
		stopServo();
		cycleStage = IDLE;
		forwardDurationMs += FORWARD_DURATION_INCREMENT_MS;
		reverseDurationMs += BACKWARD_DURATION_INCREMENT_MS;
		Serial.println(F("Cycle complete; waiting for sensor to clear"));
	}
}

void checkSensor() {
	bool sensorClear = digitalRead(E18_SENSOR_PIN) == HIGH;

	if (sensorClear) {
		sensorWasClear = true;
	} else if (sensorWasClear && cycleStage == IDLE) {
		sensorWasClear = false;
		startCycle();
	}
}

void setup() {
	Serial.begin(115200);
	pinMode(E18_SENSOR_PIN, INPUT);
	cuttingServo.attach(SERVO_PIN);
	stopServo();
	Serial.println(F("Ready: waiting for object"));
}

void loop() {
	checkSensor();
	updateCycle();
}
