// Module 1 v2 wiring.
int motorFeeder1pin1 = 13;  // roll_1 IN1
int motorFeeder1pin2 = 15;  // roll_1 IN2
int motorFeeder2pin1 = 9;   // roll_2 IN3
int motorFeeder2pin2 = 11;  // roll_2 IN4
int motorFeeder3pin1 = 19;  // roll_3 IN1
int motorFeeder3pin2 = 17;  // roll_3 IN2
int motorCrankpin1 = 5;     // motor_4 IN3
int motorCrankpin2 = 7;     // motor_4 IN4

const int servoPin = 40;          // Position servo
const int cuttingServoPin = 31;   // Continuous cutting servo
const int irSensor1 = 27;         // ir_1
const int irSensor2 = 23;         // ir_2
const int irSensor3 = 43;         // ir_3
const int irSensor4 = 25;         // ir_4
const int pictureCycleE18 = 29;   // Picture-cycle E18 sensor
const int cuttingCycleE18 = 45;   // Cutting-cycle E18 sensor

const int buttonPin = 41; // Button connected to D41 and GND
const int ledPin = 39;    // LED connected to D39

// --- State Machine Variables ---
int currentStep = 0;             // Tracks the active step in the sequence
unsigned long stepTimer = 0;     // For handling timed events (Steps 6 & 7)
bool timerStarted = false;       // Checks if the timer has been initialized for a step

// --- Debounce Variables ---
int buttonState = HIGH;      
int lastButtonState = HIGH;  
unsigned long lastDebounceTime = 0;  
unsigned long debounceDelay = 50;    

void setup() {
  Serial.begin(9600);
  
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); 
  
  // Roll IR sensors are active-LOW, matching module_1_semi_auto_v2.
  pinMode(irSensor1, INPUT_PULLUP);
  pinMode(irSensor2, INPUT_PULLUP);
  pinMode(irSensor3, INPUT_PULLUP);
  pinMode(irSensor4, INPUT_PULLUP);
  pinMode(pictureCycleE18, INPUT);
  pinMode(cuttingCycleE18, INPUT);

  // Configure Motor Pins
  pinMode(motorFeeder1pin1, OUTPUT);
  pinMode(motorFeeder1pin2, OUTPUT);
  pinMode(motorFeeder2pin1, OUTPUT);
  pinMode(motorFeeder2pin2, OUTPUT);
  pinMode(motorFeeder3pin1, OUTPUT);
  pinMode(motorFeeder3pin2, OUTPUT);
  pinMode(motorCrankpin1,   OUTPUT);
  pinMode(motorCrankpin2,   OUTPUT);
  
  stopAllMotors();
  digitalWrite(ledPin, LOW);
  Serial.println("Module 1 v2 pin map loaded:");
  Serial.println("roll_1 D13/D15, roll_2 D9/D11, roll_3 D19/D17, motor_4 D5/D7");
  Serial.println("ir_1 D27, ir_2 D23, ir_3 D43, ir_4 D25");
  Serial.println("picture E18 D29, cutting E18 D45, position servo D40, cutting servo D31");
  Serial.println("System Ready. Press button to Start (Step 1).");
}

void loop() {
  // Read and debounce the button
  int reading = digitalRead(buttonPin);
  bool buttonPressed = false;

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      // Button transitions to pressed (LOW)
      if (buttonState == LOW) {
        buttonPressed = true; 
      }
    }
  }
  lastButtonState = reading;

  // --- STATE MACHINE SEQUENCE ---
  switch (currentStep) {
    
    case 0: // Step 1: Idle. Waiting for initial button press.
      if (buttonPressed) {
        Serial.println("Step 1 -> Step 2: Starting Motor 1 & 2");
        digitalWrite(ledPin, HIGH); // Turn solid LED on to indicate system active
        
        // Turn Motor 1 ON
        digitalWrite(motorFeeder1pin1, HIGH);
        digitalWrite(motorFeeder1pin2, LOW);
        // Turn Motor 2 ON
        digitalWrite(motorFeeder2pin1, HIGH);
        digitalWrite(motorFeeder2pin2, LOW);
        
        currentStep = 2; // Advance to waiting for IR 1
      }
      break;

    case 2: // Step 3: Wait for first IR sensor (D23) to trigger
      // Adjust to == LOW if your sensor triggers LOW, or == HIGH if it triggers HIGH
      if (digitalRead(irSensor1) == LOW) { 
        Serial.println("Step 3: IR Sensor 1 Triggered. Stopping Motors.");
        stopAllMotors();
        currentStep = 3; // Advance to waiting for button press for Step 4
      }
      break;

    case 3: // Step 4: Waiting for button press to trigger shutter + move to IR 2
      if (buttonPressed) {
        Serial.println("Step 4: Shutter action...");
        shutterLED(3, 150); // Flicker LED 3 times, 150ms per flash
        digitalWrite(ledPin, HIGH); // Keep LED back on
        
        // Turn Motor 1 and Motor 2 back ON
        digitalWrite(motorFeeder1pin1, HIGH);
        digitalWrite(motorFeeder1pin2, LOW);
        digitalWrite(motorFeeder2pin1, HIGH);
        digitalWrite(motorFeeder2pin2, LOW);
        
        currentStep = 4; // Advance to waiting for IR 2
      }
      break;

    case 4: // Step 4 (cont.): Wait for second IR sensor (D25) to trigger
      if (digitalRead(irSensor2) == LOW) {
        Serial.println("Step 4 Complete: IR Sensor 2 Triggered. Stopping Motors.");
        stopAllMotors();
        currentStep = 5; // Advance to waiting for button press for Step 5
      }
      break;

    case 5: // Step 5: Waiting for button press to trigger shutter + move to IR 3
      if (buttonPressed) {
        Serial.println("Step 5: Shutter action...");
        shutterLED(3, 150); 
        digitalWrite(ledPin, HIGH);
        
        // Turn Motor 2 and Motor 3 ON
        digitalWrite(motorFeeder2pin1, HIGH);
        digitalWrite(motorFeeder2pin2, LOW);
        digitalWrite(motorFeeder3pin1, HIGH);
        digitalWrite(motorFeeder3pin2, LOW);
        
        currentStep = 6; // Advance to waiting for IR 3
      }
      break;

    case 6: // Step 5 (cont.): Wait for third IR sensor (D27) to trigger
      if (digitalRead(irSensor3) == LOW) {
        Serial.println("Step 5 Complete: IR Sensor 3 Triggered. Stopping Motors.");
        stopAllMotors();
        
        // Setup for Step 6 (5-second wait timer)
        stepTimer = millis();
        Serial.println("Step 6: Waiting 5 seconds...");
        currentStep = 7; 
      }
      break;

    case 7: // Step 6: Wait 5 seconds
      if (millis() - stepTimer >= 5000) {
        Serial.println("Step 6 Complete. Step 7: Running Motor 2 & 3 for 10 seconds...");
        
        // Turn Motor 2 and Motor 3 ON
        digitalWrite(motorFeeder2pin1, HIGH);
        digitalWrite(motorFeeder2pin2, LOW);
        digitalWrite(motorFeeder3pin1, HIGH);
        digitalWrite(motorFeeder3pin2, LOW);
        
        stepTimer = millis(); // Reset timer for the 10-second duration
        currentStep = 8;
      }
      break;

    case 8: // Step 7: Run motors for 10 seconds, then stop everything
      if (millis() - stepTimer >= 10000) {
        Serial.println("Step 7 Complete. Sequence finished. Resetting to Idle.");
        stopAllMotors();
        digitalWrite(ledPin, LOW); // Turn off system status LED
        currentStep = 0;           // Loop back to the beginning
      }
      break;
  }
}

// Helper function to handle the 3-time flickering action
void shutterLED(int flashes, int duration) {
  for (int i = 0; i < flashes; i++) {
    digitalWrite(ledPin, LOW);
    delay(duration);
    digitalWrite(ledPin, HIGH);
    delay(duration);
  }
}

// Helper function to kill power to all motor channels instantly
void stopAllMotors() {
  digitalWrite(motorFeeder1pin1, LOW);
  digitalWrite(motorFeeder1pin2, LOW);
  digitalWrite(motorFeeder2pin1, LOW);
  digitalWrite(motorFeeder2pin2, LOW);
  digitalWrite(motorFeeder3pin1, LOW);
  digitalWrite(motorFeeder3pin2, LOW);
  digitalWrite(motorCrankpin1,   LOW);
  digitalWrite(motorCrankpin2,   LOW);
}