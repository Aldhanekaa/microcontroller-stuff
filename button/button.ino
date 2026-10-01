const int buttonPin = 41; // Button connected to D41 and GND
const int buttonLED = 39;

// --- Debounce Variables ---
int buttonState = HIGH;      
int lastButtonState = HIGH;  
unsigned long lastDebounceTime = 0;  
unsigned long debounceDelay = 50;    


void setup() {
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT_PULLUP); 
  Serial.begin(115200);
  Serial.println("Starting");

}

void loop() {
  // put your main code here, to run repeatedly:
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


  if (buttonPressed) Serial.println("Pressed");
  // Serial.println(reading);

  digitalWrite(buttonLED, HIGH);
  // Serial.print("Status");
  // Serial.println(buttonPressed)



}
