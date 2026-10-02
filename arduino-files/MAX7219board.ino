#include "LedControl.h"

// Pin connections for Arduino Mega:
// DIN = 12, CLK = 11, CS (LOAD) = 10
// The '1' means we are only using one MAX7219 module
LedControl lc = LedControl(33, 37, 35, 1);

void setup() {
  // STEP 1: The USB Handshake Delay
  // Force the Mega to wait 2 seconds before touching the display.
  // This gives the M2 Air's SMC time to finish USB enumeration and lock in the 500mA limit.
  delay(2000);

  // STEP 2: Wake up the display
  lc.shutdown(0, false);

  // STEP 3: The Power Choke
  // Immediately set brightness to absolute minimum (0).
  // Do NOT increase this yet. This prevents the display from pulling a massive current spike.
  lc.setIntensity(0, 0);

  // STEP 4: Clear the Registers
  // Wipe any garbage data or default "all segments on" states 
  // that might draw excess power.
  lc.clearDisplay(0);
  
  // Serial monitor for debugging the 3-second crash window
  Serial.begin(9600);
  Serial.println("Boot successful. M2 power stabilized.");
}

void loop() {
  // A safe, low-power test: counting up slowly on just three digits.
  // This proves the display works without turning on all 64 LEDs at once.
  for (int count = 0; count < 1000; count++) {
    
    // Break the number down into individual digits
    lc.setDigit(0, 0, count % 10, false);          // 1s column
    lc.setDigit(0, 1, (count / 10) % 10, false);   // 10s column
    lc.setDigit(0, 2, (count / 100) % 10, false);  // 100s column
    
    // Print a heartbeat to the Serial Monitor
    Serial.print("Uptime heartbeat... ");
    Serial.println(count);
    
    delay(250); 
  }
}