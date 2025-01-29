// ------------------------------------------------------
// Minimal code to measure frequency on pin 8 and print
// ------------------------------------------------------
#include <Arduino.h>

// Pin Definition
#define PWM_INPUT_PIN   8   // PB0 (Digital Pin 8)

// Global Variables
volatile float measuredFrequency = 0.0;
volatile bool  frequencyUpdated  = false;
volatile unsigned long lastPulseTime = 0;  
volatile bool lastState = LOW;             

void setup() {
  Serial.begin(9600);
  
  // Configure pin as input
  pinMode(PWM_INPUT_PIN, INPUT);
  
  // ----------------------------------------------------
  // Enable Pin Change Interrupt for PB0 => Digital Pin 8
  // ----------------------------------------------------
  PCICR  |= (1 << PCIE0);    // Enable Pin Change Interrupt for PCINT[7:0]
  PCMSK0 |= (1 << PCINT0);   // Enable interrupt for PCINT0 (PB0)
}

void loop() {
  // Whenever we detect a new frequency, print it.
  if (frequencyUpdated) {
    noInterrupts();
    float freq = measuredFrequency; 
    frequencyUpdated = false;
    interrupts();
    
    Serial.print("Measured Frequency: ");
    Serial.print(freq);
    Serial.println(" Hz");
  }
}

// ------------------------------------------------------
// Pin Change Interrupt ISR for PB0 (Digital Pin 8)
// ------------------------------------------------------
ISR(PCINT0_vect) {
  bool currentState = (PINB & (1 << PB0)) != 0; // Read PB0 directly
  unsigned long now = micros();

  // Detect rising edge
  if (currentState && !lastState) {
    unsigned long elapsed = now - lastPulseTime;
    lastPulseTime = now;
    
    if (elapsed > 0) {
      // Convert elapsed time between pulses into frequency
      measuredFrequency = 1e6 / (float)elapsed;  // freq = 1 / period
      frequencyUpdated  = true;
    }
  }
  lastState = currentState;
}
