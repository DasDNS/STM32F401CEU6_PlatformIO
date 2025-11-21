#include <Arduino.h>

#define NUM_SENSORS 10

// Your pin mapping
uint8_t fsrPins[NUM_SENSORS] = {
  PB1, PB0,
  PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0
};

const int samples = 200;      // total samples for mean
const int delay_ms = 3;       // time between samples

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configure STM32 pins as analog inputs
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(fsrPins[i], INPUT_ANALOG);
  }

  Serial.println("FSR Sensor System Ready...");
}

void loop() {

  float fsrMean[NUM_SENSORS] = {0};

  // Take multiple samples for smoothing
  for (int sampleCount = 0; sampleCount < samples; sampleCount++) {
    for (int s = 0; s < NUM_SENSORS; s++) {
      fsrMean[s] += analogRead(fsrPins[s]);
    }
    delay(delay_ms);
  }

  // Compute the average
  for (int s = 0; s < NUM_SENSORS; s++) {
    fsrMean[s] /= samples;
  }

  // Print in one line (best for logging)
  Serial.print("FSR readings: ");
  for (int s = 0; s < NUM_SENSORS; s++) {
    Serial.print(fsrMean[s], 2);   // 2 decimal places
    if (s < NUM_SENSORS - 1) Serial.print(", ");
  }
  Serial.println();

  delay(1000);  // wait 1 second before next batch
}
