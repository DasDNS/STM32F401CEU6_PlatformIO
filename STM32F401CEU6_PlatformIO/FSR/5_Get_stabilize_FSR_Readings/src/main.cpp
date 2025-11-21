#include <Arduino.h>

#define NUM_SENSORS 10

uint8_t fsrPins[NUM_SENSORS] = {
  PB1, PB0,
  PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0
};

const int samples = 80;
const int delay_ms = 3;
const float threshold = 15.0;
const int stableCyclesNeeded = 5;

float prevMean[NUM_SENSORS] = {0};
float savedData[NUM_SENSORS] = {0};

int stableCounter = 0;
bool hasSaved = false;   // prevents repeated saving

void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(fsrPins[i], INPUT_ANALOG);
  }

  Serial.println("FSR Auto-Save System Ready...");
}

void loop() {

  float meanVal[NUM_SENSORS] = {0};

  // ---- Take samples ----
  for (int i = 0; i < samples; i++) {
    for (int s = 0; s < NUM_SENSORS; s++) {
      meanVal[s] += analogRead(fsrPins[s]);
    }
    delay(delay_ms);
  }

  // ---- Compute mean ----
  for (int s = 0; s < NUM_SENSORS; s++) {
    meanVal[s] /= samples;
  }

  // ---- Stability detection ----
  bool allStable = true;

  for (int s = 0; s < NUM_SENSORS; s++) {
    float delta = fabs(meanVal[s] - prevMean[s]);
    if (delta > threshold) {
      allStable = false;
    }
  }

  if (allStable) {
    stableCounter++;
  } else {
    stableCounter = 0;
    hasSaved = false;   // reset so next stable event saves again
  }

  // ---- AUTO SAVE ----
  if (stableCounter >= stableCyclesNeeded && !hasSaved) {
    for (int s = 0; s < NUM_SENSORS; s++) {
      savedData[s] = meanVal[s];
    }

    Serial.println("=====================================");
    Serial.println("STABLE → DATA SAVED");
    Serial.print("FSR Snapshot: ");

    for (int s = 0; s < NUM_SENSORS; s++) {
      Serial.print(savedData[s], 2);
      if (s < NUM_SENSORS - 1) Serial.print(", ");
    }
    Serial.println();
    Serial.println("=====================================");

    hasSaved = true;
  }

  // ---- Print live readings ----
  Serial.print("FSR Live: ");
  for (int s = 0; s < NUM_SENSORS; s++) {
    Serial.print(meanVal[s], 2);
    if (s < NUM_SENSORS - 1) Serial.print(", ");
  }
  Serial.println();

  // ---- Update last mean ----
  for (int s = 0; s < NUM_SENSORS; s++) {
    prevMean[s] = meanVal[s];
  }

  delay(200);
}
