# FSR Array Readout — STM32 WeAct Black Pill (10 sensors)

**Board:** WeAct Black Pill V3.0 — STM32F401CEU6  
https://stm32-base.org/boards/STM32F401CEU6-WeAct-Black-Pill-V3.0.html

**Sensors:** 10 × Pololu FSR (FSR X 402 Short)  
Product page: https://www.pololu.com/product/2728

**Wiring summary:**  
- Each FSR is used in a simple voltage-divider with a 10 kΩ resistor to produce a measurable voltage at the ADC pin.  
- A 0.1 µF capacitor is placed from the ADC node to ground (forming a small RC low-pass filter with the 10 kΩ resistor).  
- MCU analog inputs used (as shown in the sketch): `PB1, PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0` (10 channels).  
- MCU operating voltage & ADC reference: **3.3 V** (STM32F4 ADC reference is VDD, typically 3.3 V).

**Circuit (text diagram)**

```
   3.3V (Vcc)
     |
     o---/\/\/\---+----> ADC_PIN (to STM32)
       10 kΩ     |
                 |----||----- GND
                 |     0.1 µF
                [FSR]
                 |
                GND
```

**Explanation:** the FSR and the 10 kΩ resistor form a voltage divider. When force is applied the FSR resistance drops, changing the voltage at the ADC node. The 0.1 µF capacitor forms a first-order low-pass filter with 10 kΩ: cutoff frequency f_c = 1/(2π R C) ≈ **159 Hz** (good for rejecting high‑frequency noise).

---

## The sketch (exact)
```cpp
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
```

---

## Thorough explanation (line-by-line & concepts)

### Top-level configuration
- `#define NUM_SENSORS 10` — number of FSRs connected.  
- `fsrPins[]` — array of MCU pins used for the ADC reads. The pins are STM32 port pins (PB1, PB0, PA7, ...). Make sure these pins are ADC capable on your specific MCU package — on the STM32F401CEU6 the listed PAx/PBx are typical ADC-capable pins but verify in your board datasheet / pinmap.

### Sampling parameters
- `samples = 200` & `delay_ms = 3` — the loop accumulates `samples` readings per sensor, with `delay_ms` milliseconds between each sample. This gives a smoothing window of roughly `samples × delay_ms` ≈ 600 ms of acquisition (plus loop overhead). Averaging reduces random noise and makes readings more stable at the cost of temporal responsiveness.

### `setup()`
- `Serial.begin(115200)` — start the serial port for debugging/log output.  
- `pinMode(..., INPUT_ANALOG)` — configure each pin as analog input (STM32 Arduino cores use `INPUT_ANALOG` to enable ADC sampling on that pin).

### `loop()` — reading process
1. `float fsrMean[NUM_SENSORS] = {0};` — temporary array to accumulate the sum of ADC readings for each sensor. Using `float` preserves fractional averages when dividing later.  
2. Outer loop `for sampleCount` collects `samples` readings.  
   - Inner loop `for s` iterates each FSR and calls `analogRead(fsrPins[s])`. The reading is added to `fsrMean[s]`.  
   - `delay(delay_ms)` gives the RC filter a little time to settle and spaces samples in time.  
3. After sampling, `fsrMean[s] /= samples;` converts the accumulated sum into an average reading.  
4. The code prints all averaged readings on one line separated by commas — convenient for logging or CSV-like ingestion.

### What `analogRead()` returns on STM32?
- On STM32 Arduino cores `analogRead()` typically returns a raw ADC integer value. The ADC resolution for STM32F4 is 12-bit by default, i.e. range **0..4095**. Some Arduino cores or frameworks may **normalize** the reading to `0.0..1.0` or use 10-bit `0..1023`. **You must confirm which behavior your Arduino core uses**.
- **Recommendation:** explicitly set and read the ADC resolution (if your core supports it) for predictable behavior. For example, some cores support `analogReadResolution(12)` and `analogReadAveraging(x)` calls. If not available, treat `analogRead()` as returning `[0..4095]` and test quickly by printing integer values for debugging.

### Converting ADC to voltage
If `analogRead()` returns a 12-bit value `N` (0..4095) and Vref = 3.3 V:
```
V = (N / 4095.0) * 3.3  // volts
```
Use that `V` to compute the FSR resistance (if you know the fixed resistor value) and then map to force after calibration.

For a simple divider where `R_fixed = 10k` and FSR is connected to ground:
```
Vout = 3.3 * ( R_FSR / (R_FSR + R_fixed) )
=> R_FSR = R_fixed * Vout / (Vcc - Vout)
```
Then convert `R_FSR` to force using a calibration (see below).

### Why averaging and the RC filter are used
- The 10 kΩ + 0.1 µF RC filter reduces high-frequency noise and helps stabilise the ADC; the cutoff is ~159 Hz which is well above typical human/robotic contact changes, but it removes spikes and EMI.  
- Averaging `samples` readings further reduces noise and ADC quantization ripple.

---

## Interpreting your provided serial output (full output included verbatim below)

> **Note:** you asked not to shorten the output — it is included below exactly as you provided it.

---- Opened the serial port /dev/ttyUSB0 ----
FSR readings: 0.24, 0.28, 0.27, 0.19, 0.25, 0.18, 0.14, 0.12, 0.08, 0.22
FSR readings: 0.23, 0.27, 0.21, 0.22, 0.22, 0.22, 0.12, 0.12, 0.11, 0.26
FSR readings: 0.22, 0.21, 0.27, 0.24, 0.24, 0.19, 0.10, 0.12, 0.09, 0.21
FSR readings: 0.23, 0.24, 0.25, 0.23, 0.22, 0.13, 0.12, 0.14, 0.11, 0.22
FSR readings: 0.32, 0.23, 0.23, 0.25, 0.19, 0.19, 0.13, 0.13, 0.13, 0.23
FSR readings: 0.25, 0.20, 0.25, 0.20, 0.24, 0.18, 0.15, 0.13, 0.15, 0.25
FSR readings: 0.16, 0.23, 0.22, 0.24, 0.23, 0.19, 0.11, 0.12, 0.07, 0.24
FSR readings: 0.23, 0.27, 0.27, 0.25, 0.18, 0.16, 0.15, 0.12, 0.09, 0.22
FSR readings: 0.21, 0.24, 0.19, 0.25, 0.21, 0.15, 0.11, 0.17, 0.12, 0.22
FSR readings: 0.22, 0.25, 0.27, 0.23, 0.22, 0.15, 0.12, 0.18, 0.12, 0.21
FSR readings: 0.24, 0.19, 0.25, 0.19, 0.21, 0.20, 0.13, 0.12, 0.09, 0.23
FSR readings: 0.22, 0.19, 0.29, 0.25, 0.20, 0.16, 0.09, 0.15, 0.12, 0.22
FSR readings: 0.20, 0.20, 0.22, 0.17, 0.20, 0.13, 0.13, 0.15, 0.10, 0.26
FSR readings: 0.20, 0.22, 0.23, 0.22, 0.24, 0.15, 0.11, 0.10, 0.10, 0.21
FSR readings: 0.19, 0.25, 0.27, 0.26, 0.19, 0.14, 0.12, 0.14, 0.08, 0.22
FSR readings: 2.65, 2.42, 2.43, 2.32, 2.36, 2.26, 2.21, 2.19, 2.19, 959.51
FSR readings: 2.13, 1.94, 1.90, 1.86, 1.91, 1.82, 1.79, 1.68, 1.77, 923.70
FSR readings: 2.28, 1.98, 2.02, 1.98, 1.93, 1.86, 1.84, 1.76, 1.70, 1010.84
FSR readings: 0.24, 0.20, 0.23, 0.28, 0.19, 0.17, 0.14, 0.14, 0.13, 0.25
FSR readings: 2.34, 1.97, 2.04, 1.96, 1.94, 1.87, 1.90, 1.77, 1.78, 1012.38
FSR readings: 2.32, 2.09, 2.01, 1.92, 1.94, 1.86, 1.85, 1.76, 1.75, 1013.19
FSR readings: 2.33, 2.06, 2.06, 2.02, 1.95, 1.83, 1.88, 1.77, 1.78, 1013.13
FSR readings: 2.33, 2.04, 2.08, 1.97, 1.98, 1.89, 1.83, 1.79, 1.82, 1013.23
FSR readings: 2.32, 2.00, 2.05, 1.98, 1.92, 1.88, 1.85, 1.73, 1.85, 1013.13
FSR readings: 2.28, 2.06, 2.09, 1.98, 2.02, 1.86, 1.83, 1.70, 1.84, 1013.11
FSR readings: 2.33, 2.06, 2.03, 1.97, 1.90, 1.89, 1.86, 1.72, 1.78, 1013.15
FSR readings: 1.70, 1.50, 1.51, 1.41, 1.38, 1.32, 1.34, 1.23, 1.30, 717.72
FSR readings: 0.20, 0.19, 0.29, 0.23, 0.20, 0.16, 0.14, 0.12, 0.15, 0.22
FSR readings: 0.23, 0.21, 0.25, 0.25, 0.26, 0.19, 0.11, 0.12, 0.10, 0.20
FSR readings: 1.96, 1.98, 1.93, 1007.72, 2.31, 1.83, 1.83, 1.76, 1.75, 2.01
FSR readings: 0.18, 0.27, 0.19, 3.17, 0.17, 0.17, 0.10, 0.15, 0.10, 0.20
FSR readings: 0.20, 0.20, 0.25, 0.22, 0.19, 0.18, 0.12, 0.13, 0.16, 0.16
FSR readings: 997.52, 2.40, 1.92, 1.88, 1.89, 1.83, 1.77, 1.79, 1.80, 1.96
FSR readings: 994.11, 2.34, 1.99, 1.91, 1.86, 1.84, 1.77, 1.83, 1.77, 1.98
FSR readings: 0.23, 0.27, 0.24, 0.20, 0.20, 0.19, 0.14, 0.16, 0.11, 0.23
FSR readings: 2.02, 2.06, 1.98, 1.88, 1.84, 998.79, 2.24, 1.80, 1.79, 1.98
FSR readings: 2.02, 2.10, 1.98, 1.94, 1.90, 1002.07, 2.24, 1.82, 1.84, 1.98
FSR readings: 0.20, 0.21, 0.25, 0.20, 0.18, 0.21, 0.12, 0.09, 0.10, 0.24
FSR readings: 1.79, 1.80, 1.83, 1.79, 1.77, 1.70, 924.95, 2.11, 1.66, 1.79
FSR readings: 1.91, 1.92, 1.91, 1.89, 1.88, 1.81, 986.32, 2.25, 1.77, 1.90
FSR readings: 0.20, 0.22, 0.25, 0.26, 0.19, 0.19, 0.14, 0.13, 0.10, 0.26
FSR readings: 0.23, 0.23, 0.27, 0.25, 0.20, 0.21, 0.15, 0.10, 0.10, 0.24
FSR readings: 0.82, 0.83, 0.85, 0.81, 358.22, 0.97, 0.72, 0.75, 0.71, 0.85
FSR readings: 1.88, 1.88, 1.86, 1.84, 993.05, 2.27, 1.81, 1.81, 1.82, 1.97
FSR readings: 1.86, 1.87, 1.89, 1.86, 995.14, 2.36, 1.86, 1.78, 1.83, 2.02
FSR readings: 0.21, 0.21, 0.27, 0.27, 0.25, 0.18, 0.15, 0.19, 0.10, 0.26
FSR readings: 1.25, 1.05, 1.00, 1.05, 0.98, 0.90, 0.91, 0.79, 0.88, 463.28
FSR readings: 2.31, 1.97, 1.99, 1.96, 1.88, 1.82, 1.82, 1.67, 1.77, 1001.52
FSR readings: 0.22, 0.26, 0.25, 0.22, 0.23, 0.20, 0.12, 0.11, 0.12, 0.27
FSR readings: 1.96, 2.02, 2.07, 1009.78, 2.32, 1.90, 1.84, 1.86, 1.83, 1.99
FSR readings: 0.22, 0.23, 0.26, 0.23, 0.18, 0.21, 0.12, 0.14, 0.09, 0.25
FSR readings: 0.72, 0.65, 0.70, 0.68, 0.56, 253.30, 0.67, 0.56, 0.55, 0.65
FSR readings: 1.78, 1.79, 1.82, 1.78, 1.71, 903.41, 2.04, 1.65, 1.64, 1.77
FSR readings: 2.02, 1.98, 2.04, 1.97, 1.89, 987.22, 2.22, 1.76, 1.82, 2.01
FSR readings: 0.20, 0.22, 0.24, 0.23, 0.20, 0.17, 0.11, 0.13, 0.13, 0.23
FSR readings: 1.95, 1.98, 1.93, 1.97, 1.90, 1.82, 1.76, 1.77, 998.40, 2.38
FSR readings: 1.93, 2.00, 2.02, 1.98, 1.90, 1.83, 1.82, 1.74, 1004.08, 2.35
FSR readings: 0.40, 0.46, 0.45, 0.45, 0.38, 0.37, 0.30, 116.82, 0.35, 0.43
FSR readings: 1.96, 1.94, 1.94, 1.90, 1.89, 1.76, 1.79, 988.05, 2.16, 1.87
FSR readings: 1.96, 1.97, 1.97, 1.90, 1.85, 1.76, 1.83, 986.97, 2.14, 1.92
FSR readings: 1.95, 1.95, 1.92, 1.96, 1.90, 1.79, 1.83, 985.92, 2.15, 1.91
FSR readings: 0.23, 0.28, 0.25, 0.25, 0.22, 0.20, 0.13, 0.11, 0.10, 0.22
FSR readings: 0.22, 0.28, 0.25, 0.22, 0.23, 0.16, 0.16, 0.16, 0.10, 0.25
---- Closed serial port /dev/ttyUSB0 due to disconnection from the machine ----

---

## Concrete fixes & suggestions (step-by-step)
1. **Print raw integers for debugging**  
   Temporarily change the printing to print `(int)fsrMean[s]` to confirm the raw ADC values before scaling. That will show whether values are 0..4095 or normalized 0..1.

2. **Force ADC resolution & scaling** (if supported by your core)  
   Add at the start of `setup()`:
   ```cpp
   #if defined(analogReadResolution)
   analogReadResolution(12); // 12-bit: 0..4095
   #endif
   ```
   After averaging, explicitly convert to voltage:
   ```cpp
   float voltage = fsrMean[s] / 4095.0f * 3.3f;
   ```

3. **Check pin mapping**  
   Verify `fsrPins[]` entries are ADC-capable on your MCU package. See the board pin mapping or STM32F401 datasheet. If a pin isn't ADC-capable, move sensor to a valid ADC pin.

4. **Check wiring**  
   - Ensure each FSR is wired: Vcc (3.3V) → R_fixed (10k) → ADC node → FSR → GND (or the alternate arrangement you used).  
   - Ensure common ground between sensor Vcc, STM32 ground, and any other power supplies.  
   - Re-seat connectors and test with single sensor connected to eliminate cross-talk.

5. **Guard against extreme values**  
   If you cannot eliminate spikes, clamp values in software:
   ```cpp
   if (fsrMean[s] > 5000) fsrMean[s] = 0; // discard impossible values
   ```
   But prefer to find root cause rather than mask it.

6. **Calibrate**  
   Convert ADC → voltage → resistance → force using a bench calibration. Place known weights on the sensor and record readings. Fit a curve (often a power law) to map voltage/resistance to force.

7. **Reduce sampling window for faster response**  
   If you need faster updates reduce `samples` or `delay_ms`. For a trade-off, use exponential moving average (EMA) instead of blocking averaging.

8. **Use shielded cables or keep wires short** if you see large spikes around motors or other noisy equipment.

---

## Example: change to print integers and voltage (sample patch)
Add to `setup()`:
```
analogReadResolution(12); // ensure 12-bit reads (if supported)
```

Replace printing block with:
```
Serial.print("FSR raw: ");
for (int s = 0; s < NUM_SENSORS; s++) {
  int raw = (int)fsrMean[s]; // mean of integer reads
  float voltage = raw * 3.3f / 4095.0f;
  Serial.print(raw);
  Serial.print(" (");
  Serial.print(voltage, 3);
  Serial.print(" V)");
  if (s < NUM_SENSORS-1) Serial.print(", ");
}
Serial.println();
```

This prints both the raw ADC (0..4095) and the scaled voltage so you can see exactly what the ADC is returning and detect whether there's a core/resolution mismatch.

---

