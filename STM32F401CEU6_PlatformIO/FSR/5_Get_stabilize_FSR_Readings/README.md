# ⚡ STM32 FSR Auto-Save System (Black Pill F401)

An Arduino sketch tailored for the **STM32 Black Pill (F401CEU6)** to monitor multiple **Force Sensing Resisting (FSR) sensors**. The system continuously monitors the pressure readings and automatically saves a "snapshot" of the stable data once the entire sensor array has remained motionless or constant for a set period.

## 🎯 Key Features

* **Platform Specific:** Optimized for the STM32 Black Pill (F401CEU6) pinout.
* **10-Point Sensing:** Configured for 10 FSRs, suitable for pressure mapping or multi-point load sensing.
* **Noise Reduction:** Implements a mean-averaging filter to smooth sensor readings.
* **Stability Detection:** Uses a fixed threshold and time-based counter to detect when **all** sensors are stable.
* **Auto-Snapshot:** Saves the stable data to a variable (`savedData`) and prints it to the Serial Monitor.

## 🛠️ Hardware & Components

### 1. Microcontroller Board
* **STM32 Black Pill V3.0** (STM32F401CEU6)

### 2. Sensors
* **10x Force Sensing Resistor** (FSR 400 series, e.g., Pololu 2728).

### 3. External Circuitry
* **10x 10 kΩ Resistors:** Used for the pull-down leg of the voltage divider circuit for each FSR.
* **10x 0.1 μF Ceramic Capacitors:** Connected in parallel with the pull-down resistor to create a **Low-Pass Filter (RC filter)**. This significantly reduces high-frequency noise and stabilizes the readings, especially important for the fast ADC on the STM32.

### 4. Wiring and Pinout

The FSRs are connected to the STM32's **Analog Input Pins (A)**.

| FSR Sensor # | STM32 Black Pill Pin | Arduino Pin Name |
| :----------: | :------------------: | :--------------: |
| FSR 1 | PB1 | `PB1` |
| FSR 2 | PB0 | `PB0` |
| FSR 3 | PA7 | `PA7` |
| FSR 4 | PA6 | `PA6` |
| FSR 5 | PA5 | `PA5` |
| FSR 6 | PA4 | `PA4` |
| FSR 7 | PA3 | `PA3` |
| FSR 8 | PA2 | `PA2` |
| FSR 9 | PA1 | `PA1` |
| FSR 10 | PA0 | `PA0` |

> **💡 Note on Wiring:** Each FSR should be wired in a **voltage divider** configuration. One side of the FSR connects to **3.3V** (or 5V, depending on your setup), and the other side connects to the **GND** via a **10 kΩ pull-down resistor**. The analog reading is taken from the point *between* the FSR and the pull-down resistor/capacitor. 

## ⚙️ Configuration Parameters

The following constants in the code dictate the system's behavior and sensitivity:

| Variable | Value | Unit | Description |
| :--- | :--- | :--- | :--- |
| `samples` | `80` | Readings | The number of times the ADC reads the sensor in a single loop cycle. Higher value = smoother readings, but slower cycle time. |
| `delay_ms` | `3` | ms | Delay between individual samples. Ensures the ADC is settled between readings. |
| `threshold` | `15.0` | ADC Units | The maximum allowed fluctuation (change) between the current cycle's mean reading and the previous cycle's mean reading for stability. Lower value = more sensitive to movement. |
| `stableCyclesNeeded` | `5` | Cycles | The number of **consecutive** main loops where **all 10** sensors must be stable to trigger the auto-save. |

## 💻 Operation and Output

The code is designed to print live sensor data continuously and print a special log only when a stable state is reached.

### Live Output

* **FSR Live:** Displays the current, averaged reading of all 10 sensors.

### Auto-Save Event

When the stability condition is met:

===================================== 
STABLE → DATA SAVED FSR Snapshot: 2.11, 2.14, 2.06, 1.91, 2.01, 2.10, 1.91, 785.97, 827.55, 2.33

The values in the **FSR Snapshot** are stored permanently in the `savedData` array until the system is reset or another stable event occurs (after an unstable period).


