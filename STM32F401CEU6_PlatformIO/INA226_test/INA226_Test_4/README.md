# INA226 Current Measurement with STM32 Black Pill (PlatformIO)

This project demonstrates how to interface the **INA226 Current & Voltage Sensor Module (20A)** with an **STM32 Black Pill (STM32F411/STM32F401)** using PlatformIO + Arduino framework.

---

## 🔬 Sensor Used

**INA226 Current/Voltage Sensor (20A)**

Product link: [https://tronic.lk/product/ina226-current-voltage-sensor-module-20a](https://tronic.lk/product/ina226-current-voltage-sensor-module-20a)

The setup measures:

* **Shunt Voltage** (mV)
* **Bus Voltage** (V)
* **Load Voltage** (V)
* **Current** (mA)
* **Bus Power** (mW)

The load used in testing is a red **LED bulb + resistor**.

---

## 📌 Wiring (STM32 Black Pill → INA226)

| STM32 Pin | INA226 Pin |
| :---: | :---: |
| PB7 (SDA) | SDA |
| PB6 (SCL) | SCL |
| 3.3V | VCC |
| GND | GND |

---

## 📌 Features of this Project

* **I²C Scanner** to detect sensor on the bus
* INA226 initialization & error handling
* Continuous voltage, current, and power reading
* Handling I²C error codes
* Measurements logged every **3 seconds**
* Verified theoretical vs real current values with multiple resistors

---

## 📘 Code Used

This code uses the `INA226_WE` library to interface with the sensor and includes an I²C scanner and error checking.

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <INA226_WE.h>

#define I2C_ADDRESS 0x40

/* INA226 object */
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);

void checkForI2cErrors();

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Scanning I2C bus for devices...");

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();

  bool found = false;

  // I2C Scanner
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
      if (addr == 0x40) {
        found = true;
      }
    }
  }

  if (found) {
    Serial.println("INA226 detected! Initializing...");
  } else {
    Serial.println("INA226 not found on I2C bus.");
  }

  // Initialize INA226 once
  if (!ina226.init()) {
    Serial.println("Failed to init INA226. Check your wiring.");
    while (1) {}
  }

  ina226.waitUntilConversionCompleted();
}

void loop() {
  float shuntVoltage_mV = 0.0;
  float loadVoltage_V = 0.0;
  float busVoltage_V = 0.0;
  float current_mA = 0.0;
  float power_mW = 0.0;

  shuntVoltage_mV = ina226.getShuntVoltage_mV();
  busVoltage_V = ina226.getBusVoltage_V();
  current_mA = ina226.getCurrent_mA();
  power_mW = ina226.getBusPower();
  loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

  checkForI2cErrors();

  Serial.print("Shunt Voltage [mV]: ");
  Serial.println(shuntVoltage_mV);
  Serial.print("Bus Voltage [V]: ");
  Serial.println(busVoltage_V);
  Serial.print("Load Voltage [V]: ");
  Serial.println(loadVoltage_V);
  Serial.print("Current[mA]: ");
  Serial.println(current_mA);
  Serial.print("Bus Power [mW]: ");
  Serial.println(power_mW);

  if (!ina226.overflow) {
    Serial.println("Values OK - no overflow");
  } else {
    Serial.println("Overflow! Choose higher current range");
  }
  Serial.println();

  delay(3000);
}

void checkForI2cErrors() {
  byte errorCode = ina226.getI2cErrorCode();

  if (errorCode) {
    Serial.print("I2C error: ");
    Serial.println(errorCode);

    switch (errorCode) {
      case 1:
        Serial.println("Data too long to fit in transmit buffer");
        break;
      case 2:
        Serial.println("Received NACK on transmit of address");
        break;
      case 3:
        Serial.println("Received NACK on transmit of data");
        break;
      case 4:
        Serial.println("Other error");
        break;
      case 5:
        Serial.println("Timeout");
        break;
      default:
        Serial.println("Can't identify the error");
    }

    while (1) {}
  }
}

```
## 📊 Summary of Measured Values

<img width="1112" height="374" alt="Screenshot" src="https://github.com/user-attachments/assets/90715e3c-3084-4980-bb10-8c35183ab7e6" />

---

## 📟 Serial Monitor Output

Below are the exact serial readings for each resistor value.

### 1. 10 kΩ Resistor
```plaintext
Shunt Voltage [mV]: 0.04
Bus Voltage [V]: 5.26
Load Voltage [V]: 5.26
Current[mA]: 0.40
Bus Power [mW]: 1.88
Values OK - no overflow
2. 1 kΩ Resistor
plaintext
Copy code
Shunt Voltage [mV]: 0.38
Bus Voltage [V]: 5.24
Load Voltage [V]: 5.24
Current[mA]: 3.85
Bus Power [mW]: 20.62
Values OK - no overflow
3. 330 Ω Resistor
plaintext
Copy code
Shunt Voltage [mV]: 0.98
Bus Voltage [V]: 5.18
Load Voltage [V]: 5.18
Current[mA]: 9.85
Bus Power [mW]: 51.25
Values OK - no overflow
4. 110 Ω Resistor
plaintext
Copy code
Shunt Voltage [mV]: 2.43
Bus Voltage [V]: 5.09
Load Voltage [V]: 5.09
Current[mA]: 24.27
Bus Power [mW]: 123.75
Values OK - no overflow
📈 Observations
Measured current closely matches theoretical values, especially for lower resistances.

Percentage deviation increases for high-value resistors due to:

INA226 resolution limitations at low currents

LED non-linear behavior

No overflow occurred in any measurement.

📦 PlatformIO Configuration
Ensure your platformio.ini includes the correct environment settings for the STM32 Black Pill:

ini
Copy code
[env:blackpill_f411ce]
platform = ststm32
board = blackpill_f411ce
framework = arduino
upload_protocol = stlink
monitor_speed = 115200
📝 Conclusion
This project successfully validates the use of the INA226 for low-power current sensing with STM32 microcontrollers. The readings are accurate and stable, making the setup suitable for LED load testing, battery monitoring, and power analysis applications.
