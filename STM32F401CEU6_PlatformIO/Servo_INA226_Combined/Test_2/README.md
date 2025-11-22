# MG996R Servo + INA226 Current Sensor with STM32 Black Pill

This project demonstrates controlling a **MG996R servo motor** with an **STM32 Black Pill (STM32F411/STM32F401)** while measuring current, voltage, and power using the **INA226 Current & Voltage Sensor Module (20A)**. The code is built using **PlatformIO + Arduino framework**.

---

## 🔬 Components Used

* **MG996R Servo Motor**
* **INA226 Current/Voltage Sensor (20A)**
* **STM32 Black Pill (STM32F411/STM32F401)**

**INA226 Product Link:** [Tronic INA226 Sensor](https://tronic.lk/product/ina226-current-voltage-sensor-module-20a)

---

## 📌 Pin Connections

| STM32 Pin | Component |
| :-------: | :------- |
| PB13      | Servo Signal (MG996R) |
| PB7       | INA226 SDA |
| PB6       | INA226 SCL |
| 3.3V      | INA226 VCC |
| GND       | Servo & INA226 GND |

---

## 📘 Features

* I²C Scanner to detect INA226 on the bus
* INA226 initialization & error handling
* Continuous reading of:
  * Shunt Voltage (mV)
  * Bus Voltage (V)
  * Load Voltage (V)
  * Current (mA)
  * Bus Power (mW)
* Servo control with MG996R
* Current monitoring during servo motion
* Measurements logged every 0.5–3 seconds

---

## 💻 Code Overview

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <INA226_WE.h>

#define I2C_ADDRESS 0x40
Servo myservo;
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);
#define SERVO_PIN PB13

void checkForI2cErrors();
void printINA226Data();
void moveServoAndMeasure(int angle, int waitMs);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== MG996R Servo + INA226 Current Sensor Test ===");
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();

  // I2C Scanner
  bool found = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (addr == 0x40) found = true;
    }
  }

  if (!ina226.init()) while(1) {} // INA226 init
  ina226.waitUntilConversionCompleted();

  myservo.attach(SERVO_PIN);
  Serial.println("Servo attached to PB13.");
  delay(1000);
}

void loop() {
  moveServoAndMeasure(0, 3000);
  moveServoAndMeasure(180, 3000);
  moveServoAndMeasure(90, 3000);
}

void checkForI2cErrors() {
  byte errorCode = ina226.getI2cErrorCode();
  if (errorCode) {
    while (1) {}
  }
}

void printINA226Data() {
  float shuntVoltage_mV = ina226.getShuntVoltage_mV();
  float busVoltage_V = ina226.getBusVoltage_V();
  float current_mA = ina226.getCurrent_mA();
  float power_mW = ina226.getBusPower();
  float loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

  checkForI2cErrors();

  Serial.print("Shunt Voltage [mV]: "); Serial.println(shuntVoltage_mV);
  Serial.print("Bus Voltage [V]: "); Serial.println(busVoltage_V);
  Serial.print("Load Voltage [V]: "); Serial.println(loadVoltage_V);
  Serial.print("Current[mA]: "); Serial.println(current_mA);
  Serial.print("Bus Power [mW]: "); Serial.println(power_mW);

  if (!ina226.overflow) Serial.println("Values OK - no overflow");
  else Serial.println("Overflow! Choose higher current range");
  Serial.println();
  delay(3000);
}

void moveServoAndMeasure(int angle, int waitMs) {
  Serial.print("Moving servo to "); Serial.print(angle); Serial.println("°");
  myservo.write(angle);
  unsigned long start = millis();
  while (millis() - start < waitMs) {
    printINA226Data();
    delay(500);
  }
}
```

## 📟 Sample Serial Monitor Output
---
```plaintext
Moving servo to 0°
Shunt Voltage [mV]: 0.53
Bus Voltage [V]: 5.24
Load Voltage [V]: 5.24
Current[mA]: 5.30
Bus Power [mW]: 27.50
Values OK - no overflow
```
```plaintext
Moving servo to 180°
Shunt Voltage [mV]: 1.68
Bus Voltage [V]: 5.11
Load Voltage [V]: 5.11
Current[mA]: 16.80
Bus Power [mW]: 86.25
Values OK - no overflow
```
```plaintext
Moving servo to 90°
Shunt Voltage [mV]: 0.52
Bus Voltage [V]: 5.26
Load Voltage [V]: 5.26
Current[mA]: 5.32
Bus Power [mW]: 27.50
Values OK - no overflow
```

## 📦 PlatformIO Configuration
--
```plaintext
[env:blackpill_f411ce]
platform = ststm32
board = blackpill_f411ce
framework = arduino
upload_protocol = stlink
monitor_speed = 115200
```

## 📝 Observations & Conclusion
---
```plaintext
Current drawn by the servo increases when moving to extreme positions (0° or 180°).

Measurements are stable and show no overflow with INA226 at the chosen current range.

This setup is suitable for servo testing, power analysis, and low-power robotic applications with STM32.
```
