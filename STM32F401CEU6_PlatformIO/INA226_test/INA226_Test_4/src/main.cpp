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

  // Initialize INA226 once (your original logic)
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
