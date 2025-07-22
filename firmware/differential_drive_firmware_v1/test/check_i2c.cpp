#include <Arduino.h>
#include <Wire.h>

void setup() {
  Wire.begin(21, 22);             // Join I2C bus (use Wire.begin(SDA, SCL) for ESP32)
  Serial.begin(115200);
  while (!Serial);          // Wait for Serial Monitor to open (for boards like Leonardo)
  Serial.println("\nI2C Scanner");

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at 0x");
      Serial.println(address, HEX);
    }
  }
}

void loop() {
  // Do nothing
}
