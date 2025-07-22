#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_AS5600.h>
#include <config.h>
#include <TCA9548A.h>
#include <Utilize.h>

#define SENSOR_COUNT 3

TCA9548A tca;
Adafruit_AS5600 as5600;

void setup() {
    Serial.begin(115200);
    tca.begin();

    for (int i = 0; i < SENSOR_COUNT; i++) {
        // if (i == 0) {
        //     tca.disableAll();  // direct sensor
        // } else {
        //     if (!tca.selectChannel(FIRST_TCA_CHANNEL + i - 1)) {
        //         Serial.print("Failed to select TCA channel ");
        //         Serial.println(i - 1);
        //         continue;
        //     }
        // }

        if (!tca.selectChannel(FIRST_TCA_CHANNEL + i)) {
            Serial.print("Failed to select TCA channel ");
            Serial.println(FIRST_TCA_CHANNEL + i);
            continue;
        }

        if (!as5600.begin()) {
            Serial.print("AS5600 ");
            Serial.print(i);
            Serial.println(" not found!");
            continue;
        }

        // You can configure each individually if needed
        as5600.setPowerMode(AS5600_POWER_MODE_NOM);
        as5600.setHysteresis(AS5600_HYSTERESIS_OFF);
        as5600.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);
        as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
        as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
        as5600.setZPosition(0);
        as5600.setMPosition(4095);
        as5600.setMaxAngle(4095);
    }
}

void loop() {
    String output = "";

    for (int i = 0; i < SENSOR_COUNT; i++) {
        // if (i == 0) {
        //     tca.disableAll();
        // } else {
        //     tca.selectChannel(FIRST_TCA_CHANNEL + i - 1);
        // }

        tca.selectChannel(FIRST_TCA_CHANNEL + i);

        // delayMicroseconds(500);  // Give I2C some time to settle

        // as5600.begin();
        // as5600.setMaxAngle(4095); 
        // reinitialize sensor after switching
        if (!as5600.begin()) {
            output += "Sensor " + String(i) + ": --- (fail init) | ";
            continue;
        }

        uint16_t raw = as5600.getAngle();
        if (raw == 0xFFFF) {
            output += "Sensor " + String(i) + ": --- | ";
            continue;
        }

        float angle = WrapDegs(raw * 360.0 / 4096.0);
        output += "Sensor " + String(i) + ": ";
        output += String(angle, 1) + "°";

        if (as5600.isAGCminGainOverflow()) {
            output += " (MH)";
        } else if (as5600.isAGCmaxGainOverflow()) {
            output += " (ML)";
        } else {
            output += " (OK)";
        }

        if (i < SENSOR_COUNT - 1) output += " | ";
    }

    Serial.println(output);
    delay(100);
}
