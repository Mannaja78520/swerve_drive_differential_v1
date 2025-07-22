/*!
 * @file AS5600_basic.ino
 *
 * Basic example for the Adafruit AS5600 library
 *
 * Written by Limor Fried for Adafruit Industries.
 * MIT license, all text above must be included in any redistribution
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_AS5600.h>

#include <Utilize.h>
#include <PIDF.h>
#include <ESP32Servo.h> 

Adafruit_AS5600 as5600;
float max_val = 70.0;
float min_val = -max_val;
float Kp = 13.0;
float Ki = 0.0;
float i_min = -max_val;
float i_max = max_val;
float Kd = 3.0;
float Kf = 0.0;
float error_tolerance = 1.5; // 0.75 degrees

float target_angle = 0;

PIDF rotage_wheel1(min_val, max_val, Kp, Ki, i_min, i_max , Kd, Kf, error_tolerance);
Servo Servo_rotage_wheel1;

int Servo_rotage_wheel1_Pin = 27;

void setup() {
    Serial.begin(115200);
    while (!Serial)
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    Servo_rotage_wheel1.setPeriodHertz(50);
    Servo_rotage_wheel1.attach(Servo_rotage_wheel1_Pin, 500, 2500); // Attach the servo to pin 27 with min and max pulse widths
    Servo_rotage_wheel1.write(90);
    delay(500);
    
    Serial.println("Adafruit AS5600 Basic Test");
    
    if (!as5600.begin()) {
        Serial.println("Could not find AS5600 sensor, check wiring!");
        while (1)
        delay(10);
    }
    
    Serial.println("AS5600 found!");
    
    
    as5600.enableWatchdog(false);
    // Normal (high) power mode
    as5600.setPowerMode(AS5600_POWER_MODE_NOM);
    // No Hysteresis
    as5600.setHysteresis(AS5600_HYSTERESIS_OFF);
    
    // analog output
    as5600.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);
    
    // OR can do pwm! 
    // as5600.setOutputStage(AS5600_OUTPUT_STAGE_DIGITAL_PWM);
    // as5600.setPWMFreq(AS5600_PWM_FREQ_920HZ);
    
    // setup filters
    as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
    as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
    
    // Reset position settings to defaults
    as5600.setZPosition(0);
    as5600.setMPosition(4095);
    as5600.setMaxAngle(4095);
    
    Serial.println("Waiting for magnet detection...");
}

void loop() {
    // if (! as5600.isMagnetDetected()) {
    //     return;
    // }

    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        target_angle = WrapDegs(input.toFloat());
        if (target_angle < 0 || target_angle > 360) {
            Serial.println("Invalid angle! Please enter a value between 0 and 360.");
        }
    }

    // Continuously read and display angle values
    uint16_t angle = as5600.getAngle();
    float angleDegs = WrapDegs(angle * 360.0 / 4096.0);
    float output = rotage_wheel1.compute_with_error(WrapDegs(target_angle - angleDegs));

    float drive_output = 1500 + output;
    Servo_rotage_wheel1.writeMicroseconds(drive_output); // Adjust the servo position based on PIDF output
    // delay(1000);

    Serial.print("now: ");
    Serial.print(angleDegs, 2);
    Serial.print(" | target: ");
    Serial.print(target_angle, 2);
    Serial.print(" | output: ");
    Serial.print(output, 2);
    Serial.print(" | drive_output: ");
    Serial.print(drive_output);

    // // Check status conditions
    // if (as5600.isAGCminGainOverflow()) {
    //     Serial.print(" | MH: magnet too strong");
    // }
    // if (as5600.isAGCmaxGainOverflow()) {
    //     Serial.print(" | ML: magnet too weak");
    // }

    Serial.println();

}