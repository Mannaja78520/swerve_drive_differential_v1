#ifndef TEENSY_HARDWARE_PIN_H
#define TEENSY_HARDWARE_PIN_H


#define K_P 0.6
#define K_I 0.8
#define K_D 0.5
#define K_F 0.2
#define I_Max -1
#define I_Min -1 

//define your robot' specs here
#define MOTOR_MAX_RPM 7200                                               // motor's max RPM          
#define MAX_RPM_RATIO 0.85                                              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 24                                      // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 24                                      // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 24                                 // current voltage reading of the power connected to the motor (used for calibration)
#define ENCODER1_PULSES_PER_REVOLUTION 2500                             // encoder 1 pulse
#define ENCODER2_PULSES_PER_REVOLUTION 2000                             // encoder 2 pulse
#define ENCODER3_PULSES_PER_REVOLUTION 2500                             // encoder 3 pulse
// #define ENCODER4_PULSES_PER_REVOLUTION 2500                             // encoder 4 pulse
#define ENCODER_TICKS 4                                                 // encoder ticks
#define COUNTS_PER_REV1 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.0762                                           // wheel's diameter in meters
// #define LR_WHEELS_DISTANCE 0.335                                        // distance between left and right wheels
#define PWM_BITS 10                                                     // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000                                             // PWM Frequency
#define PWM_Max pow(2, PWM_BITS) - 1
#define PWM_Min -PWM_Max

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false

//  Motor Brake
#define MOTOR1_BRAKE true
#define MOTOR2_BRAKE true
#define MOTOR3_BRAKE true

// Motor 1 Parameters
#define MOTOR1_PWM  -1
#define MOTOR1_IN_A 9
#define MOTOR1_IN_B 10

// Motor 2 Parameters
#define MOTOR2_PWM  -1
#define MOTOR2_IN_A 11
#define MOTOR2_IN_B 12

// Motor 3 Parameters
#define MOTOR3_PWM  -1
#define MOTOR3_IN_A 23
#define MOTOR3_IN_B 22


// INVERT ENCODER DIRECTIONS
#define MOTOR1_ENCODER_INV false
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV false

// Encoder 1 Parameter
#define MOTOR1_ENCODER_INCRIMENT -1
#define MOTOR1_ENCODER_PIN_A 0
#define MOTOR1_ENCODER_PIN_B 1

// Encoder 2 Parameter
#define MOTOR2_ENCODER_INCRIMENT -1
#define MOTOR2_ENCODER_PIN_A 2
#define MOTOR2_ENCODER_PIN_B 3

// Encoder 3 Parameter
#define MOTOR3_ENCODER_INCRIMENT -1
#define MOTOR3_ENCODER_PIN_A 4  
#define MOTOR3_ENCODER_PIN_B 5 

// Servo Parameter
#define CONTINUTE_SERVO1_PIN 6
#define CONTINUTE_SERVO2_PIN 7
#define CONTINUTE_SERVO3_PIN 8

// I2C communication
#define SCL_PIN 19
#define SDA_PIN 18
#define SCL1_PIN 16
#define SDA1_PIN 17
#define SCL2_PIN 24
#define SDA2_PIN 25

#endif