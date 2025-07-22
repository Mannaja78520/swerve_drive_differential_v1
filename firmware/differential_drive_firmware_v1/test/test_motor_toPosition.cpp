#include <Arduino.h>
#include <config.h>

#include <ESP32Encoder.h>
#include <PIDF.h>
#include <motor.h>
#include <config.h>

float max_val = 1023.0;
float min_val = -max_val;
float Kp = 0.6;
float Ki = 0.0002;
float i_min = -1;
float i_max = -1;
float Kd = 0.2;
float Kf = 0.0;
float error_tolerance = 7; // 0.75 degrees

int target_position = 0;


ESP32Encoder encoder;
PIDF motor1_PIDF(min_val, max_val, Kp, Ki, i_min, i_max , Kd, Kf, error_tolerance);
Controller motor1(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BRAKE, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
void setup(){
	
	Serial.begin(115200);
	// Enable the weak pull down resistors

	//ESP32Encoder::useInternalWeakPullResistors = puType::down;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors = puType::up;

	// use pin 19 and 18 for the first encoder
	encoder.attachFullQuad(MOTOR1_ENCODER_PIN_A, MOTOR1_ENCODER_PIN_B);

	// set starting count value after attaching
	encoder.setCount(0);


	Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));
    // motor1.spin(200); 
    // delay(1000); 
    // motor1.spin(0); // Stop the motor after initial spin
    // delay(1000);
}

void loop(){
	// Loop and read the count
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        target_position = input.toFloat();
    }
    
	Serial.println("Encoder count = " + String((int32_t)encoder.getCount()));
	float output = motor1_PIDF.compute(target_position, encoder.getCount());
    // Serial.println(output);
	motor1.spin(output);
	// delay(100);
}
