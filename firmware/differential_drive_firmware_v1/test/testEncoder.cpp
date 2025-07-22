#include <Arduino.h>
#include <config.h>

#include <ESP32Encoder.h>

ESP32Encoder encoder;

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
}

void loop(){
	// Loop and read the count
	Serial.println("Encoder count = " + String((int32_t)encoder.getCount()));
	// delay(100);
}
