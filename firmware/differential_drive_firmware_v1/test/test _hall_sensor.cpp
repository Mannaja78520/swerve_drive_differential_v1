#include <Arduino.h>
#include <config.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);
  pinMode(Hall_Sensor1, INPUT_PULLUP);

}

void loop() {
  Serial.print("Hall Sensor State: ");
  Serial.println(digitalRead(Hall_Sensor1) == LOW ? "Magnet Detected" : "No Magnet");
  delay(1000); // Delay for readability
  }