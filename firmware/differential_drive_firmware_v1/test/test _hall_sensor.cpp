#include <Arduino.h>
#include <config.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);
  pinMode(33, INPUT_PULLUP);

}

void loop() {
  Serial.print("Hall Sensor State: ");
  Serial.println(digitalRead(33) == LOW ? "Magnet Detected" : "No Magnet");
  delay(1000); // Delay for readability
  }