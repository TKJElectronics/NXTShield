#include <Wire.h>
#include <NXTShield.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print(Motor1.readPosition());
  Serial.print('\t');
  Serial.println(Motor2.readPosition());
  delay(100);
}
