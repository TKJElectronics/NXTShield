#include <Wire.h>
#include <NXTShield.h>

Motor1 leftMotor;
Motor2 rightMotor;

void setup() {
  Serial.begin(115200);
}

void loop() {    
    Serial.print(leftMotor.readPosition(),DEC);
    Serial.print("\t");
    Serial.println(rightMotor.readPosition(),DEC);
    delay(100);
}
