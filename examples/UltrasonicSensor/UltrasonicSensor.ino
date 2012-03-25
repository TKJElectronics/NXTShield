#include <Wire.h>
#include <NXTShield.h>

UltrasonicSensor sensor;
int readProductID = 0x08;

void setup() {
  Serial.begin(115200);

  int* data = sensor.readCommand(readProductID, 4);
  for(int i = 0; i < 4 ; i++) // This should print "LEGO"
    Serial.write(data[i]);
  Serial.println("");
}

void loop() {
    Serial.println(sensor.readDistance());
}
