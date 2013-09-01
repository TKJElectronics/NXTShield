#include <Wire.h>
#include <NXTShield.h>

int readProductID = 0x08;

void setup() {
  Serial.begin(115200);

  uint8_t data[4];
  uint8_t rcode = UltrasonicSensor.readCommand(readProductID, data, 4);
  if (!rcode) { // Check error code
    Serial.write(data, 4); // This should print "LEGO"
    Serial.println();
  } else
    Serial.println("Error reading sensor");
}

void loop() {
  int distance = UltrasonicSensor.readDistance();
  if (distance != -1)
    Serial.println(distance);
  else
    Serial.println("Error reading sensor");
}
