#include <Wire.h>
#include <NXTShield.h>

void setup() {
  Serial.begin(115200);
}

void loop(){
  // Forward at full speed for 2 seconds
  Serial.println("Forward for 2 seconds");
  Motor1.move(FORWARD, 255);
  Motor2.move(FORWARD, 255);
  delay(2000);

  // Backward at full speed for 2 seconds
  Serial.println("Backward for 2 seconds");
  Motor1.move(BACKWARD, 255);
  Motor2.move(BACKWARD, 255);
  delay(2000);

  // Stop both motors for 2 seconds
  Serial.println("Stop for 2 seconds");
  Motor1.stop();
  Motor2.stop();
  delay(2000);

  // Rotate 360 degrees (one resolution) and coast motors
  Serial.println("Coast");
  Motor1.move(FORWARD, 255, 360, COAST);
  Motor2.move(FORWARD, 255, 360, COAST);
  while (Motor1.isTurning() || Motor2.isTurning()); // Wait until it has reached the position
  delay(2000);

  // Rotate 360 degrees (one resolution) and brake motors
  Serial.println("Brake");
  Motor1.move(BACKWARD, 255, 360, BRAKE);
  Motor2.move(BACKWARD, 255, 360, BRAKE);
  while (Motor1.isTurning() || Motor2.isTurning()); // Wait until it has reached the position
  delay(2000);
}
