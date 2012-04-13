#include <Wire.h>
#include <NXTShield.h>

Motor1 leftMotor;
Motor2 rightMotor;

void setup() {
  Serial.begin(115200);
}

void loop(){
    // Forward at full speed for 2 seconds
    Serial.println("Forward for 2 seconds");
    leftMotor.move(forward, 255);
    rightMotor.move(forward, 255);
    delay(2000);
    
    // Backward at full speed for 2 seconds
    Serial.println("Backward for 2 seconds");
    leftMotor.move(backward, 255);
    rightMotor.move(backward, 255);
    delay(2000);
    
    // Stop both motors for 2 seconds
    Serial.println("Stop for 2 seconds");
    leftMotor.stop();
    rightMotor.stop();
    delay(2000);
    
    // Rotate 360 degrees (one resolution) and coast motors
    Serial.println("Coast");
    leftMotor.move(forward, 255, 360, coast);
    rightMotor.move(forward, 255, 360, coast);
    while(leftMotor.isTurning()); // Wait until it has reach the position
    while(rightMotor.isTurning());
    delay(500);
    
    // Rotate 360 degrees (one resolution) and brake motors
    Serial.println("Brake");
    leftMotor.move(backward, 255, 360, brake);
    rightMotor.move(backward, 255, 360, brake);
    while(leftMotor.isTurning()); // Wait until it has reach the position
    while(rightMotor.isTurning());
    delay(500);
}
