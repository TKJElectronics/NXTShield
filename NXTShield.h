/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 
 Contact information
 -------------------
 
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#ifndef NXTShield_h
#define NXTShield_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Wire.h"

/* Ultrasonic Sensor */
#define SCL_Frequency   11494.252873563 // The same I2C frequency as the NXT
#define prescaler       4
#define sensorAddress   0x01 // 0x01 in a 7-bit context 0x02 in a 8-bit context
#define readMeasurement 0x42
#define clockPin        4 // It needs an extra clock pulse before reading again, we use this pin for that

/* Motors */
//Encoders
#define tachFirst1      2
#define tachFirst2      5
#define tachSecond1     3
#define tachSecond2     6

//First motor
#define firstPWM        9
#define logicFirst1     7
#define logicFirst2     11

//Second motor
#define secondPWM       10
#define logicSecond1    12
#define logicSecond2    8

//Enums for move functions
enum Direction {
    backward = 0,
    forward,
};
enum Brake {
    brake = 0,
    coast,
};

class UltrasonicSensor { // To use the ultrasonic sensor with an Arduino Mega, one have to connect pin 20 (SDA) to A4 and pin 21 (SCL) to A5
public:
    UltrasonicSensor();
    int readDistance(void);
    int* readCommand(int address,int length);
    void setCommand(int address,int command);
private:
    void clockPulse(void);    
    unsigned long _timer;
};
class Motor1 {    
public:
    Motor1();
    void move(Direction direction, uint8_t torque);
    void move(Direction direction, uint8_t torque, int rotation, Brake brake);
    void stop(void);    
    void coast(void);
    long readPosition(void);
    void resetPosition(void);
    bool isTurning(void);    
};
class Motor2 {    
public:
    Motor2();
    void move(Direction direction, uint8_t torque);
    void move(Direction direction, uint8_t torque, int rotation, Brake brake);
    void stop(void);    
    void coast(void);
    long readPosition(void);
    void resetPosition(void);
    bool isTurning(void);    
};

#endif