/*
 * NXTShield library
 * Version 1.0 March, 2012
 * Copyright 2012 Kristian Lauszus, TKJ Electronics
 * For details, see http://blog.tkjelectronics.dk/
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
    backward,
    forward,
};
enum Brake {
    brake = 0,
    coast = 1,
};

class UltrasonicSensor {
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
    int readPosition(void);
    void resetPosition(void);
};
class Motor2 {    
public:
    Motor2();
    void move(Direction direction, uint8_t torque);
    void move(Direction direction, uint8_t torque, int rotation, Brake brake);
    void stop(void);    
    void coast(void);
    int readPosition(void);
    void resetPosition(void);    
};

#endif