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

#ifndef _nxtshield_h_
#define _nxtshield_h_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h" // The standard Arduino Wire library

/* Ultrasonic Sensor */
#define sensorAddress   0x01 // 0x01 in a 7-bit context 0x02 in a 8-bit context
#define readMeasurement 0x42

#define BACKWARD 0
#define FORWARD 1
#define BRAKE 2
#define COAST 3

class Sensor { // To use the ultrasonic sensor with an Arduino with a different pinout than the Uno, one have to connect pin 20 (SDA) to A4 and pin 21 (SCL) to A5
public:
    Sensor();
    int16_t readDistance(void); // Returns the distance to the object. Will return -1 on error.
    uint8_t readCommand(uint8_t address, uint8_t *data, uint8_t length); // Return a value above 0 if there is an error.
    uint8_t setCommand(uint8_t address, uint8_t command); // Send different commends to the sensor.

private:
    void clockPulse(void); // The clock pin needs an extra wiggle in the I2C communication
    static const uint8_t clockPin /* = 4 */; // It needs an extra clock pulse before reading again, we use this pin for that
    uint32_t timer;
};

extern Sensor UltrasonicSensor;

class Motor {
public:
    Motor(uint8_t tach1, uint8_t tach2, uint8_t pwm, uint8_t logic1, uint8_t logic2);
    void move(uint8_t direction, uint8_t speed); // Used to move the in a certain direction at a constant speed.
    void move(uint8_t direction, uint8_t speed, uint32_t rotation, uint8_t brake); // This will stop automatically when the motor reaches a certain position.
    void stop(void); // Used to stop the motor rapidly.
    void coast(void); // Used to coast the motor.
    int32_t readPosition(void); // Read the encoder position.
    void resetPosition(void); // Reset the encoder position.
    bool isTurning(void); // Check if the wheel is turning.

    /* Not normally used by the user - these are used by the interrupt routine */
    bool readEncodersEqual(void); // Used by the interrupt routines.
    volatile bool block, forward, brake;
    volatile int32_t position;

private:
    volatile bool turning;
    uint8_t pwmPin, tach1Pin;

    /*
     * These are used to read and write to the port registers - see http://www.arduino.cc/en/Reference/PortManipulation
     * I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/
     */
    uint8_t tach1BitMask, tach2BitMask;
    uint8_t logic1BitMask, logic2BitMask;
    volatile uint8_t *tach1InputPort, *tach2InputPort;
    volatile uint8_t *logic1OutPort, *logic2OutPort;
};

extern Motor Motor1, Motor2;

#endif