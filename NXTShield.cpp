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

#include "NXTShield.h"

const uint8_t Sensor::clockPin = 4;

/* Ultrasonic Sensor */
Sensor::Sensor() {
    const double SCL_Frequency = 11494.252873563; // The same I2C frequency as the NXT
    const uint8_t prescaler = 4;

    Wire.begin();

    /* Set I2C frequency to 11494,253Hz - the same as the NXT */
    TWSR |= _BV(TWPS0); // Set prescaler to 4
    TWSR &= ~(_BV(TWPS1));

    // SCL Frequency = F_CPU/(16+2*TWBR*Prescaler) - see p. 223 in ATmega328P's datasheet
    TWBR = ((F_CPU/SCL_Frequency)-16)/prescaler/2; // Bit Rate Register = 172 for 16MHz and 85 for 8MHz
    pinMode(clockPin, INPUT_PULLUP); // Needed for receiving to work
    timer = millis();
}
int16_t Sensor::readDistance(void) {
    uint8_t data;
    if (readCommand(readMeasurement, &data, 1))
        return -1;
    return data;
}
uint8_t Sensor::readCommand(uint8_t address, uint8_t *data, uint8_t length) {
    const uint16_t I2C_TIMEOUT = 10; // Used to check for errors in I2C communication
    uint32_t timeOutTimer;
    uint8_t buffer[max(9, length)]; // We need to request at least 9 in order for it to work

    if ((millis() - timer) < 25) // There has to be approx. 25 ms between commands
        delay(25 - (millis() - timer));
    timer = millis();

    Wire.beginTransmission(sensorAddress); // Transmit to device
    Wire.write(address);
    uint8_t rcode = Wire.endTransmission(); // Don't release the bus
    if (rcode) // Check error code
        return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission

    clockPulse(); // Needed for receiving to work

    Wire.requestFrom((uint8_t)sensorAddress, sizeof(buffer)); // Send repeated start, request 9 and send stop command

    for (uint8_t i = 0; i < sizeof(buffer); i++) {
        if (Wire.available())
            buffer[i] = Wire.read();
        else {
            timeOutTimer = millis();
            while (((millis() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
            if (Wire.available())
                buffer[i] = Wire.read();
            else
                return 5; // This error value is not already taken by endTransmission
        }
    }
    for (uint8_t i = 0; i < length; i++)
        data[i] = buffer[i];
    return 0;
}
uint8_t Sensor::setCommand(uint8_t address, uint8_t command) {
    if ((millis() - timer) < 25) // There has to be approx. 25 ms between commands
        delay(25 - (millis() - timer));
    timer = millis();

    Wire.beginTransmission(sensorAddress); // Transmit to device
    Wire.write(address);
    Wire.write(command);
    return Wire.endTransmission(); // See: http://arduino.cc/en/Reference/WireEndTransmission
}
void Sensor::clockPulse(void) {
    delayMicroseconds(60);
    pinMode(clockPin, OUTPUT);
    digitalWrite(clockPin, LOW);
    delayMicroseconds(34);
    pinMode(clockPin, INPUT_PULLUP);
    delayMicroseconds(60);
}
/*
 ' Wires on NXT jack plug.
 ' Wire colors may vary. Pin 1 is always nearest latch.
 ' 1 White +9V
 ' 2 Black GND
 ' 3 Red GND
 ' 4 Green +5V
 ' 5 Yellow SCL - also connect clockpin to give a extra low impulse
 ' 6 Blue SDA
 ' Do not use I2C pullup resistor - already provided within sensor.
 */

/* Variables used by the interrupt routine for the encoders */
volatile int32_t firstTachPos, secondTachPos;

// The interrupt routines handles the encoder reading
void firstEncoder() {
    if (Motor1.readEncodersEqual()) // pin 2 == pin 5 - See http://www.arduino.cc/en/Reference/PortManipulation
        firstTachPos++;
    else
        firstTachPos--;

    if (Motor1.block && Motor1.isTurning()) { // Check if the encoders should stop the motor
        if ((Motor1.forward && firstTachPos <= Motor1.position) || (!Motor1.forward && firstTachPos >= Motor1.position)) {
            if (Motor1.brake)
                Motor1.stop();
            else
                Motor1.coast();
        }
    }
}

void secondEncoder() {
    if (Motor2.readEncodersEqual()) // pin 3 == pin 6 - See http://www.arduino.cc/en/Reference/PortManipulation
        secondTachPos++;
    else
        secondTachPos--;

    if (Motor2.block && Motor2.isTurning()) { // Check if the encoders should stop the motor
        if ((Motor2.forward && secondTachPos <= Motor2.position) || (!Motor2.forward && secondTachPos >= Motor2.position)) {
            if (Motor2.brake)
                Motor2.stop();
            else
                Motor2.coast();
        }
    }
}

/* Motor class */
Motor::Motor(uint8_t tach1, uint8_t tach2, uint8_t pwm, uint8_t logic1, uint8_t logic2) {
    tach1Pin = tach1;
    pinMode(tach1Pin, INPUT);
    pinMode(tach2, INPUT);

    pwmPin = pwm;
    pinMode(pwmPin, OUTPUT);
    pinMode(logic1, OUTPUT);
    pinMode(logic2, OUTPUT);

    /* Read the port register and bit mask for the different pins */
    tach1BitMask = digitalPinToBitMask(tach1Pin);
    tach1InputPort = portInputRegister(digitalPinToPort(tach1Pin));

    tach2BitMask = digitalPinToBitMask(tach2);
    tach2InputPort = portInputRegister(digitalPinToPort(tach2));

    logic1BitMask = digitalPinToBitMask(logic1);
    logic1OutPort = portOutputRegister(digitalPinToPort(logic1));

    logic2BitMask = digitalPinToBitMask(logic2);
    logic2OutPort = portOutputRegister(digitalPinToPort(logic2));

    if (tach1Pin == 2) { // Pin 2
#ifdef digitalPinToInterrupt // Use macro defined in pins_arduino.h
        attachInterrupt(digitalPinToInterrupt(tach1Pin), firstEncoder, CHANGE); // Pin 2
#elif defined(__AVR_ATmega32U4__) // Arduino Leonardo
        attachInterrupt(1, firstEncoder, CHANGE); // pin 2
#else
        attachInterrupt(0, firstEncoder, CHANGE); // pin 2
#endif
    } else if (tach1Pin == 3) { // Pin 3
#ifdef digitalPinToInterrupt // Use macro defined in pins_arduino.h
        attachInterrupt(digitalPinToInterrupt(tach1Pin), secondEncoder, CHANGE); // pin 3
#elif defined(__AVR_ATmega32U4__) // Arduino Leonardo
        attachInterrupt(0, secondEncoder, CHANGE); // pin 3
#else
        attachInterrupt(1, secondEncoder, CHANGE); // pin 3
#endif
    }
}

void Motor::move(uint8_t direction, uint8_t speed) {
    turning = true;
    block = false;

    analogWrite(pwmPin, speed);

    if (direction == FORWARD) {
        *logic1OutPort |= logic1BitMask;
        *logic2OutPort &= ~logic2BitMask;
    } else if (direction == BACKWARD) {
        *logic1OutPort &= ~logic1BitMask;
        *logic2OutPort |= logic2BitMask;
    }
}
void Motor::move(uint8_t direction, uint8_t speed, uint32_t rotation, uint8_t halt) {
    turning = true;
    block = true;
    if (halt == BRAKE)
        brake = true;
    else if (halt == COAST)
        brake = false;

    analogWrite(pwmPin, speed);

    int32_t tachPos;
    if (tach1Pin == 2)
        tachPos = firstTachPos;
    else
        tachPos = secondTachPos;

    if (direction == FORWARD) {
        forward = true;
        position = tachPos-rotation;
        *logic1OutPort |= logic1BitMask;
        *logic2OutPort &= ~logic2BitMask;
    } else if (direction == BACKWARD) {
        forward = false;
        position = tachPos+rotation;
        *logic1OutPort &= ~logic1BitMask;
        *logic2OutPort |= logic2BitMask;
    }
}
void Motor::stop(void) {
    turning = false;
    analogWrite(pwmPin, 255);
    *logic1OutPort |= logic1BitMask;
    *logic2OutPort |= logic2BitMask;
}
void Motor::coast(void) {
    turning = false;
    analogWrite(pwmPin, 0);
}
int32_t Motor::readPosition(void) {
    if (tach1Pin == 2)
        return firstTachPos;
    return secondTachPos;
}
void Motor::resetPosition(void) {
    if (tach1Pin == 2)
        firstTachPos = 0;
    else
        secondTachPos = 0;
}
bool Motor::isTurning(void) {
    return turning;
}
bool Motor::readEncodersEqual(void) {
    return ((bool)(*tach1InputPort & tach1BitMask) == (bool)(*tach2InputPort & tach2BitMask));
}

Motor Motor1(2, 5, 9, 7, 11); // Create the first motor instance
Motor Motor2(3, 6, 10, 12, 8); // Create the second motor instance

Sensor UltrasonicSensor; // Create the Ultrasonic Sensor instance