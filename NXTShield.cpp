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

/* Ultrasonic Sensor */
UltrasonicSensor::UltrasonicSensor() {    
    Wire.begin();
    
    /* Set I2C frequency to 11494,253Hz - the same as the NXT */    
    TWSR |= _BV(TWPS0); // Set prescaler to 4  
    TWSR &= ~(_BV(TWPS1));
    
    // SCL Frequency = F_CPU/(16+2*TWBR*Prescaler) - see p. 223 in atmega328's datasheet
    TWBR = ((F_CPU/SCL_Frequency)-16)/prescaler/2; // Bit Rate Register = 172 for 16MHz and 85 for 8MHz  
    pinMode(clockPin, INPUT); // Needed for receiving to work
    digitalWrite(clockPin, HIGH);    
    _timer = millis();
}
int UltrasonicSensor::readDistance(void) {
    int* data = readCommand(readMeasurement,1);
    return data[0];
}
int* UltrasonicSensor::readCommand(int address, int length) {
    if((millis() - _timer) < 25) // There has to be approx. 25 ms between commands
        delay(25 - (millis() - _timer));
    _timer = millis();
    
    Wire.beginTransmission(sensorAddress); // transmit to device
    Wire.write(address);
    Wire.endTransmission();
    
    clockPulse(); // Needed for receiving to work
    
    Wire.requestFrom(sensorAddress,9); // Send repeated start, request 9 and send stop command
    int buffer[9];
    for(uint8_t i = 0; i < 9; i++) // This small hack is needed for it to work properly
        buffer[i] = Wire.read();
    Wire.endTransmission();
    
    int newBuf[length];
    for(uint8_t i = 0; i < length; i++)
        newBuf[i] = buffer[i];
    
    return newBuf;
}
void UltrasonicSensor::setCommand(int address,int command) {
    if((millis() - _timer) < 25) // There has to be approx. 25 ms between commands
        delay(25 - (millis() - _timer));
    _timer = millis();
    
    Wire.beginTransmission(sensorAddress); // transmit to device
    Wire.write(address);
    Wire.write(command);
    Wire.endTransmission();    
}
void UltrasonicSensor::clockPulse(void) {
    delayMicroseconds(60);
    pinMode(clockPin, OUTPUT);
    digitalWrite(clockPin, LOW); 
    delayMicroseconds(34);
    pinMode(clockPin, INPUT);
    digitalWrite(clockPin, HIGH); 
    delayMicroseconds(60);    
}
/*
 ' Wires on NXT jack plug.
 ' Wire colours may vary. Pin 1 is always nearest latch.
 ' 1 White +9V
 ' 2 Black GND
 ' 3 Red GND
 ' 4 Green +5V
 ' 5 Yellow SCL - also connect clockpin to give a extra low impuls
 ' 6 Blue SDA
 ' Do not use i2c pullup resistor - already provided within sensor.
 */

/* Interrupt code for the motors */
volatile signed long _firstTachPos = 0;
volatile signed long _secondTachPos = 0;

bool _isTurning1;
bool block1;
bool forward1;
bool brake1;
long position1;

bool _isTurning2;
bool block2;
bool forward2;
bool brake2;
long position2;

// Read using the registers, as this is faster
void firstEncoder() { 
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    if((bool)(PINE & _BV(4)) == (bool)(PINE & _BV(3))) // pin 2 == pin 5 - See http://arduino.cc/en/Hacking/PinMapping2560
        _firstTachPos++;
    else
        _firstTachPos--;
    #else
    if((bool)(PIND & _BV(2)) == (bool)(PIND & _BV(5))) // pin 2 == pin 5 - See http://arduino.cc/en/Hacking/PinMapping168
        _firstTachPos++;
    else
        _firstTachPos--;
    #endif
    
    if (block1) { // This will ensure that both motors can move at the same time
        if(forward1) {
            if(_firstTachPos >= position1) {
                _isTurning1 = false;
                if(brake1) {
                    digitalWrite(firstPWM, HIGH);
                    digitalWrite(logicFirst1, HIGH);
                    digitalWrite(logicFirst2, HIGH);
                }                
                else
                    digitalWrite(firstPWM, LOW);
            }                
        } else {
            if(_firstTachPos <= position1) {
                _isTurning1 = false;
                if(brake1) {
                    digitalWrite(firstPWM, HIGH);
                    digitalWrite(logicFirst1, HIGH);
                    digitalWrite(logicFirst2, HIGH);
                }                
                else
                    digitalWrite(firstPWM, LOW);
            } 
        }
    }
}
void secondEncoder() { 
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    if((bool)(PINE & _BV(5)) == (bool)(PINH & _BV(3))) // pin 3 == pin 6 - See http://arduino.cc/en/Hacking/PinMapping2560
        _secondTachPos++;
    else
        _secondTachPos--;
    #else
    if((bool)(PIND & _BV(3)) == (bool)(PIND & _BV(6))) // pin 3 == pin 6 - See http://arduino.cc/en/Hacking/PinMapping168
        _secondTachPos++;
    else
        _secondTachPos--;
    #endif
    
    if (block2) { // This will ensure that both motors can move at the same time
        if(forward2) {
            if(_secondTachPos >= position2) {
                _isTurning2 = false;                
                if(brake2) {
                    digitalWrite(secondPWM, HIGH);
                    digitalWrite(logicSecond1, HIGH);
                    digitalWrite(logicSecond2, HIGH);
                }                
                else
                    digitalWrite(secondPWM, LOW);
            }                
        } else {
            if(_secondTachPos <= position2) {
                _isTurning2 = false;
                if(brake2) {
                    digitalWrite(secondPWM, HIGH);
                    digitalWrite(logicSecond1, HIGH);
                    digitalWrite(logicSecond2, HIGH);
                }                
                else
                    digitalWrite(secondPWM, LOW);
            } 
        }
    }
}

/* First Motor */
Motor1::Motor1() {
    pinMode(tachFirst1, INPUT);
    pinMode(tachFirst2, INPUT);
    
    pinMode(firstPWM, OUTPUT);
    pinMode(logicFirst1, OUTPUT);
    pinMode(logicFirst1, OUTPUT);
    
    attachInterrupt(0, firstEncoder, CHANGE); // pin 2    
}
void Motor1::move(Direction direction, uint8_t torque) {
    _isTurning1 = true;    
    block1 = false;
    
    analogWrite(firstPWM, torque);    
    if(direction == forward) {
        digitalWrite(logicFirst1, LOW);
        digitalWrite(logicFirst2, HIGH);
    } else if(direction == backward) {
        digitalWrite(logicFirst1, HIGH);
        digitalWrite(logicFirst2, LOW);
    }
}
void Motor1::move(Direction direction, uint8_t torque, int rotation, Brake halt) {
    _isTurning1 = true;    
    block1 = true;
    if(halt == 0)
        brake1 = true;
    else if(halt == 1)
        brake1 = false;
    
    analogWrite(firstPWM, torque);    
    if(direction == forward) {
        forward1 = true;
        position1 = _firstTachPos+rotation;  
        digitalWrite(logicFirst1, LOW);
        digitalWrite(logicFirst2, HIGH);
    } else if(direction == backward) { 
        forward1 = false;        
        position1 = _firstTachPos-rotation;
        digitalWrite(logicFirst1, HIGH);
        digitalWrite(logicFirst2, LOW);   
    }     
}
void Motor1::stop(void) {
    _isTurning1 = false;    
    digitalWrite(firstPWM, HIGH);
    digitalWrite(logicFirst1, HIGH);
    digitalWrite(logicFirst2, HIGH);
}
void Motor1::coast(void) {
    _isTurning1 = false;    
    digitalWrite(firstPWM, LOW);
}
long Motor1::readPosition(void) {
    return _firstTachPos;        
}
void Motor1::resetPosition(void) {
    _firstTachPos = 0;
}
bool Motor1::isTurning(void) {
    return _isTurning1;
}

/* Second Motor */
Motor2::Motor2() {
    pinMode(tachSecond1, INPUT);
    pinMode(tachSecond2, INPUT);
    
    pinMode(secondPWM, OUTPUT);
    pinMode(logicSecond1, OUTPUT);
    pinMode(logicSecond2, OUTPUT);
    
    attachInterrupt(1, secondEncoder, CHANGE); // pin 3
}
void Motor2::move(Direction direction, uint8_t torque) {
    _isTurning2 = true;        
    block2 = false;    
    
    analogWrite(secondPWM, torque);    
    if(direction == forward) {
        digitalWrite(logicSecond1, LOW);
        digitalWrite(logicSecond2, HIGH);
    } else if(direction == backward) {
        digitalWrite(logicSecond1, HIGH);
        digitalWrite(logicSecond2, LOW);
    }
}
void Motor2::move(Direction direction, uint8_t torque, int rotation, Brake halt) {
    _isTurning2 = true;
    block2 = true;
    if(halt == 0)
        brake2 = true;
    else if(halt == 1)
        brake2 = false;
    
    analogWrite(secondPWM, torque);
    if(direction == forward) {
        forward2 = true;
        position2 = _secondTachPos+rotation;  
        digitalWrite(logicSecond1, LOW);
        digitalWrite(logicSecond2, HIGH);
    } else if(direction == backward) {  
        forward2 = false;
        position2 = _secondTachPos-rotation;
        digitalWrite(logicSecond1, HIGH);
        digitalWrite(logicSecond2, LOW);    
    }    
}
void Motor2::stop(void) {
    _isTurning2 = false;
    digitalWrite(secondPWM, HIGH);
    digitalWrite(logicSecond1, HIGH);
    digitalWrite(logicSecond2, HIGH);
}
void Motor2::coast(void) {
    _isTurning2 = false;
    digitalWrite(secondPWM, LOW);
}
long Motor2::readPosition(void) {
    return _secondTachPos;        
}
void Motor2::resetPosition(void) {
    _secondTachPos = 0;   
}
bool Motor2::isTurning(void) {
    return _isTurning2;
}