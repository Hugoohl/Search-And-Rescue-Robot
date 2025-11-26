#pragma once 
#include <QTRSensors.h>
#include <Arduino.h>
#include <PID_v1.h>
#include "MotorDriver.h"

class LineSensor{
    public:
     LineSensor();
     void begin(uint8_t arrayPin1, uint8_t arrayPin2, uint8_t arrayPin3, uint8_t arrayPin4, uint8_t singlePin, MotorDriver motor);
     void calibrate();
     void followLine();
     String checkJunction();

    private:
    QTRSensors qtr;
    uint16_t _sensors[4];
    int16_t _position;
    int16_t _lSpeed;
    int16_t _rSpeed;
    PID pid;
    double _output;
    double _input;
    double _target;
  



};

