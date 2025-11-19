#pragma once 
#include <QTRSensors.h>
#include <Arduino.h>
#include <PID_v1.h>

class LineSensor{
    public:
     LineSensor();
     void begin(uint8_t arrayPin1, uint8_t arrayPin2, uint8_t arrayPin3, uint8_t arrayPin4, uint8_t singlePin);
     void calibrate();
     int followLine();
     String checkJunction();

    private:
    QTRSensors qtr;
    uint16_t _sensors[4];
    int16_t _position;
    PID pid;
    double _output;
    double _input;
    double _target;
  



};

