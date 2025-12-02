#pragma once
#include <QTRSensors.h>
#include <Arduino.h>
#include <PID_v1.h>
#include "Types.h"
#include "Config.h"

class LineControl
{
public:
    LineControl();
    void begin(const uint8_t arrayPins[], const uint8_t singlePin);
    void calibrate();
    void computeSpeedsPid(int &leftSpeed, int &rightSpeed);
    void computeSpeeds(int &leftSpeed, int &rightSpeed);
    bool isLineLost();
    void readSingle();
    uint8_t getSingle();
    uint16_t getArraySensorValues(int i);
    String junctionTypeToString(JunctionType type);
    JunctionType detectJunction();

private:
    QTRSensors qtr;
    uint16_t _arraySensors[4];
    uint16_t _singleSensor;
    uint8_t _singlePin;

    double _input = 0;
    double _output = 0;
    double _setpoint = 0;
    PID pid;

   

    
};
