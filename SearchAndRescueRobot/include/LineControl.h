#pragma once
#include <QTRSensors.h>
#include <Arduino.h>
#include <PID_v1.h>
#include "Types.h"
#include "Config.h"

class LineControl {
public:
    LineControl();

    void begin(const uint8_t arrayPins[], const uint8_t singlePin[]);
    void calibrateStep();
    void calibrate();

    // PID line follow
    void computeSpeedsPid(int &leftSpeed, int &rightSpeed);

    // simple fallback
    void computeSpeeds(int &leftSpeed, int &rightSpeed);

    // sensors
    void readSingle();
    uint16_t getLine();
    uint16_t getSingle();
    uint16_t getArraySensorValues(int i);

    // classification
    JunctionType detectJunction();      // robust-ish pattern classifier
    bool isDeadend();                   // based on all sensors
    bool isLineLost();                  // no sensor sees line

    String junctionTypeToString(JunctionType type);

private:
    QTRSensors qtr;
    QTRSensors qtrSingle;
    uint16_t _arraySensors[4]{};
    uint16_t _singleSensor[1]{};

    double _input = 0;
    double _output = 0;
    double _setpoint = 0;
    PID pid;

    float _posFiltered = CENTER_POS;

    void readArrayCalibrated();
};
