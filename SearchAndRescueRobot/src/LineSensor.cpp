#include "LineSensor.h"
#include "Config.h"
#include <Arduino.h>


LineSensor::LineSensor(): pid(&_input, &_output, &_target, IR_KP, IR_KI, IR_KD, DIRECT) {}

void LineSensor::begin(uint8_t arrayPin1, uint8_t arrayPin2, uint8_t arrayPin3, uint8_t arrayPin4, uint8_t singlePin){
    qtr.setTypeAnalog();
     static const uint8_t pins[4] = {
        arrayPin1,
        arrayPin2,
        arrayPin3,
        arrayPin4
     };
    qtr.setSensorPins(pins, 4);
    pid.SetOutputLimits(-DC_MOTOR_MAX_SPEED, DC_MOTOR_MAX_SPEED);
    pid.SetMode(AUTOMATIC);

}

void LineSensor::calibrate(){       //Calibrate, move sensors over the extremes for 5 seconds.
 for (uint8_t i = 0; i < 250; i++){
    qtr.calibrate();
    delay(20);
  }
}

int LineSensor::followLine(){
    qtr.readCalibrated(_sensors);
    _position = qtr.readLineBlack(_sensors);
    

    // _input = static_cast<double> (_position);
    // _target = 1500;
    // pid.Compute();


    return _position;
}