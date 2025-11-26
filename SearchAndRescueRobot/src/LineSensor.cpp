#include "LineSensor.h"
#include "Config.h"
#include <Arduino.h>

LineSensor::LineSensor() : pid(&_input, &_output, &_target, IR_KP, IR_KI, IR_KD, DIRECT) {}

void LineSensor::begin(uint8_t arrayPin1, uint8_t arrayPin2, uint8_t arrayPin3, uint8_t arrayPin4, uint8_t singlePin, MotorDriver motor)
{
  qtr.setTypeAnalog();
  static const uint8_t pins[4] = {
      arrayPin1,
      arrayPin2,
      arrayPin3,
      arrayPin4};
  qtr.setSensorPins(pins, 4);
  pid.SetOutputLimits(-DC_MOTOR_MAX_SPEED, DC_MOTOR_MAX_SPEED);
  pid.SetMode(AUTOMATIC);
}

void LineSensor::calibrate()
{ // Calibrate, move sensors over the extremes for 5 seconds.
  for (uint8_t i = 0; i < 250; i++)
  {
    qtr.calibrate();
    delay(20);
  }
}

void LineSensor::followLine()
{
  qtr.readCalibrated(_sensors);
  _position = qtr.readLineBlack(_sensors);
  if (_position > 1000 && _position < 2500)
  {
    _lSpeed, _rSpeed = 50;
  }
  else if (_position < 1000)
  {
    _lSpeed = 50; _rSpeed = 15;
  }
  else if (_position > 2000)
  {
    _lSpeed = 15, _rSpeed = 50;
  }

  motor.drive(_lSpeed, _rSpeed);
}