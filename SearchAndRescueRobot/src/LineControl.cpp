#include "LineControl.h"

LineControl::LineControl() : pid(&_input, &_output, &_setpoint, IR_KP, IR_KI, IR_KD, DIRECT) {}

void LineControl::begin(const uint8_t arrayPins[], const uint8_t singlePin)
{

  qtr.setTypeAnalog();
  qtr.setSensorPins(arrayPins, 4);

  _singlePin = singlePin;

  pinMode(_singlePin, INPUT);

  _setpoint = CENTER_POS;
  pid.SetOutputLimits(-DC_MOTOR_MAX_SPEED, DC_MOTOR_MAX_SPEED);
  pid.SetMode(AUTOMATIC);
}

void LineControl::calibrate()
{

  for (uint16_t i = 0; i < CALIBRATION_TIME / 20; i++)
  {
    qtr.calibrate();
    delay(20);
  }
}

void LineControl::computeSpeeds(int &leftSpeed, int &rightSpeed)
{
  uint16_t pos = qtr.readLineBlack(_arraySensors);

  if (pos > 1000 && pos < 2500)
  {
    leftSpeed =DC_MOTOR_BASE_SPEED;
    rightSpeed =DC_MOTOR_BASE_SPEED;
  }
  else if (pos < 1000)
  {
    leftSpeed = DC_MOTOR_TURN_SPEED;
    rightSpeed =DC_MOTOR_BASE_SPEED;
  }
  else if (pos > 2000)
  {
    leftSpeed =DC_MOTOR_BASE_SPEED;
    rightSpeed = DC_MOTOR_TURN_SPEED;
  }
}

void LineControl::computeSpeedsPid(int &leftSpeed, int &rightSpeed)
{
  uint16_t pos = qtr.readLineBlack(_arraySensors);

  _input = pos;

  pid.Compute();

  int correction = static_cast<int>(_output);

  leftSpeed = DC_MOTOR_BASE_SPEED + correction;
  rightSpeed = DC_MOTOR_BASE_SPEED - correction;

  leftSpeed = constrain(leftSpeed, -DC_MOTOR_MAX_SPEED, DC_MOTOR_MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -DC_MOTOR_MAX_SPEED, DC_MOTOR_MAX_SPEED);
}

bool LineControl::isLineLost()
{
  return true;
}

uint8_t LineControl::getSingle(){
  readSingle();
  return _singleSensor;
}

void LineControl::readSingle()
{
  _singleSensor = analogRead(_singlePin);
}

JunctionType LineControl::detectJunction()
{
  readSingle();
  qtr.readCalibrated(_arraySensors);
  bool s[4];
  for (int i = 0; i < 4; i++)
  {
    s[i] = _arraySensors[i] > JUNCTION_THRESHOLD;
  }

  bool single = _singleSensor > JUNCTION_THRESHOLD;

  if (s[0] && s[1] && s[2] && s[3] && single)
  {
    return JunctionType::CROSS;
  }
  if (!s[0] && s[1] && s[2] && s[3] && single)
  {
    return JunctionType::LEFT_T;
  }
  if (s[0] && s[1] && s[2] && !s[3] && single)
  {
    return JunctionType::RIGHT_T;
  }
  if (s[0] && s[1] && s[2] && s[3] && !single)
  {
    return JunctionType::T;
  }
  if (!s[0] &&  !s[3] && !single)
  {
    return JunctionType::DEAD_END;
  }

  return JunctionType::NONE;
}

String LineControl::junctionTypeToString(JunctionType type)
{
  switch (type)
  {
  case JunctionType::NONE:
    return "none";
  case JunctionType::LEFT_T:
    return "leftT";
  case JunctionType::RIGHT_T:
    return "rightT";
  case JunctionType::T:
    return "T";
  case JunctionType::CROSS:
    return "cross";
  case JunctionType::DEAD_END:
    return "deadend";
  default:
    return "unknown";
  }
}

uint16_t LineControl::getArraySensorValues(int i){
  
 return _arraySensors[i];
  
}


// void LineControl::followLine()
// {
//   qtr.readCalibrated(_sensors);
//   _position = qtr.readLineBlack(_sensors);
//   if (_position > 1000 && _position < 2500)
//   {
//     _lSpeed, _rSpeed =DC_MOTOR_BASE_SPEED;
//   }
//   else if (_position < 1000)
//   {
//     _lSpeed =DC_MOTOR_BASE_SPEED; _rSpeed = 15;
//   }
//   else if (_position > 2000)
//   {
//     _lSpeed = 15, _rSpeed =DC_MOTOR_BASE_SPEED;
//   }

//   motor.drive(_lSpeed, _rSpeed);
// }