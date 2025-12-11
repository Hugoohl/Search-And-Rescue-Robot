#include "LineControl.h"

LineControl::LineControl() : pid(&_input, &_output, &_setpoint, IR_KP, IR_KI, IR_KD, DIRECT) {}

void LineControl::begin(const uint8_t arrayPins[], const uint8_t singlePin[])
{

  qtr.setTypeAnalog();
  qtr.setSensorPins(arrayPins, 4);
  qtrSingle.setTypeAnalog();
  qtrSingle.setSensorPins(singlePin, 1);

 

  _setpoint = CENTER_POS;
  pid.SetOutputLimits(-PID_MAX_SPEED, PID_MAX_SPEED);
  pid.SetMode(AUTOMATIC);
}

void LineControl::calibrateStep()
{
    qtr.calibrate();
}

void LineControl::calibrate()
{

  for (uint16_t i = 0; i < CALIBRATION_TIME / 20; i++)
  {
    qtr.calibrate();
    qtrSingle.calibrate();
    delay(20);
  }
}

void LineControl::computeSpeeds(int &leftSpeed, int &rightSpeed)
{
  uint16_t pos = qtr.readLineBlack(_arraySensors);

  if (pos > 1000 && pos < 2500)
  {
    leftSpeed = DC_MOTOR_BASE_SPEED;
    rightSpeed = DC_MOTOR_BASE_SPEED;
  }
  else if (pos < 1000)
  {
    leftSpeed = DC_MOTOR_TURN_SPEED;
    rightSpeed = DC_MOTOR_BASE_SPEED;
  }
  else if (pos > 2000)
  {
    leftSpeed = DC_MOTOR_BASE_SPEED;
    rightSpeed = DC_MOTOR_TURN_SPEED;
  }
}

void LineControl::computeSpeedsPid(int &leftSpeed, int &rightSpeed)
{
  uint16_t pos = qtr.readLineBlack(_arraySensors);
  _input = pos;

  pid.Compute();

  int correction = (int)_output; // e.g. ±200

  // no need to clamp to base, allow reverse
  correction = constrain(correction, -PID_MAX_SPEED, PID_MAX_SPEED);

  leftSpeed = DC_MOTOR_BASE_SPEED + correction;
  rightSpeed = DC_MOTOR_BASE_SPEED - correction;

  // these allow reverse (negative values)
  leftSpeed = constrain(leftSpeed, -DC_MOTOR_MAX_SPEED, DC_MOTOR_MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -DC_MOTOR_MAX_SPEED, DC_MOTOR_MAX_SPEED);
}

bool LineControl::isLineLost()
{
    qtr.readCalibrated(_arraySensors);

    bool anyOnLine = false;
    for (int i = 0; i < 4; ++i) {
        if (_arraySensors[i] > JUNCTION_THRESHOLD) { // high = black
            anyOnLine = true;
            break;
        }
    }
    return !anyOnLine; 
}

uint16_t LineControl::getSingle()
{
  readSingle();
  return _singleSensor[0];
}

bool LineControl::isOnLine(){
  int pos = qtr.readLineBlack(_arraySensors);
  if (pos >1000 && pos <2000){
    return true;
  }
  return false;
}

void LineControl::readSingle()
{
  qtrSingle.readCalibrated(_singleSensor);
}

bool LineControl::isJunction()
{
  qtr.readCalibrated(_arraySensors);

  bool leftOuter = _arraySensors[0] > JUNCTION_THRESHOLD;
  bool rightOuter = _arraySensors[3] > JUNCTION_THRESHOLD;

  return (leftOuter || rightOuter);
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

  bool single = _singleSensor[0] > JUNCTION_THRESHOLD;

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
  if (!s[0] && !s[3] && !single)
  {
    return JunctionType::DEAD_END;
  }

  return JunctionType::NONE;
}

bool LineControl::isDeadend(){
  if(detectJunction() == JunctionType::DEAD_END){
    return true;
  }
  return false;
}

String LineControl::junctionTypeToString(JunctionType type)
{
  switch (type)
  {
  case JunctionType::NONE:
    return "none";
  case JunctionType::JUNCTION:
    return "junction";
  case JunctionType::LEFT_T:
    return "leftT";
  case JunctionType::RIGHT_T:
    return "rightT";
  case JunctionType::RIGHT:
    return "right";
  case JunctionType::LEFT:
    return "left";
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

uint16_t LineControl::getArraySensorValues(int i)
{

  return _arraySensors[i];
}

