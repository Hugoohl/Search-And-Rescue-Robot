#include "LineControl.h"

LineControl::LineControl()
    : pid(&_input, &_output, &_setpoint, IR_KP, IR_KI, IR_KD, DIRECT) {}

void LineControl::begin(const uint8_t arrayPins[], const uint8_t singlePin[])
{
  qtr.setTypeAnalog();
  qtr.setSensorPins(arrayPins, 4);

  qtrSingle.setTypeAnalog();
  qtrSingle.setSensorPins(singlePin, 1);

  _setpoint = CENTER_POS;
  _posFiltered = CENTER_POS;

  pid.SetOutputLimits(-PID_MAX_SPEED, PID_MAX_SPEED);
  pid.SetMode(AUTOMATIC);
}

void LineControl::calibrateStep()
{
  qtr.calibrate();
  qtrSingle.calibrate();
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

void LineControl::readArrayCalibrated()
{
  qtr.readCalibrated(_arraySensors);
}

void LineControl::readSingle()
{
  qtrSingle.readCalibrated(_singleSensor);
}



uint16_t LineControl::getSingle()
{
  readSingle();
  return _singleSensor[0];
}

uint16_t LineControl::getArraySensorValues(int i)
{
  return _arraySensors[i];
}

uint16_t LineControl::getLine(){
  uint16_t pos = qtr.readLineBlack(_arraySensors);
  return pos;
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
  else
  {
    leftSpeed = DC_MOTOR_BASE_SPEED;
    rightSpeed = DC_MOTOR_TURN_SPEED;
  }
}

void LineControl::computeSpeedsPid(int &leftSpeed, int &rightSpeed)
{
  uint16_t pos = qtr.readLineBlack(_arraySensors);

  _posFiltered = (1.0f - LINE_POS_ALPHA) * _posFiltered + LINE_POS_ALPHA * (float)pos;

  // raw error in "QTR position units" (~ -1500..+1500)
  double error = (double)_posFiltered - (double)CENTER_POS;

  _input = error;
  _setpoint = 0;
  pid.Compute();

  int correction = (int)_output;

  int l = DC_MOTOR_BASE_SPEED + correction;
  int r = DC_MOTOR_BASE_SPEED - correction;

  l = constrain(l, 40, DC_MOTOR_MAX_SPEED);
  r = constrain(r, 40, DC_MOTOR_MAX_SPEED);

  leftSpeed = l;
  rightSpeed = r;
}

bool LineControl::isLineLost()
{
  readArrayCalibrated();
  readSingle();

  bool any = false;
  for (int i = 0; i < 4; i++)
  {
    if (_arraySensors[i] > JUNCTION_THRESHOLD)
    {
      any = true;
      break;
    }
  }
  if (_singleSensor[0] > JUNCTION_THRESHOLD_SINGLE)
    any = true;

  return !any;
}

bool LineControl::isDeadend()
{
  // Dead-end in your use-case: line effectively gone under all sensors
  return isLineLost();
}

JunctionType LineControl::detectJunction()
{
  readSingle();
  readArrayCalibrated();

  bool s[4];
  for (int i = 0; i < 4; i++)
    s[i] = _arraySensors[i] > JUNCTION_THRESHOLD;
  bool single = _singleSensor[0] > JUNCTION_THRESHOLD_SINGLE;

  int count = (int)s[0] + (int)s[1] + (int)s[2] + (int)s[3];

  // If nothing sees line -> dead end / lost
  if (count == 0 && !single)
    return JunctionType::DEAD_END;

  // Strong intersection/cross (often: all or almost all see black)
  if (count >= 4 && single)
    return JunctionType::CROSS;

  // T: array sees wide black but single does not
  if (count >= 4 && !single)
    return JunctionType::T;

  // Left/Right T patterns (looser than before; better for angled entry)
  if (!s[0] && s[1] && s[2] && s[3] && single)
    return JunctionType::LEFT_T;
  if (s[0] && s[1] && s[2] && !s[3] && single)
    return JunctionType::RIGHT_T;

  if ((s[0] || s[1]) && !(s[2] || s[3]) && !single)
    return JunctionType::LEFT;
  if ((s[2] || s[3]) && !(s[0] || s[1]) && !single)
    return JunctionType::RIGHT;

  // “Generic junction” if many sensors see black
  if (count >= 3 || single)
    return JunctionType::JUNCTION;

  return JunctionType::NONE;
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
