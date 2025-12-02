#include "Navigator.h"

Navigator::Navigator(LineControl &line, MotorDriver &motor) : _line(line), _motor(motor) {}

void Navigator::begin()
{
    _state = RobotState::WAIT_FOR_START;
    _motor.stop();
}

void Navigator::update()
{
    switch (_state)
    {
    case RobotState::WAIT_FOR_START:
        handleWaitForStart();
        break;

    case RobotState::CALIBRATING:
        handleCalibrating();
        break;

    case RobotState::RUN_MISSION:
        handleRunMission();
        break;

    case RobotState::MISSION_DONE:
        handleMissionDone();
        break;
    }
}

void Navigator::handleWaitForStart()
{
    Serial.println("Calibration started");
    _state = RobotState::CALIBRATING;
}

void Navigator::handleCalibrating()
{
    _motor.stop();
    _line.calibrate();
    Serial.println("Calibration Done, starting mission");
    _state = RobotState::RUN_MISSION;
}

void Navigator::handleRunMission()
{
    _missionStartTime = millis();

    int leftSpeed = 0;
    int rightSpeed = 0;
    JunctionType type;

    _line.computeSpeeds(leftSpeed, rightSpeed);
    _motor.drive(leftSpeed, rightSpeed);
    type = _line.detectJunction();
    Serial.println(_line.junctionTypeToString(type));

    




}

void Navigator::handleMissionDone()
{
    _motor.stop();
}
