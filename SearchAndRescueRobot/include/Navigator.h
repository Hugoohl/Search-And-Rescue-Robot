#pragma once
#include <Arduino.h>
#include "Types.h"
#include "Pins.h"
#include "Config.h"
#include "MotorDriver.h"
#include "LineControl.h"




class Navigator {
public:
    Navigator(LineControl &line, MotorDriver &motor);
    void begin();
    void update();

private:
    RobotState _state = RobotState::WAIT_FOR_START;
    MissionState _phase = MissionState::SEARCH_LEFT;

    LineControl   &_line;
    MotorDriver   &_motor;


    unsigned long _missionStartTime = 0;

    void handleWaitForStart();
    void handleCalibrating();
    void handleRunMission();
    void handleMissionDone();

    bool missionTimeExceeded() const;
};