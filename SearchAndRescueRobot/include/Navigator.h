#pragma once
#include <Arduino.h>
#include "Types.h"
#include "Pins.h"
#include "Config.h"
#include "MotorDriver.h"
#include "LineControl.h"
#include "UltraSonic.h"
#include "Gripper.h"

class Navigator
{
public:
    Navigator(LineControl &line, MotorDriver &motor, UltraSonic &sonar, Gripper &serv);

    void begin();
    void update();

    void rotate180();
    void rotate90left();
    void rotate90right();

    void testLinePidAndTurns();
    bool detectCylinder();

    void followRightWall();
    void followLeftWall();



    // ------------------------
    // Robot global state
    // ------------------------
    RobotState _state = RobotState::WAIT_FOR_START;

    // ------------------------
    // Mission phase
    // ------------------------
    MissionState _phase = MissionState::WAIT_FOR_BUTTON;

    // Stores which search phase triggered pickup (needed to select return phase)
    MissionState _phaseBeforePickup = MissionState::WAIT_FOR_BUTTON;

    // Which return phase was completed (1,2,3)
    int _lastReturnPhase = 0;

    // Dependencies
    LineControl &_line;
    MotorDriver &_motor;
    UltraSonic &_sonar;
    Gripper &_serv;

    // Time tracking if needed
    unsigned long _missionStartTime = 0;

    // ------------------------
    // Core State Handlers
    // ------------------------
    void handleWaitForStart();
    void handleCalibrating();
    void handleRunMission();
    void handleMissionDone();

    // ------------------------
    // Mission Phase Handlers
    // ------------------------
    void handleSearchRight1();
    void handleReturnLeft1();

    void handleSearchRight2();
    void handleReturnLeft2();

    void handleSearchLeftFinal();
    void handleIslandSearch();
    void handleReturnLeftFinal();

    void handlePickup();
    void handleRelease();

    // ------------------------
    // Utility helpers
    // ------------------------
    bool startButtonPressed();

    bool atStartSquare();  // TODO: implement your own logic
    bool inIslandRegion(); // TODO: implement your island location check
    bool lineLost();

    unsigned long _junctionLockUntil = 0;
    bool junctionLocked() const;
    void lockJunction(unsigned long ms);
};
