#include "Navigator.h"
#include "Arduino.h"

Navigator::Navigator(LineControl &line, MotorDriver &motor, UltraSonic &sonar, Gripper &serv)
    : _line(line), _motor(motor), _sonar(sonar), _serv(serv) {}

void Navigator::begin()
{
    _state = RobotState::WAIT_FOR_START;
    _phase = MissionState::WAIT_FOR_BUTTON;

    pinMode(START_BUTTON_PIN, INPUT_PULLUP); // FIX: avoid floating
    _motor.stop();

    _missionStartTime = 0;
}

bool Navigator::startButtonPressed()
{
    return digitalRead(START_BUTTON_PIN) == LOW; // pressed -> GND
}

void Navigator::update()
{
    // always keep sonar fresh (non-blocking)
    _sonar.update();

    switch (_state)
    {
    case RobotState::WAIT_FOR_START: handleWaitForStart(); break;
    case RobotState::CALIBRATING:    handleCalibrating();  break;
    case RobotState::RUN_MISSION:    handleRunMission();   break;
    case RobotState::MISSION_DONE:   handleMissionDone();  break;
    }
}

void Navigator::handleWaitForStart()
{
    _motor.stop();
    if (startButtonPressed())
    {
        Serial.println("Start pressed -> CALIBRATING");
        _state = RobotState::CALIBRATING;
    }
}

void Navigator::handleCalibrating()
{
    static unsigned long calibStartTime = 0;
    static unsigned long lastToggleTime = 0;
    static bool turnLeft = true;

    if (calibStartTime == 0)
    {
        calibStartTime = millis();
        lastToggleTime = calibStartTime;
        turnLeft = true;
        Serial.println("Calibration started, sweeping over line...");
    }

    unsigned long now = millis();
    unsigned long elapsed = now - calibStartTime;

    if (elapsed < CALIBRATION_TIME)
    {
        _line.calibrateStep();

        const int sweepSpeed = DC_MOTOR_CALIB_SPEED;
        if (now - lastToggleTime > 700)
        {
            turnLeft = !turnLeft;
            lastToggleTime = now;
        }

        if (turnLeft) _motor.drive(-sweepSpeed, sweepSpeed);
        else         _motor.drive( sweepSpeed,-sweepSpeed);
        return;
    }

    _motor.stop();
    calibStartTime = 0;
    lastToggleTime = 0;

    Serial.println("Calibration done -> RUN_MISSION");
    _phase = MissionState::SEARCH_RIGHT_1;
    _state = RobotState::RUN_MISSION;

    _missionStartTime = millis(); // FIX: set once here
}

void Navigator::handleMissionDone()
{
    _motor.stop();
    Serial.println("Mission Complete!");
}

// ---------------- Rotation helpers with timeout ----------------

void Navigator::rotate180()
{
    Serial.println("rotate180");
    const uint16_t TH = JUNCTION_THRESHOLD_SINGLE;
    unsigned long start = millis();

    // 1) leave current black patch
    unsigned long leaveStart = millis();
    while (_line.getSingle() > TH)
    {
        if (millis() - leaveStart > ROTATE_EXIT_BLACK_TIMEOUT_MS) break;
        _motor.drive(DC_MOTOR_BASE_SPEED, -DC_MOTOR_BASE_SPEED);
    }

    // 2) find line again (timeout)
    while (_line.getSingle() < TH)
    {
        if (millis() - start > ROTATE_TIMEOUT_MS) break;
        _motor.drive(DC_MOTOR_BASE_SPEED, -DC_MOTOR_BASE_SPEED);
    }

    _motor.stop();
}

void Navigator::rotate90left()
{
    Serial.println("rotate90left");
    const uint16_t TH = JUNCTION_THRESHOLD_SINGLE;
    unsigned long start = millis();

    unsigned long leaveStart = millis();
    while (_line.getSingle() > TH)
    {
        if (millis() - leaveStart > ROTATE_EXIT_BLACK_TIMEOUT_MS) break;
        _motor.drive(-DC_MOTOR_BASE_SPEED, DC_MOTOR_BASE_SPEED);
    }

    while (_line.getSingle() < TH)
    {
        if (millis() - start > ROTATE_TIMEOUT_MS) break;
        _motor.drive(-DC_MOTOR_BASE_SPEED, DC_MOTOR_BASE_SPEED);
    }

    _motor.stop();
}

void Navigator::rotate90right()
{
    Serial.println("rotate90right");
    const uint16_t TH = JUNCTION_THRESHOLD_SINGLE;
    unsigned long start = millis();

    unsigned long leaveStart = millis();
    while (_line.getSingle() > TH)
    {
        if (millis() - leaveStart > ROTATE_EXIT_BLACK_TIMEOUT_MS) break;
        _motor.drive(DC_MOTOR_BASE_SPEED, -DC_MOTOR_BASE_SPEED);
    }

    while (_line.getSingle() < TH)
    {
        if (millis() - start > ROTATE_TIMEOUT_MS) break;
        _motor.drive(DC_MOTOR_BASE_SPEED, -DC_MOTOR_BASE_SPEED);
    }

    _motor.stop();
}

// ---------------- Core mission loop ----------------

bool Navigator::detectCylinder()
{
    return _sonar.cylinderDetected();
}

bool Navigator::inIslandRegion()
{
    return false; // TODO
}

bool Navigator::atStartSquare()
{
    // Keep your logic, but it depends on better dead-end now
    if (_line.isDeadend() && _sonar.getJunction() == JunctionType::RIGHT)
    {
        Serial.println("At starting square");
        return true;
    }
    return false;
}

// ---------------- Smarter, debounced junction handler ----------------

static bool debouncedJunction(LineControl &line, JunctionType &outType)
{
    static JunctionType lastType = JunctionType::NONE;
    static unsigned long stableSince = 0;

    JunctionType t = line.detectJunction();

    if (t != lastType)
    {
        lastType = t;
        stableSince = millis();
        return false;
    }

    if (t != JunctionType::NONE && (millis() - stableSince) >= JUNCTION_DEBOUNCE_MS)
    {
        outType = t;
        return true;
    }

    return false;
}

// ---------------- Wall-follow functions (fixed junction gating) ----------------

void Navigator::followRightWall()
{
    if (startButtonPressed())
    {
        Serial.println("Button pressed -> STOP");
        _motor.stop();
        _phase = MissionState::WAIT_FOR_BUTTON;
        _state = RobotState::WAIT_FOR_START;
        return;
    }

    int leftSpeed = 0, rightSpeed = 0;
    _line.computeSpeedsPid(leftSpeed, rightSpeed);

    JunctionType irType;
    bool atJunction = debouncedJunction(_line, irType);

    if (!atJunction)
    {
        _motor.drive(leftSpeed, rightSpeed);
        return;
    }

    // At a junction -> stop and decide using ultrasound
    _motor.stop();
    delay(10);

    JunctionType us = _sonar.getJunction();
    Serial.print("IR: "); Serial.print(_line.junctionTypeToString(irType));
    Serial.print(" | US: "); Serial.println(_line.junctionTypeToString(us));

    // Right-hand rule preference
    switch (us)
    {
    case JunctionType::DEAD_END: rotate180(); break;
    case JunctionType::RIGHT:
    case JunctionType::RIGHT_T: rotate90right(); break;
    case JunctionType::T:
    case JunctionType::CROSS:   rotate90right(); break;
    case JunctionType::LEFT:
    case JunctionType::LEFT_T:  rotate90left(); break; // only if needed
    default: break; // go straight
    }
}

void Navigator::followLeftWall()
{
    if (startButtonPressed())
    {
        Serial.println("Button pressed -> STOP");
        _motor.stop();
        _phase = MissionState::WAIT_FOR_BUTTON;
        _state = RobotState::WAIT_FOR_START;
        return;
    }

    int leftSpeed = 0, rightSpeed = 0;
    _line.computeSpeedsPid(leftSpeed, rightSpeed);

    JunctionType irType;
    bool atJunction = debouncedJunction(_line, irType);

    if (!atJunction)
    {
        _motor.drive(leftSpeed, rightSpeed);
        return;
    }

    _motor.stop();
    delay(10);

    JunctionType us = _sonar.getJunction();
    Serial.print("IR: "); Serial.print(_line.junctionTypeToString(irType));
    Serial.print(" | US: "); Serial.println(_line.junctionTypeToString(us));

    // Left-hand rule preference
    switch (us)
    {
    case JunctionType::DEAD_END: rotate180(); break;
    case JunctionType::LEFT:
    case JunctionType::LEFT_T:  rotate90left(); break;
    case JunctionType::T:
    case JunctionType::CROSS:   rotate90left(); break;
    case JunctionType::RIGHT:
    case JunctionType::RIGHT_T: rotate90right(); break;
    default: break;
    }
}

void Navigator::handleRunMission()
{
    if (startButtonPressed())
    {
        Serial.println("Button pressed -> STOP MISSION");
        _motor.stop();
        _phase = MissionState::WAIT_FOR_BUTTON;
        _state = RobotState::WAIT_FOR_START;
        return;
    }

    switch (_phase)
    {
    case MissionState::SEARCH_RIGHT_1:    handleSearchRight1(); break;
    case MissionState::RETURN_LEFT_1:     handleReturnLeft1(); break;
    case MissionState::SEARCH_RIGHT_2:    handleSearchRight2(); break;
    case MissionState::RETURN_LEFT_2:     handleReturnLeft2(); break;
    case MissionState::SEARCH_LEFT_FINAL: handleSearchLeftFinal(); break;
    case MissionState::ISLAND_SEARCH:     handleIslandSearch(); break;
    case MissionState::RETURN_LEFT_FINAL: handleReturnLeftFinal(); break;
    case MissionState::PICKUP:            handlePickup(); break;
    case MissionState::RELEASE:           handleRelease(); break;
    case MissionState::DONE:              handleMissionDone(); break;
    default: break;
    }
}

void Navigator::handleSearchRight1()
{
    followRightWall();
    if (detectCylinder())
    {
        Serial.println("Cylinder detected -> PICKUP");
        _phase = MissionState::PICKUP;
        _phaseBeforePickup = MissionState::SEARCH_RIGHT_1;
    }
}

void Navigator::handleReturnLeft1()
{
    followLeftWall();
    if (atStartSquare())
    {
        _lastReturnPhase = 1;
        _phase = MissionState::RELEASE;
    }
}

void Navigator::handleSearchRight2()
{
    followRightWall();
    if (detectCylinder())
    {
        _phase = MissionState::PICKUP;
        _phaseBeforePickup = MissionState::SEARCH_RIGHT_2;
    }
}

void Navigator::handleReturnLeft2()
{
    followLeftWall();
    if (atStartSquare())
    {
        _lastReturnPhase = 2;
        _phase = MissionState::RELEASE;
    }
}

void Navigator::handleSearchLeftFinal()
{
    followLeftWall();

    if (detectCylinder())
    {
        _phase = MissionState::PICKUP;
        _phaseBeforePickup = MissionState::SEARCH_LEFT_FINAL;
    }

    if (inIslandRegion() && !detectCylinder())
    {
        _phase = MissionState::ISLAND_SEARCH;
    }
}

void Navigator::handleIslandSearch()
{
    Serial.println("Mission state: Island search");
    // TODO: implement island behaviour
    if (detectCylinder())
    {
        _phase = MissionState::PICKUP;
        _phaseBeforePickup = MissionState::SEARCH_LEFT_FINAL;
    }
}

void Navigator::handleReturnLeftFinal()
{
    followLeftWall();
    if (atStartSquare())
    {
        _lastReturnPhase = 3;
        _phase = MissionState::RELEASE;
    }
}

void Navigator::handlePickup()
{
    _motor.stop();
    delay(80);
    _serv.pickup();

    rotate180();

    if (_phaseBeforePickup == MissionState::SEARCH_RIGHT_1)      _phase = MissionState::RETURN_LEFT_1;
    else if (_phaseBeforePickup == MissionState::SEARCH_RIGHT_2) _phase = MissionState::RETURN_LEFT_2;
    else                                                        _phase = MissionState::RETURN_LEFT_FINAL;
}

void Navigator::handleRelease()
{
    _motor.stop();
    delay(80);
    _serv.release();

    // back off a little
    _motor.drive(-DC_MOTOR_BASE_SPEED, -DC_MOTOR_BASE_SPEED);
    delay(350);
    _motor.stop();

    rotate180();

    if (_lastReturnPhase == 1)      _phase = MissionState::SEARCH_RIGHT_2;
    else if (_lastReturnPhase == 2) _phase = MissionState::SEARCH_LEFT_FINAL;
    else                            _phase = MissionState::DONE;
}
