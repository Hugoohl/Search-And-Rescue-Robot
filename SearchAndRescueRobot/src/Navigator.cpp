#include "Navigator.h"
#include "Arduino.h"

Navigator::Navigator(LineControl &line, MotorDriver &motor, UltraSonic &sonar) : _line(line), _motor(motor), _sonar(sonar) {}

void Navigator::begin()
{
    _state = RobotState::WAIT_FOR_START;
    _phase = MissionState::WAIT_FOR_BUTTON;

    pinMode(START_BUTTON_PIN, INPUT);
    _motor.stop();
}

bool Navigator::startButtonPressed()
{
    return digitalRead(START_BUTTON_PIN) == LOW;
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
    _motor.stop();

    if (startButtonPressed())
    {
        Serial.println("Start pressed -> CALIBRATING");
        _state = RobotState::CALIBRATING;
    }
}

void Navigator::handleCalibrating()
{
    _motor.stop();
    _line.calibrate();
    Serial.println("Calibration Done, starting mission");
    _phase = MissionState::SEARCH_RIGHT_1;
    _state = RobotState::RUN_MISSION;
}

void Navigator::handleMissionDone()
{
    _motor.stop();
    Serial.println("Mission Complete!");
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
    _missionStartTime = millis();

    switch (_phase)
    {
    case MissionState::SEARCH_RIGHT_1:
        handleSearchRight1();
        break;

    case MissionState::RETURN_LEFT_1:
        handleReturnLeft1();
        break;

    case MissionState::SEARCH_RIGHT_2:
        handleSearchRight2();
        break;

    case MissionState::RETURN_LEFT_2:
        handleReturnLeft2();
        break;

    case MissionState::SEARCH_LEFT_FINAL:
        handleSearchLeftFinal();
        break;

    case MissionState::ISLAND_SEARCH:
        handleIslandSearch();
        break;

    case MissionState::RETURN_LEFT_FINAL:
        handleReturnLeftFinal();
        break;

    case MissionState::PICKUP:
        handlePickup();
        break;

    case MissionState::RELEASE:
        handleRelease();
        break;

    case MissionState::DONE:
        handleMissionDone();
        break;

    default:
        break;
    }
}

bool Navigator::detectCylinder()
{
    
    return false;
}

bool Navigator::inIslandRegion()
{
    
    return false;
}

bool Navigator::atStartSquare()
{
    
    return false;
}

bool Navigator::lineLost(){
    if(_line.detectJunction() == JunctionType::DEAD_END && _sonar.readDistFront() >THRESHOLD_DISTANCE){
        

        return true;
    }
    return false;
}

void Navigator::rotate180()
{
    while(_line.getSingle() <JUNCTION_THRESHOLD_SINGLE){
        _motor.drive(DC_MOTOR_BASE_SPEED, -DC_MOTOR_BASE_SPEED);
    }
    _motor.stop();
}

void Navigator::rotate90left()
{
    while(_line.getSingle() <JUNCTION_THRESHOLD_SINGLE){
        _motor.drive(DC_MOTOR_BASE_SPEED, DC_MOTOR_TURN_SPEED);
    }
    _motor.stop();
}

void Navigator::rotate90right()
{
    while(_line.getSingle() <JUNCTION_THRESHOLD_SINGLE){
        _motor.drive(DC_MOTOR_TURN_SPEED, DC_MOTOR_BASE_SPEED);
    }
    _motor.stop();
}

void Navigator::testLinePidAndTurns()
{
    static bool justTurned = false;   // to avoid re-triggering on the same physical junction

    int leftSpeed = 0;
    int rightSpeed = 0;

    // 1) Check IR for junction
    bool isJunc  = _line.isJunction();

    if (!isJunc) {
        // No junction -> normal PID line following
        justTurned = false;
        _line.computeSpeedsPid(leftSpeed, rightSpeed);
        _motor.drive(leftSpeed, rightSpeed);
        return;
    }

    // If we're still physically on the same junction as last loop, don't turn again
    if (justTurned) {
        _line.computeSpeedsPid(leftSpeed, rightSpeed);
        _motor.drive(leftSpeed, rightSpeed);
        return;
    }

    // We have a new junction
    justTurned = true;
    _motor.stop();
    delay(100);   // small stabilization delay

    // 2) Ask ultrasonic what kind of junction this is
    JunctionType usJ = _sonar.getJunction();

    // 3) Decide turn based on ultrasonic junction type
    switch (usJ)
    {
    case JunctionType::DEAD_END:
        // Wall ahead and both sides blocked -> U-turn
        rotate180();
        break;

    case JunctionType::LEFT_T:
        // Front blocked, left open -> turn left
        rotate90left();
        break;

    case JunctionType::RIGHT_T:
        // Front blocked, right open -> turn right
        rotate90right();
        break;

    case JunctionType::T:
    case JunctionType::CROSS:
        // Both sides open (and maybe front) -> for test, choose right-hand rule
        rotate90right();
        break;

    case JunctionType::NONE:
    default:
        // Ultrasonic saw nothing special -> go straight (do nothing)
        break;
    }

    // After turn, let PID handle re-centering on the new line
}



void Navigator::followRightWall()
{
    int leftSpeed = 0;
    int rightSpeed = 0;

    // Base line following
    lineLost();
    _line.computeSpeeds(leftSpeed, rightSpeed);

    unsigned int distF = _sonar.readDistFront();
    unsigned int distR = _sonar.readDistRight();
    unsigned int distL = _sonar.readDistLeft();

    if (distF == 0) distF = MAX_DISTANCE;
    if (distR == 0) distR = MAX_DISTANCE;
    if (distL == 0) distL = MAX_DISTANCE;

    if (distF < THRESHOLD_DISTANCE)
    {
        if (distR > LOST_WALL) {
            // Turn RIGHT
            rotate90right();
        }
        else if (distL > LOST_WALL) {
            // Turn LEFT
            rotate90left();
        } else if (distL > LOST_WALL && distR > LOST_WALL){
            // Turn RIGHT
              rotate90right();
        }
        else {
            // DEAD END
            rotate180();
        }
    }

    _motor.drive(leftSpeed, rightSpeed);
}

void Navigator::followLeftWall()
{
    int leftSpeed = 0;
    int rightSpeed = 0;
    lineLost();
    
    _line.computeSpeeds(leftSpeed, rightSpeed);

    unsigned int distF = _sonar.readDistFront();
    unsigned int distR = _sonar.readDistRight();
    unsigned int distL = _sonar.readDistLeft();

    if (distF < THRESHOLD_DISTANCE)
    {
        if (distL > LOST_WALL) {
            // Turn LEFT
            leftSpeed  = DC_MOTOR_TURN_SPEED;
            rightSpeed = DC_MOTOR_BASE_SPEED;
        }
        else if (distR > LOST_WALL) {
            // Turn RIGHT
            leftSpeed  = DC_MOTOR_BASE_SPEED;
            rightSpeed = DC_MOTOR_TURN_SPEED;
        } else if (distL > LOST_WALL && distR > LOST_WALL){
            // Turn RIGHT
            leftSpeed  = DC_MOTOR_TURN_SPEED;
            rightSpeed = DC_MOTOR_BASE_SPEED;
        }
        else {
            // DEAD END
            rotate180();
        }
    }

    _motor.drive(leftSpeed, rightSpeed);
}

void Navigator::handleSearchRight1()
{
    followRightWall();

    if (detectCylinder()) {
        _phase = MissionState::PICKUP;
        _phaseBeforePickup = MissionState::SEARCH_RIGHT_1;
    }
}


// ---- RETURN WITH FIRST CYLINDER ----
void Navigator::handleReturnLeft1()
{
    followLeftWall();

    if (atStartSquare()) {
        _lastReturnPhase = 1;
        _phase = MissionState::RELEASE;
    }
}


// ---- SECOND CYLINDER SEARCH ----
void Navigator::handleSearchRight2()
{
    followRightWall();

    if (detectCylinder()) {
        _phase = MissionState::PICKUP;
        _phaseBeforePickup = MissionState::SEARCH_RIGHT_2;
    }
}


// ---- RETURN SECOND CYLINDER ----
void Navigator::handleReturnLeft2()
{
    followLeftWall();

    if (atStartSquare()) {
        _lastReturnPhase = 2;
        _phase = MissionState::RELEASE;
    }
}


// ---- SEARCH THIRD CYLINDER USING LEFT WALL ----
void Navigator::handleSearchLeftFinal()
{
    followLeftWall();

    if (detectCylinder()) {
        _phase = MissionState::PICKUP;
        _phaseBeforePickup = MissionState::SEARCH_LEFT_FINAL;
    }

    // If cylinder not found in expected place → go island
    if (inIslandRegion() && !detectCylinder()) {
        _phase = MissionState::ISLAND_SEARCH;
    }
}


// ---- ISLAND SEARCH PATTERN ----
void Navigator::handleIslandSearch()
{
    // You implement your pattern here
    // Hard-coded movement

    if (detectCylinder()) {
        _phase = MissionState::PICKUP;
        _phaseBeforePickup = MissionState::SEARCH_LEFT_FINAL;
    }
}


// ---- RETURN FINAL CYLINDER ----
void Navigator::handleReturnLeftFinal()
{
    followLeftWall();

    if (atStartSquare()) {
        _lastReturnPhase = 3;
        _phase = MissionState::RELEASE;
    }
}


// ---- PICKUP ----
void Navigator::handlePickup()
{
    // TODO: add servo/arm code

    rotate180();

    if (_phaseBeforePickup == MissionState::SEARCH_RIGHT_1)
        _phase = MissionState::RETURN_LEFT_1;

    else if (_phaseBeforePickup == MissionState::SEARCH_RIGHT_2)
        _phase = MissionState::RETURN_LEFT_2;

    else if (_phaseBeforePickup == MissionState::SEARCH_LEFT_FINAL)
        _phase = MissionState::RETURN_LEFT_FINAL;
}


// ---- RELEASE CYLINDER ----
void Navigator::handleRelease()
{
    // TODO: open claw servo
    _motor.stop();

    if (_lastReturnPhase == 1)
        _phase = MissionState::SEARCH_RIGHT_2;

    else if (_lastReturnPhase == 2)
        _phase = MissionState::SEARCH_LEFT_FINAL;

    else if (_lastReturnPhase == 3)
        _phase = MissionState::DONE;
} 