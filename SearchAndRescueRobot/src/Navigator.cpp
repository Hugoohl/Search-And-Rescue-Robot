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
    unsigned long t = millis();
    while (millis() - t < 1500) {        
        _motor.drive(-DC_MOTOR_BASE_SPEED, DC_MOTOR_BASE_SPEED);
    }
    Serial.println("Uturn");
    _motor.stop();
}

void Navigator::rotate90left()
{
    unsigned long t = millis();
    while (millis() - t < 750) {        
        _motor.drive(-DC_MOTOR_BASE_SPEED, DC_MOTOR_BASE_SPEED);
    }
    Serial.println("Turning left");
    _motor.stop();
    
}

void Navigator::rotate90right()
{
    unsigned long t = millis();
    while (millis() - t < 750) {        
        _motor.drive(DC_MOTOR_BASE_SPEED, -DC_MOTOR_BASE_SPEED);
    }
    Serial.println("Turning right");
    _motor.stop();
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