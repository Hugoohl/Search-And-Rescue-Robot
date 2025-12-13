#include <Arduino.h>
#include "Config.h"
#include "Pins.h"
#include "MotorDriver.h"
#include "LineControl.h"
#include "Types.h"
#include "Navigator.h"
#include <QTRSensors.h>
#include "UltraSonic.h"
#include "Gripper.h"

MotorDriver motor;
LineControl line;
UltraSonic sonar;
Gripper serv;

Navigator nav(line, motor, sonar, serv);

int junctionCount = 0;
bool returning = false;
bool picked = false;
bool calib = false;

bool calibrated = false;
unsigned long calibStartTime = 0;
unsigned long lastToggleTime = 0;
bool calibTurnLeft = true;

unsigned long junctionLockUntil = 0;

// tweak
const unsigned long JUNCTION_LOCK_MS = 600; // ignore new junctions after one
const int JUNCTION_TARGET_OUT = 10;         // <-- junction number where cylinder is
const int JUNCTION_TARGET_BACK = 10;        // <-- junction number to stop at on return (start)

void setup()
{
  Serial.begin(9600);
  motor.begin(MOTOR_PIN_M1A, MOTOR_PIN_M1B, MOTOR_PIN_M2A, MOTOR_PIN_M2B);
  uint8_t arrayPins[] = {IR_ARRAY_PIN1, IR_ARRAY_PIN2, IR_ARRAY_PIN3, IR_ARRAY_PIN4};
  uint8_t singlePin[] = {IR_PIN};
  line.begin(arrayPins, singlePin);
  serv.begin(GRIP_SERVO_PIN, TILT_SERVO_PIN);
  sonar.begin(US_PIN_TRIG_RIGHT, US_PIN_TRIG_LEFT, US_PIN_TRIG_FRONT, US_PIN_ECHO_RIGHT, US_PIN_ECHO_LEFT, US_PIN_ECHO_FRONT, MAX_DISTANCE);
}

int leftSpeed = 0;
int rightSpeed = 0;

bool junctionEvent()
{
  if (millis() < junctionLockUntil)
    return false;

  // "a junction happened" = many sensors see black (you already have this)
  JunctionType j = line.detectJunction();

  bool isJunction = (j == JunctionType::JUNCTION ||
                     j == JunctionType::LEFT_T ||
                     j == JunctionType::RIGHT_T ||
                     j == JunctionType::T ||
                     j == JunctionType::CROSS ||
                     j == JunctionType::RIGHT ||
                     j == JunctionType::LEFT);

  if (isJunction  && sonar.getJunctionStable() != JunctionType::NONE)
  {
    junctionLockUntil = millis() + JUNCTION_LOCK_MS;
    return true;
  }
  return false;
}



bool calibrationSweep()
{
  // returns true when calibration is finished

  if (calibStartTime == 0)
  {
    calibStartTime = millis();
    lastToggleTime = calibStartTime;
    calibTurnLeft = true;
    Serial.println("Calibration started, sweeping over line...");
  }

  unsigned long now = millis();
  unsigned long elapsed = now - calibStartTime;

  if (elapsed < CALIBRATION_TIME)
  {
    line.calibrateStep();

    const int sweepSpeed = DC_MOTOR_CALIB_SPEED;

    if (now - lastToggleTime > 700)
    {
      calibTurnLeft = !calibTurnLeft;
      lastToggleTime = now;
    }

    if (calibTurnLeft)
      motor.drive(-sweepSpeed, sweepSpeed);
    else
      motor.drive(sweepSpeed, -sweepSpeed);

    return false; // still calibrating
  }
  const uint16_t TH = JUNCTION_THRESHOLD_SINGLE;
  unsigned long tFind = millis();
  int onCount = 0;

  while (millis() - tFind < 1500)
  {                  
    
    int pos = line.getLine();
    if(pos>1500){
      motor.drive(-DC_MOTOR_CALIB_SPEED, DC_MOTOR_CALIB_SPEED); // rotate right slowly
    } else if(pos<1500){
      motor.drive(DC_MOTOR_CALIB_SPEED, -DC_MOTOR_CALIB_SPEED); // rotate right slowly
    }
    

    if (line.getSingle() > TH)
      onCount++;
    else
      onCount = 0;

    if (onCount >= 6)
      break; // stable detection
    delay(5);
  }

  motor.stop();

  // small forward to be fully on line
  motor.drive(DC_MOTOR_BASE_SPEED, DC_MOTOR_BASE_SPEED);
  delay(150);
  motor.stop();   

  calibStartTime = 0;
  lastToggleTime = 0;

  Serial.println("Calibration done!");
  return true; // finished
}

// placeholders you will replace later
void TURN_LEFT() { nav.rotate90left(); }
void TURN_RIGHT() { nav.rotate90right(); }
void TURN_AROUND() { nav.rotate180(); }

void loop()
{
  if (!calibrated)
  {
    calibrated = calibrationSweep();
    return; // don't run mission until calibration done
  }

  sonar.update();
  bool cylinder = false;
  // refresh enough times so front is updated at least once

  uint16_t f = sonar.readDistFront();
  if (f < CYLINDER_DIST)
  {
    cylinder = true;
  }

  Serial.println(cylinder);

  if (!picked && cylinder)
  {
    Serial.println("cylinder found");
    motor.stop();
    delay(150);

    serv.pickup();
    picked = true;

    TURN_AROUND();

    returning = true;
    junctionCount = 10-junctionCount; // reset counter for return route
    junctionLockUntil = millis() + 900;
  }

  // Follow line by PID unless we decide otherwise
  int l = 0, r = 0;
  
  line.computeSpeedsPid(l, r);
  if(returning){l = l -30; r = r-30;}
  motor.drive(l, r);

  // Detect passing a junction (just an event)
  if (junctionEvent())
  {
    junctionCount++;
    Serial.println(junctionCount);

    motor.stop();
    delay(80);

    // ---------------- OUTGOING PATH ----------------
    if (!returning)
    {
      // HARD CODE: what to do at each junction count
      if (junctionCount == 1)
      {
        TURN_RIGHT(); // <-- change later
      }
      else if (junctionCount == 2)
      {
        TURN_RIGHT(); // <-- change later
      }
      else if (junctionCount == 3)
      {
        TURN_LEFT(); // ...
      }
      else if (junctionCount == 4)
      {
        TURN_RIGHT();
      }
      else if (junctionCount == 5)
      {
      }
      else if (junctionCount == 6)
      {
        TURN_RIGHT(); // <-- change later
      }
      else if (junctionCount == 7)
      {
        TURN_RIGHT(); //
      }
      else if (junctionCount == 8)
      {
        TURN_LEFT();
      }
      else if (junctionCount == 9)
      {
        TURN_LEFT(); // ...
      }
      else if (junctionCount == 10)
      {
        TURN_LEFT();
      }
    }

    // ---------------- RETURN PATH ----------------
    else
    { 
      
      // HARD CODE return turns by junction count on the way back
      if (junctionCount == 1)
      {
        TURN_RIGHT();
      }
      else if (junctionCount == 2)
      {
        TURN_RIGHT(); // <-- change later
      }
      else if (junctionCount == 3)
      {
        TURN_RIGHT(); // ...
      }
      else if (junctionCount == 4)
      {
        TURN_LEFT();
      }
      else if (junctionCount == 5)
      {
        TURN_LEFT();
      }
      else if (junctionCount == 6)
      {
       
      }
      else if (junctionCount == 7)
      {
        TURN_LEFT(); // ...
      }
      else if (junctionCount == 8)
      {
        TURN_RIGHT();
      }
      else if (junctionCount == 9)
      {
        TURN_LEFT(); // ...
      }
      else if (junctionCount == 10)
      {
        TURN_LEFT();
      }

      // Stop at "start" after a known number of junctions on return
      if (junctionCount == JUNCTION_TARGET_BACK)
      {
        motor.stop();
        TURN_RIGHT();
        serv.release();
      }
    }

    // After any turn, push forward a bit so we don't re-trigger the same junction
    motor.drive(DC_MOTOR_BASE_SPEED, DC_MOTOR_BASE_SPEED);
    delay(180);
  }
}