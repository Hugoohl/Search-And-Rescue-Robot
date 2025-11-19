#include "MotorDriver.h"
#include "Config.h"



void MotorDriver::begin(int leftA, int leftB, int rightA, int rightB){
    _lA = leftA; _lB = leftB;
    _rA = rightA; _rB = rightB;
    pinMode(_lA, OUTPUT);
    pinMode(_lB, OUTPUT);
    pinMode(_rA, OUTPUT);
    pinMode(_rB, OUTPUT);
    stop();

}

void MotorDriver::drive(int leftSpeed, int rightSpeed){
    leftSpeed = constrain(leftSpeed, -DC_MOTOR_MAX_SPEED, DC_MOTOR_MAX_SPEED);
    rightSpeed = constrain(-rightSpeed, -DC_MOTOR_MAX_SPEED, DC_MOTOR_MAX_SPEED);

    if (leftSpeed >= 0) { analogWrite(_lA, leftSpeed); digitalWrite(_lB, LOW); }
    else                   { analogWrite(_lB, -leftSpeed); digitalWrite(_lA, LOW); }

  
    if (rightSpeed >= 0) { analogWrite(_rA, rightSpeed); digitalWrite(_rB, LOW); }
    else                    { analogWrite(_rB, -rightSpeed); digitalWrite(_rA, LOW); }
}

void MotorDriver::stop(){
    digitalWrite(_lA, LOW);
    digitalWrite(_lB, LOW);
    digitalWrite(_rA, LOW);
    digitalWrite(_rB, LOW);
}