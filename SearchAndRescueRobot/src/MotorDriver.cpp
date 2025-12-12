#include "MotorDriver.h"
#include "Config.h"
#include <Arduino.h>

static inline int applyInvert(int v, bool inv) { return inv ? -v : v; }

void MotorDriver::begin(uint8_t leftA, uint8_t leftB, uint8_t rightA, uint8_t rightB) {
    _lA = leftA; _lB = leftB;
    _rA = rightA; _rB = rightB;
    pinMode(_lA, OUTPUT);
    pinMode(_lB, OUTPUT);
    pinMode(_rA, OUTPUT);
    pinMode(_rB, OUTPUT);
    stop();
}

void MotorDriver::drive(int leftSpeed, int rightSpeed) {
    leftSpeed  = applyInvert(leftSpeed,  LEFT_MOTOR_INVERTED);
    rightSpeed = applyInvert(rightSpeed, RIGHT_MOTOR_INVERTED);

    leftSpeed  = constrain(leftSpeed,  -DC_MOTOR_MAX_SPEED, DC_MOTOR_MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -DC_MOTOR_MAX_SPEED, DC_MOTOR_MAX_SPEED);

    if (leftSpeed >= 0) { analogWrite(_lA, leftSpeed);  digitalWrite(_lB, LOW); }
    else                { analogWrite(_lB, -leftSpeed); digitalWrite(_lA, LOW); }

    if (rightSpeed >= 0) { analogWrite(_rA, rightSpeed);  digitalWrite(_rB, LOW); }
    else                 { analogWrite(_rB, -rightSpeed); digitalWrite(_rA, LOW); }
}

void MotorDriver::stop() {
    drive(0, 0);
}
