#pragma once
#include <Arduino.h>

class MotorDriver {
    public:
    
     void begin(uint8_t leftA, uint8_t leftB, uint8_t rightA, uint8_t rightB);
     void drive(int leftSpeed, int rightSpeed);
     void stop();

    private:
     uint8_t _lA, _lB, _rA, _rB;
    

};