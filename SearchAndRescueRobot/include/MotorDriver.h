#pragma once
#include <Arduino.h>

class MotorDriver {
    public:
    
     void begin(int leftA, int leftB, int rightA, int rightB);
     void drive(int leftSpeed, int rightSpeed);
     void stop();

    private:
     int _lA, _lB, _rA, _rB;
    

};