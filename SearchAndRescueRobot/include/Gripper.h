#pragma once
#include <Servo.h>
#include "Config.h"

class Gripper{

    public:
    void begin(uint8_t gripPin, uint8_t tiltPin);
    void pickup();
    void release();


    private:
    Servo gripServ;
    Servo tiltServ;
};
