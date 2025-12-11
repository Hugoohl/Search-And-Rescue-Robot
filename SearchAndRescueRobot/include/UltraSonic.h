#pragma once
#include <Arduino.h>
#include <NewPing.h>
#include "Types.h"
#include "Config.h"

class UltraSonic {
public:
    UltraSonic();
    void begin(uint8_t trigPinRight,uint8_t trigPinLeft,uint8_t trigPinFront,
         uint8_t echoPinRight, 
         uint8_t echoPinLeft, 
         uint8_t echoPinFront, 
         unsigned int maxDistanceCm);

    unsigned int readDistRight();
    unsigned int readDistLeft();
    unsigned int readDistFront();
    JunctionType getJunction();
    bool cylinderDetected();

    
    unsigned int filteredPing(NewPing &sonar );
    
    

private:
    NewPing _sonarRight;
    NewPing _sonarLeft;
    NewPing _sonarFront;
};
