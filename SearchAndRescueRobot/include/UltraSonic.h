#pragma once
#include <Arduino.h>
#include <NewPing.h>

class UltraSonic {
public:
    UltraSonic();
    void begin(uint8_t trigPin,
         uint8_t echoPinRight, 
         uint8_t echoPinLeft, 
         uint8_t echoPinFront, 
         unsigned int maxDistanceCm);

    unsigned int readDistRight();
    unsigned int readDistLeft();
    unsigned int readDistFront();
    

private:
    NewPing _sonarRight;
    NewPing _sonarLeft;
    NewPing _sonarFront;
};
