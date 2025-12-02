#include "Ultrasonic.h"

UltraSonic::UltraSonic()
    : _sonarRight(0, 0, 1), _sonarLeft(0, 0, 1), _sonarFront(0, 0, 1)
{
}

void UltraSonic::begin(uint8_t trigPin, uint8_t echoPinRight, uint8_t echoPinLeft, uint8_t echoPinFront, unsigned int maxDistanceCm)
{
    _sonarRight = NewPing(trigPin, echoPinRight, maxDistanceCm);
    _sonarLeft = NewPing(trigPin, echoPinLeft, maxDistanceCm);
    _sonarFront = NewPing(trigPin, echoPinFront, maxDistanceCm);
}

unsigned int UltraSonic::readDistRight()
{
    return _sonarRight.ping_cm(); 
}

unsigned int UltraSonic::readDistLeft()
{
    return _sonarLeft.ping_cm(); 
}

unsigned int UltraSonic::readDistFront()
{
    return _sonarFront.ping_cm();
}
