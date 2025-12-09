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

unsigned int UltraSonic::readDistFront() {
    unsigned int d = _sonarFront.ping_cm();
    delay(30); // let echoes die out
    return d;
}

unsigned int UltraSonic::readDistLeft() {
    unsigned int d = _sonarLeft.ping_cm();
    delay(30);
    return d;
}

unsigned int UltraSonic::readDistRight() {
    unsigned int d = _sonarRight.ping_cm();
    delay(30);
    return d;
}

bool UltraSonic::cylinderDetected() {
    if(readDistFront() < CYLINDER_DIST){ return true;}
    return false;
}

JunctionType UltraSonic::getJunction(){
    unsigned int dF = readDistFront();
    unsigned int dL = readDistLeft();
    unsigned int dR = readDistRight();

    bool frontOpen = (dF > LOST_WALL);
    bool leftOpen  = (dL > LOST_WALL);
    bool rightOpen = (dR > LOST_WALL); 

    // Serial.print("front open: ");
    // Serial.print(frontOpen);
    // Serial.println();
    // Serial.print("left open: ");
    // Serial.print(leftOpen);
    // Serial.println();
    // Serial.print("right open: ");
    // Serial.print(rightOpen);
    // Serial.println();



    if (frontOpen && leftOpen && rightOpen)
        return JunctionType::CROSS;

    if (!frontOpen && leftOpen && !rightOpen)
        return JunctionType::LEFT;
    
    if (!frontOpen && !leftOpen && rightOpen)
        return JunctionType::RIGHT;
  
    if (!frontOpen && leftOpen && rightOpen)
        return JunctionType::T;

 
    if (frontOpen && leftOpen && !rightOpen)
        return JunctionType::LEFT_T;


    if (frontOpen && !leftOpen && rightOpen)
        return JunctionType::RIGHT_T;

  
    if (!frontOpen && !leftOpen && !rightOpen)
        return JunctionType::DEAD_END;

 
    return JunctionType::NONE;


}



