#include "Ultrasonic.h"
#include <Arduino.h>

UltraSonic::UltraSonic()
    : _sonarRight(0, 0, 1), _sonarLeft(0, 0, 1), _sonarFront(0, 0, 1)
{
}

void UltraSonic::begin(uint8_t trigPinRight,uint8_t trigPinLeft,uint8_t trigPinFront, uint8_t echoPinRight, uint8_t echoPinLeft, uint8_t echoPinFront, unsigned int maxDistanceCm)
{

    pinMode(trigPinFront,OUTPUT);
    pinMode(trigPinLeft,OUTPUT);
    pinMode(trigPinRight,OUTPUT);
    _sonarRight = NewPing(trigPinRight, echoPinRight, maxDistanceCm);
    _sonarLeft = NewPing(trigPinLeft, echoPinLeft, maxDistanceCm);
    _sonarFront = NewPing(trigPinFront, echoPinFront, maxDistanceCm);
}

unsigned int UltraSonic::filteredPing(NewPing &sonar)
{
    const uint8_t NUM_SAMPLES = 3;
    unsigned int samples[NUM_SAMPLES];
    uint8_t count = 0;

    for (uint8_t i = 0; i < NUM_SAMPLES; i++)
    {
        unsigned int d = sonar.ping_cm();



        // NewPing returns 0 if no echo. Treat that as "far away".
        if (d == 0 || d > MAX_DISTANCE)
        {
            d = MAX_DISTANCE;
        }

        samples[count++] = d;
        delay(30);
    }

    if (count == 0)
    {
        return MAX_DISTANCE;
    }

    // Sort for median
    bool swapped;
    do
    {
        swapped = false;
        for (uint8_t i = 0; i + 1 < count; i++)
        {
            if (samples[i] > samples[i + 1])
            {
                unsigned int tmp = samples[i];
                samples[i] = samples[i + 1];
                samples[i + 1] = tmp;
                swapped = true;
            }
        }
    } while (swapped);

    return samples[count / 2];
}


// ---- Public distance API -----------------------------------------------

unsigned int UltraSonic::readDistFront() {
    return filteredPing(_sonarFront);
}

unsigned int UltraSonic::readDistLeft() {
    return filteredPing(_sonarLeft);
}

unsigned int UltraSonic::readDistRight() {
    return filteredPing(_sonarRight);
}
bool UltraSonic::cylinderDetected()
{
    if (readDistFront() < CYLINDER_DIST)
    {
        return true;
    }
    return false;
}

JunctionType UltraSonic::getJunction()
{
    unsigned int dF = readDistFront();
    unsigned int dL = readDistLeft();
    unsigned int dR = readDistRight();

    bool frontOpen = (dF > LOST_WALL);
    bool leftOpen = (dL > LOST_WALL);
    bool rightOpen = (dR > LOST_WALL);

    Serial.print("F: ");
    Serial.print(dF);
    Serial.print(" L: ");
    Serial.print(dL);
    Serial.print(" R: ");
    Serial.println(dR);

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
