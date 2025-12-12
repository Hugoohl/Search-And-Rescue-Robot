#include "Ultrasonic.h"

UltraSonic::UltraSonic()
    : _sonarRight(0, 0, 1), _sonarLeft(0, 0, 1), _sonarFront(0, 0, 1) {}

void UltraSonic::begin(uint8_t trigPinRight, uint8_t trigPinLeft, uint8_t trigPinFront,
                       uint8_t echoPinRight, uint8_t echoPinLeft, uint8_t echoPinFront,
                       unsigned int maxDistanceCm)
{
    pinMode(trigPinFront, OUTPUT);
    pinMode(trigPinLeft, OUTPUT);
    pinMode(trigPinRight, OUTPUT);

    _sonarRight = NewPing(trigPinRight, echoPinRight, maxDistanceCm);
    _sonarLeft  = NewPing(trigPinLeft,  echoPinLeft,  maxDistanceCm);
    _sonarFront = NewPing(trigPinFront, echoPinFront, maxDistanceCm);

    _dR = _dL = _dF = MAX_DISTANCE;
    _rrIndex = 0;
    _lastPingMs = 0;
}

void UltraSonic::update()
{
    unsigned long now = millis();
    if (now - _lastPingMs < SONAR_PING_PERIOD_MS) return;
    _lastPingMs = now;

    // Ping one sensor per call (round robin)
    if (_rrIndex == 0) _dF = sanitize(_sonarFront.ping_cm());
    else if (_rrIndex == 1) _dR = sanitize(_sonarRight.ping_cm());
    else _dL = sanitize(_sonarLeft.ping_cm());

    _rrIndex = (_rrIndex + 1) % 3;
}

bool UltraSonic::cylinderDetected() const
{
    return (_dF < CYLINDER_DIST);
}

JunctionType UltraSonic::getJunction() const
{
    bool frontOpen = (_dF > LOST_WALL);
    bool leftOpen  = (_dL > LOST_WALL);
    bool rightOpen = (_dR > LOST_WALL);

    if (frontOpen && leftOpen && rightOpen) return JunctionType::CROSS;

    if (!frontOpen && leftOpen && !rightOpen) return JunctionType::LEFT;
    if (!frontOpen && !leftOpen && rightOpen) return JunctionType::RIGHT;

    if (!frontOpen && leftOpen && rightOpen) return JunctionType::T;

    if (frontOpen && leftOpen && !rightOpen)  return JunctionType::LEFT_T;
    if (frontOpen && !leftOpen && rightOpen)  return JunctionType::RIGHT_T;

    if (!frontOpen && !leftOpen && !rightOpen) return JunctionType::DEAD_END;

    return JunctionType::NONE;
}
