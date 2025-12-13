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
    _sonarLeft = NewPing(trigPinLeft, echoPinLeft, maxDistanceCm);
    _sonarFront = NewPing(trigPinFront, echoPinFront, maxDistanceCm);

    _dR = _dL = _dF = MAX_DISTANCE;
    _rrIndex = 0;
    _lastPingMs = 0;
}

void UltraSonic::update()
{
    unsigned long now = millis();
    if (now - _lastPingMs < SONAR_PING_PERIOD_MS)
        return;
    _lastPingMs = now;

    // Ping one sensor per call (round robin)
    if (_rrIndex == 0)
        _dF = sanitize(_sonarFront.ping_cm());
    else if (_rrIndex == 1)
        _dR = sanitize(_sonarRight.ping_cm());
    else
        _dL = sanitize(_sonarLeft.ping_cm());

    _rrIndex = (_rrIndex + 1) % 3;
}

bool UltraSonic::cylinderDetected() const
{
    return (_dF < CYLINDER_DIST);
}

JunctionType UltraSonic::getJunction() const
{
    bool frontOpen = (_dF > LOST_WALL);
    bool leftOpen = (_dL > LOST_WALL);
    bool rightOpen = (_dR > LOST_WALL);

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

JunctionType UltraSonic::getJunctionStable() const
{
    constexpr uint16_t OPEN_TH = LOST_WALL;
    constexpr uint16_t CLOSE_TH = LOST_WALL - 5;

    auto isUnknown = [](uint16_t d)
    {
        return d == 0 || d >= MAX_DISTANCE;
    };

    auto openWithHyst = [&](uint16_t d, bool prevOpen)
    {
        if (isUnknown(d))
            return prevOpen;
        if (prevOpen)
            return d > CLOSE_TH;
        else
            return d > OPEN_TH;
    };

    static bool fOpen = false, lOpen = false, rOpen = false;
    fOpen = openWithHyst(_dF, fOpen);
    lOpen = openWithHyst(_dL, lOpen);
    rOpen = openWithHyst(_dR, rOpen);

    // Serial.print("front: "); Serial.println(_dF);
    // Serial.print("left: "); Serial.println(_dL);
    // Serial.print("right: "); Serial.println(_dR);

    unsigned long now = millis();

    JunctionType j = JunctionType::NONE;
    if (fOpen && lOpen && rOpen)
        j = JunctionType::CROSS;
    else if (!fOpen && !lOpen && !rOpen)
        j = JunctionType::DEAD_END;
    else if (_dF <20 && lOpen && rOpen)
        j = JunctionType::T;
    else if (fOpen && lOpen && !rOpen)
        j = JunctionType::LEFT_T;
    else if (fOpen && !lOpen && rOpen)
        j = JunctionType::RIGHT_T;
    else if (!fOpen && lOpen && !rOpen)
        j = JunctionType::LEFT;
    else if (!fOpen && !lOpen && rOpen)
        j = JunctionType::RIGHT;

    if (j == _lastJ && j != JunctionType::NONE)
        _sameCount++;
    else
        _sameCount = 0;

    _lastJ = j;
    if (_sameCount >= 2)
        _stableJ = j;

    if (j == _lastJ && j != JunctionType::NONE)
        _sameCount++;
    else
        _sameCount = 0;

    _lastJ = j;

    // If we got a stable non-NONE, accept it and remember time
    if (_sameCount >= 2 && j != JunctionType::NONE)
    {
        _stableJ = j;
        _lastNonNoneMs = now;
    }

    // If we keep seeing NONE for a while, release back to NONE
    if (j == JunctionType::NONE && (now - _lastNonNoneMs) > 250)
    {
        _stableJ = JunctionType::NONE;
    }

    return _stableJ;
}
