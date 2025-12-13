#pragma once
#include <Arduino.h>
#include <NewPing.h>
#include "Types.h"
#include "Config.h"

class UltraSonic
{
public:
    UltraSonic();

    void begin(uint8_t trigPinRight, uint8_t trigPinLeft, uint8_t trigPinFront,
               uint8_t echoPinRight, uint8_t echoPinLeft, uint8_t echoPinFront,
               unsigned int maxDistanceCm);

    // call this frequently (each loop)
    void update();

    // cached distances

    unsigned int readDistRight() const { return _dR; }
    unsigned int readDistLeft() const { return _dL; }
    unsigned int readDistFront() const { return _dF; }
    JunctionType getJunctionStable() const;
    mutable unsigned long _lastNonNoneMs = 0;

    JunctionType getJunction() const;
    bool cylinderDetected() const;

private:
    NewPing _sonarRight;
    NewPing _sonarLeft;
    NewPing _sonarFront;

    unsigned int _dR = MAX_DISTANCE;
    unsigned int _dL = MAX_DISTANCE;
    unsigned int _dF = MAX_DISTANCE;

    uint8_t _rrIndex = 0;
    unsigned long _lastPingMs = 0;

    unsigned int sanitize(unsigned int d) const
    {
        if (d == 0 || d > MAX_DISTANCE)
            return MAX_DISTANCE;
        return d;
    }

    mutable JunctionType _stableJ = JunctionType::NONE;
    mutable JunctionType _lastJ = JunctionType::NONE;
    mutable uint8_t _sameCount = 0;

    
};
