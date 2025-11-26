#pragma once

enum class RobotState {
    WAIT_FOR_START,
    CALIBRATING,
    RUN_MISSION,
};

enum class JunctionType {
    NONE,
    LEFT,
    RIGHT,
    T,
    CROSS,
    DEAD_END,

};