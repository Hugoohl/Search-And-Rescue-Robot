#pragma once

enum class RobotState {
    WAIT_FOR_START,
    CALIBRATING,
    RUN_MISSION,
    MISSION_DONE,
};

enum class JunctionType {
    NONE,
    LEFT_T,
    RIGHT_T,
    T,
    CROSS,
    DEAD_END,
};

enum class MissionState{
    SEARCH_RIGHT,
    PICKUP,
    SEARCH_LEFT,
    RETURN,
    RELEASE,

};