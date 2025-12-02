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

enum class MissionState {
    WAIT_FOR_BUTTON,
    SEARCH_RIGHT_1,
    RETURN_LEFT_1,
    SEARCH_RIGHT_2,
    RETURN_LEFT_2,
    SEARCH_LEFT_FINAL,
    RETURN_LEFT_FINAL,
    ISLAND_SEARCH,
    PICKUP,
    RELEASE,
    DONE
};