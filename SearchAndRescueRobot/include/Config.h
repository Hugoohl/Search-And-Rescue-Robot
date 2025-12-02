#pragma once


// DC motors
constexpr int DC_MOTOR_MAX_SPEED = 255;
constexpr int DC_MOTOR_BASE_SPEED = 100;
constexpr int DC_MOTOR_TURN_SPEED = 0;

// Line PID
constexpr double IR_KP = 0.1;
constexpr double IR_KI = 0;
constexpr double IR_KD = 0;

//Line values

constexpr uint16_t CENTER_POS = 1500; // for 4 sensors
constexpr uint16_t CALIBRATION_TIME = 4000; //ms
constexpr uint16_t JUNCTION_THRESHOLD = 500;

//UltraSonic Thresholds