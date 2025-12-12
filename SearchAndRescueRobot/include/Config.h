#pragma once


// DC motors
constexpr int DC_MOTOR_MAX_SPEED = 255;
constexpr int DC_MOTOR_BASE_SPEED = 95;
constexpr int DC_MOTOR_TURN_SPEED = 0;
constexpr int DC_MOTOR_CALIB_SPEED = 60;



// Line PID
constexpr double IR_KP = 0.2;
constexpr double IR_KI = 0.0;
constexpr double IR_KD = 10;
constexpr int PID_MAX_SPEED = 50;


//Line values

constexpr uint16_t CENTER_POS = 1500; // for 4 sensors
constexpr uint16_t CALIBRATION_TIME = 6000; //ms
constexpr uint16_t JUNCTION_THRESHOLD = 500;
constexpr uint16_t JUNCTION_THRESHOLD_SINGLE = 900;

//UltraSonic Thresholds
constexpr uint16_t MAX_DISTANCE = 400;
constexpr uint8_t THRESHOLD_DISTANCE = 15;
constexpr uint8_t LOST_WALL = 30;
constexpr uint8_t CYLINDER_DIST = 4;


//Servos

constexpr uint8_t SERVO_GRIP_SPEED    = 0;    // continuous servo close
constexpr uint8_t SERVO_RELEASE_SPEED = 180;  // continuous servo open
constexpr uint16_t SERVO_RELEASE_TIME = 400;  // was 100, made longer

constexpr uint8_t SERVO_TILT_ANGLE =10;       // tilt down
constexpr uint8_t SERVO_NORM_ANGLE = 120;      // normal upright

// New timing constants
constexpr uint16_t SERVO_TILT_TIME = 700;     // ms: time to fully tilt
constexpr uint16_t SERVO_GRIP_TIME = 500;     // ms: time to close gripper

