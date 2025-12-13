#pragma once

// ---------------- Motors ----------------
constexpr int DC_MOTOR_MAX_SPEED = 255;
constexpr int DC_MOTOR_BASE_SPEED = 70;      // moderate base speed
constexpr int DC_MOTOR_TURN_SPEED = 0;       // used only in special cases
constexpr int DC_MOTOR_CALIB_SPEED = 60;

// Motor inversion flags (set based on wiring)
constexpr bool LEFT_MOTOR_INVERTED  = false;
constexpr bool RIGHT_MOTOR_INVERTED = true;  // your driver previously inverted right :contentReference[oaicite:0]{index=0}

// Allow small reverse during PID to reduce “slam to 0” oscillations
constexpr bool PID_ALLOW_REVERSE = true;
constexpr int  PID_MIN_SPEED = PID_ALLOW_REVERSE ? -40 : 0;

// ---------------- Line PID ----------------
// Start stable: tune P first, then add small D
constexpr double IR_KP = 0.1;
constexpr double IR_KI = 0.00;
constexpr double IR_KD = 0.01 ;

// Max correction added/subtracted from base speed
constexpr int PID_MAX_SPEED = 70;

// Position filtering (reduces noise -> less D-triggered wobble)
constexpr float LINE_POS_ALPHA = 0.35f;  // 0..1 (higher = less smoothing)

// ---------------- Line values ----------------
constexpr uint16_t CENTER_POS = 1500;          // for 4 sensors
constexpr uint16_t CALIBRATION_TIME = 6000;    // ms
constexpr uint16_t JUNCTION_THRESHOLD = 500;
constexpr uint16_t JUNCTION_THRESHOLD_SINGLE = 900;

// Debounce so angled entries don’t false-trigger
constexpr uint16_t JUNCTION_DEBOUNCE_MS = 60;

// ---------------- Ultrasonic thresholds ----------------
constexpr uint16_t MAX_DISTANCE = 400;
constexpr uint8_t THRESHOLD_DISTANCE = 15;
constexpr uint8_t LOST_WALL = 25;
constexpr uint8_t CYLINDER_DIST = 5;

// Non-blocking sonar update rate (ms between pings per sensor)
constexpr uint8_t SONAR_PING_PERIOD_MS = 35;

// ---------------- Turning safety ----------------
constexpr uint16_t ROTATE_TIMEOUT_MS = 1400;
constexpr uint16_t ROTATE_EXIT_BLACK_TIMEOUT_MS = 500;

// ---------------- Servos ----------------
constexpr uint8_t  SERVO_GRIP_SPEED    = 0;
constexpr uint8_t  SERVO_RELEASE_SPEED = 180;
constexpr uint16_t SERVO_RELEASE_TIME  = 400;

constexpr uint8_t  SERVO_TILT_ANGLE = 50;
constexpr uint8_t  SERVO_NORM_ANGLE = 120;

constexpr uint16_t SERVO_TILT_TIME = 700;
constexpr uint16_t SERVO_GRIP_TIME = 500;

constexpr uint16_t JUNCTION_BREAK = 500;


