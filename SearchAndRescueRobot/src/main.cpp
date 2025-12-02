#include <Arduino.h>
#include "Config.h"
#include "Pins.h"
#include "MotorDriver.h"
#include "LineControl.h"
#include "Types.h"
#include "Navigator.h"
#include <QTRSensors.h>
#include "UltraSonic.h"

MotorDriver motor;
LineControl line;
UltraSonic sonar;

Navigator nav(line, motor, sonar);

void setup()
{

  motor.begin(MOTOR_PIN_M1A, MOTOR_PIN_M1B, MOTOR_PIN_M2A, MOTOR_PIN_M2B);
  uint8_t arrayPins[] = {IR_ARRAY_PIN1, IR_ARRAY_PIN2, IR_ARRAY_PIN3, IR_ARRAY_PIN4};
  line.begin(arrayPins, IR_PIN);
  sonar.begin(US_PIN_TRIG, US_PIN_ECHO_RIGHT, US_PIN_ECHO_LEFT, US_PIN_ECHO_FRONT, MAX_DISTANCE);
  nav.begin();
  line.calibrate();

  Serial.begin(9600);
}

void loop()
{
  int leftspeed = 0;
  int rightspeed = 0;

  line.computeSpeeds(leftspeed,rightspeed);
  motor.drive(leftspeed,rightspeed);
}
