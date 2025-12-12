#include <Arduino.h>
#include "Config.h"
#include "Pins.h"
#include "MotorDriver.h"
#include "LineControl.h"
#include "Types.h"
#include "Navigator.h"
#include <QTRSensors.h>
#include "UltraSonic.h"
#include "Gripper.h"

MotorDriver motor;
LineControl line;
UltraSonic sonar;
Gripper serv;

Navigator nav(line, motor, sonar, serv);

void setup()
{
  Serial.begin(9600);
  motor.begin(MOTOR_PIN_M1A, MOTOR_PIN_M1B, MOTOR_PIN_M2A, MOTOR_PIN_M2B);
  uint8_t arrayPins[] = {IR_ARRAY_PIN1, IR_ARRAY_PIN2, IR_ARRAY_PIN3, IR_ARRAY_PIN4};
  uint8_t singlePin[] = {IR_PIN};
  line.begin(arrayPins, singlePin);
  serv.begin(GRIP_SERVO_PIN, TILT_SERVO_PIN);
  sonar.begin(US_PIN_TRIG_RIGHT,US_PIN_TRIG_LEFT,US_PIN_TRIG_FRONT, US_PIN_ECHO_RIGHT, US_PIN_ECHO_LEFT, US_PIN_ECHO_FRONT, MAX_DISTANCE);
  line.calibrate();
  
  

 
  

  
}

int leftSpeed = 0;
int rightSpeed = 0;

void loop()
{
 
  line.computeSpeedsPid(leftSpeed, rightSpeed);
  motor.drive(leftSpeed, rightSpeed);


}

