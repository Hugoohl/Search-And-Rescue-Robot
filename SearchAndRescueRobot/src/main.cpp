#include <Arduino.h>
#include "config.h"
#include "pins.h"
#include "MotorDriver.h"
#include "LineSensor.h"

MotorDriver motor;
LineSensor lineSensor;


void setup() {

  motor.begin(MOTOR_PIN_M1A,MOTOR_PIN_M1B,MOTOR_PIN_M2A,MOTOR_PIN_M2B);
  lineSensor.begin(IR_ARRAY_PIN1,IR_ARRAY_PIN2,IR_ARRAY_PIN3,IR_ARRAY_PIN4, 1);
  lineSensor.calibrate();
  Serial.begin(9600);
  
  
}


void loop() {
 int position = lineSensor.followLine();
 if(position > 1000 && position < 2500){
  motor.drive(50,50);
 } else if(position <1000){
  motor.drive(50, 15);

 } else if(position >2000){
  motor.drive(15,50);
 }
    
}
