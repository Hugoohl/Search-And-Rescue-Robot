#include <Arduino.h>
#include "Config.h"
#include "Pins.h"
#include "MotorDriver.h"
#include "LineControl.h"
#include "Types.h"

MotorDriver motor;
LineControl line;

int leftSpeed = 0;
int rightSpeed = 0;



void setup()
{

  Serial.begin(9600);

  motor.begin(MOTOR_PIN_M1A, MOTOR_PIN_M1B, MOTOR_PIN_M2A, MOTOR_PIN_M2B);

  uint8_t arrayPins[] = {IR_ARRAY_PIN1, IR_ARRAY_PIN2, IR_ARRAY_PIN3, IR_ARRAY_PIN4};
  
  line.begin(arrayPins, IR_PIN);

  line.calibrate();
}

void loop()
{
 JunctionType type = line.detectJunction();


}

