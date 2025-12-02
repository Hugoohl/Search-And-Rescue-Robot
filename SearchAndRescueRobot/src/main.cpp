#include <Arduino.h>
#include "Config.h"
#include "Pins.h"
#include "MotorDriver.h"
#include "LineControl.h"
#include "Types.h"
#include "Navigator.h"
#include <QTRSensors.h>

MotorDriver motor;
LineControl line;
Navigator nav(line,motor);


void setup()
{

  

  motor.begin(MOTOR_PIN_M1A, MOTOR_PIN_M1B, MOTOR_PIN_M2A, MOTOR_PIN_M2B);

  uint8_t arrayPins[] = {IR_ARRAY_PIN1, IR_ARRAY_PIN2, IR_ARRAY_PIN3, IR_ARRAY_PIN4};
  
  line.begin(arrayPins, IR_PIN);

  nav.begin();
  
  Serial.begin(9600);

}

void loop()
{
  
 
nav.update();

  


}

