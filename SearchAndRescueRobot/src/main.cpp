#include <Arduino.h>
#include "config.h"
#include "pins.h"




int lSpeed = 0;
int rSpeed = 0;

void setup() {
  Serial.begin(9600); // send and receive at 9600 baud
  pinMode(MOTOR_PIN_M2A, OUTPUT);
  pinMode(MOTOR_PIN_M2B, OUTPUT);
  while (!Serial) ;
  Serial.println("enter speed between -100 and 100, on the form: 50,50  ");
}



void runMotors(int leftSpeed,int rightSpeed){ //Function to run the motors, the Speeds can be set between -100 and 100
  if(rightSpeed>=0 and rightSpeed != 0){
    int Speed = map(rightSpeed, 0, 100, 0, 255);
    analogWrite(MOTOR_PIN_M2A, Speed);
    digitalWrite(MOTOR_PIN_M2B, LOW);    
  }
  if(rightSpeed<=0 and rightSpeed != 0){
    int Speed = map(rightSpeed, -100, 0, 255, 0);
    digitalWrite(MOTOR_PIN_M2A, LOW);
    analogWrite(MOTOR_PIN_M2B, Speed);
  }
  if(rightSpeed == 0){
   digitalWrite(MOTOR_PIN_M2A, LOW);
   digitalWrite(MOTOR_PIN_M2B, LOW);
  }
  if(leftSpeed>=0 and leftSpeed != 0){
    int Speed = map(leftSpeed, 0, 100, 0, 255);
    analogWrite(MOTOR_PIN_M1B, Speed);
    digitalWrite(MOTOR_PIN_M1A, LOW); 
  }
  if(leftSpeed<=0 and leftSpeed != 0){
    int Speed = map(leftSpeed, -100, 0, 255, 0);
    digitalWrite(MOTOR_PIN_M1B, LOW);
    analogWrite(MOTOR_PIN_M1A, Speed);
  }
  if(leftSpeed == 0){
    digitalWrite(MOTOR_PIN_M1B, LOW);
    digitalWrite(MOTOR_PIN_M1A, LOW);   
  }  
}

void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readString();
    inputString.trim();
    int commaindex = inputString.indexOf(",");
    lSpeed =(inputString.substring(0,commaindex)).toInt();
    rSpeed =  (inputString.substring(commaindex+1) ).toInt();
    }  
    runMotors(lSpeed, rSpeed);     
}
