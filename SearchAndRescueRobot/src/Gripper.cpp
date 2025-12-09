#include "Gripper.h"
#include <Arduino.h>

void Gripper::begin(uint8_t gripPin, uint8_t tiltPin){
    gripServ.attach(gripPin);   // continuous servo (gripper)
    tiltServ.attach(tiltPin);   // standard servo (tilt)
    tiltServ.write(SERVO_NORM_ANGLE);
    gripServ.write(SERVO_RELEASE_SPEED);   // e.g. 180 = full speed open

    unsigned long start = millis();
    while (millis() - start < SERVO_RELEASE_TIME) {
        // blocking but simple – fine for now
    }
    
    
    

}

void Gripper::pickup(){
    // 1. Tilt robot down
    tiltServ.write(SERVO_TILT_ANGLE);
    delay(SERVO_TILT_TIME);   // wait until tilt is down

    // 2. Close gripper (continuous servo)
    gripServ.write(SERVO_GRIP_SPEED);  // e.g. 0 = full speed close
    delay(SERVO_GRIP_TIME);           // let it close
  

    // 3. Tilt robot back up
    tiltServ.write(SERVO_NORM_ANGLE);
    delay(SERVO_TILT_TIME);          // wait until back upright
}

void Gripper::release(){
    // Open gripper (continuous servo other direction)
    gripServ.write(SERVO_RELEASE_SPEED);   // e.g. 180 = full speed open

    unsigned long start = millis();
    while (millis() - start < SERVO_RELEASE_TIME) {
        // blocking but simple – fine for now
    }

    gripServ.write(90);  // stop gripper
}
