
/*
  simpleMovements.ino

 This  sketch simpleMovements shows how they move each servo motor of Braccio

 Created on 18 Nov 2015
 by Andrea Martino

 This example is in the public domain.
 */

#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

void setup() {
  //Initialization functions and set up the initial position for Braccio
  //All the servo motors will be positioned in the "safety" position:
    //Base (M1):0 degrees
    //Shoulder (M2): 40 degrees
    //Elbow (M3): 180 degrees
    //Wrist vertical (M4): 170 degrees
    //Wrist rotation (M5): 0 degrees
    //gripper (M6): 73 degrees (closed)
  Braccio.begin();

  //Go Home 
  Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 10);
}

void loop() {
   /*
   Braccio.ServoMovement(step delay, M1, M2, M3, M4, M5, M6)
   
   Step Delay: a milliseconds delay between the movement of each servo.  Allowed values from 10 to 30 msec.
   M1 = base degrees. Allowed values from 0 to 180 degrees
   M2 = shoulder degrees. Allowed values from 15 to 165 degrees
   M3 = elbow degrees. Allowed values from 0 to 180 degrees
   M4 = wrist vertical degrees. Allowed values from 0 to 180 degrees
   M5 = wrist rotation degrees. Allowed values from 0 to 180 degrees
   M6 = gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
  */
    
  //1. 
  Braccio.ServoMovement(20, 0, 90, 90, 90, 90, 50);
  delay(1000); //Wait ms

  //2. 
  Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 50);
  delay(1000); //Wait ms

  //3. 
  Braccio.ServoMovement(20, 180, 90, 90, 90, 90, 50);
  delay(1000); //Wait ms
}
