
/*
  Braccio Communication via Serial Port
 */

#include <Braccio.h>
#include <Servo.h>
#include <string.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;
char *ptr = NULL;
int joints[6];

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
  
  // start serial port at 9600 bps and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //Ready! -> Go Home 
  joints[0] = 90;
  joints[1] = 90;
  joints[2] = 90;
  joints[3] = 90;
  joints[4] = 90;
  joints[5] = 10;
  Braccio.ServoMovement(20, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
  //Serial.println("Braccio Ready for Operation");
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

  //1. Read Serial Port
  if (Serial.available() > 0) {
    // get incoming string:
    //while (Serial.available() == 0) {}   //wait for data available
    String msgIn = Serial.readString();    //read until timeout
    msgIn.trim();                          // remove any \r \n whitespace at the end of the String
    //Serial.print("He recibido por serie: ");
    //Serial.println(msgIn); 
      
    // Separate String to Array 
    char *strings[10];
    byte index = 0;
    const char *delimiter =" ";
    ptr = strtok(msgIn.c_str(), delimiter);
    while (ptr != NULL)
    {
      strings[index] = ptr;
      index++;
      ptr = strtok(NULL, delimiter);
    }

    // Check CMD
    String cmd = String(strings[0]);
    if (cmd == "JOINTS")    // important, Single quotes for char*, double quotes for String
    {
      //JOINTS T1 T2 T3 T4 T5 T6
      // send joint values to Braccio:
      //Serial.println("CMD JOINTS detectado!. Setting new values. ");
      joints[0] = atoi(strings[1]);
      joints[1] = atoi(strings[2]);
      joints[2] = atoi(strings[3]);
      joints[3] = atoi(strings[4]);
      joints[4] = atoi(strings[5]);
      joints[5] = atoi(strings[6]);
      Braccio.ServoMovement(20, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
      Serial.println("ok");
    }
    else if (cmd == "STATUS")    // important, Single quotes for char*, double quotes for String
    {
      // send current joint values to Serial:      
      //Serial.println("STATUS detectado!.");
      char writeBuffer[80];
      //sprintf(writeBuffer,"JOINTS %.2f %.2f %.2f %.2f %.2f %.2f", joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
      sprintf(writeBuffer,"%d %d %d %d %d %d", joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
      Serial.println(writeBuffer);
    }
    else
    {
      Serial.println("error");
      //Serial.print("CMD no reconocido: ");
      //Serial.println(strings[0]);
    }
  }
}
