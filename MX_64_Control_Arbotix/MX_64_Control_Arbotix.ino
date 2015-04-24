//===============MX-64 Control Sketch========
/* This sketch controls the MX-motors. Firs this sketch sets up the motors
to the desired setup parameters. Then it reads in the commands from the matlab
code serially, parses the commands, and assigns the angles to the proper motors.
Written by John Gardiner
3/22/2015
*/
//import ax12 library to send DYNAMIXEL commands
#include <ax12.h>
#include <ctype.h>

String srd = "";
int currentID = 0;
int currentAngle = 0;
char readingarray [6];
char idarray [2] = {'3','3'};
char anglearray [4] = {'2','0','4','8'};
int motorarray [6] = {4,5,6,14,15,16};
int i = 0;

void setup()
{
  //Serial Setup
  Serial.begin(57142);
  delay(10);
  //Serial.println("poop");
    
  //Set Up Motors
  for (int i = 0; i < 6; i++)
  {
    SetLED(motorarray[i],1);
    //Serial.print("Set "); Serial.print(motorarray[i]); Serial.println(" LED");
    SetP(motorarray[i],4);
    //SetI(motorarray[i],4);
    //SetD(motorarray[i],4); 
    SetPosition(motorarray[i],2048);
    //EEPromLock(motorarray[i],1);
    delay(1000);       
  }
  
}

void loop()
{
  while (Serial.available() > 0) 
  {
    //feed string -> 00!0000 .... id!angle
    //BE SURE TO SEND THE STRING WITH 0's EVEN IF SINGLE DIGIT
    char reading = Serial.read();
    if (reading == '\n'){break;}
    readingarray[i] = Serial.read();
    //srd = Serial.readStringUntil('\n');
    i++;
  }
  //srd = String(readingarray);
  i = 0;
  //Set char arrays
  idarray [0] = readingarray[0];
  idarray [1] = readingarray[1];
  anglearray [0] = readingarray[3];
  anglearray [1] = readingarray[4];
  anglearray [2] = readingarray[5];
  anglearray [3] = readingarray[6];
  
  //convert char arrays to strings
  String sid(idarray);
  String sangle(anglearray);
  
  //convert strings to numbers
  currentID = sid.toInt();
  currentAngle = sangle.toInt();
  
  //Set motor Angle
  SetPosition(currentID,currentAngle);//set angle of motor
      
  //print for debug
  //Serial.println("Set Reached");
  //Serial.print(sid); Serial.print("|");
  //Serial.println(sangle);
}

void PrintDebugInfo(String bug)
{
  if (bug == "ON")
  { 
    //prints "Debug | counter | idarray | anglearray
    Serial.print("Debug | ");
    Serial.print(idarray);
    Serial.print(" | ");Serial.println(anglearray);
  }
      
}


  
   




