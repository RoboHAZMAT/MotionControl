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
//#include <SoftwareSerial.h>


String srd = "";
volatile int currentID = 0;
volatile int currentAngle = 0;
char readingarray [7];
char idarray [2] = {
  '3','3'};
char anglearray [4] = {
  '2','0','4','8'};
int motorarray [6] = {
  4,5,6,14,15,16};
int i = 0;
int positions [6] = {2048,2048,2048,2048,2048,2048};

void setup()
{
  //Serial Setup
  Serial.begin(57142);
  //Serial1.begin(57600);

  //Set Up Motors
  for (int i = 0; i < 6; i++)
  {
    SetLED(motorarray[i],1);
    SetP(motorarray[i],4);
    //SetI(motorarray[i],4);
    //SetD(motorarray[i],4);
    SetPosition(motorarray[i],2048);
    //EEPromLock(motorarray[i],1);
    delay(10);
  }

}

void loop()
{
  for (int i = 0; i < 6; i++)
  {
    SetPosition(motorarray[i],positions[i]);
    delay(2);
  }
}

void serialEvent()
{
  int j = 0;
  while (Serial.available() > 0)
  {
      char thevar = Serial.read();     
      readingarray[j] = thevar;
      j++;
      delay(1);
  }
  
  //Serial.println("Hi, I am Arbotix");
  Serial.println(String(readingarray));
  Serial.println(millis());
  
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
  
  //Adds the angle of the current id to the correct place
  //in the positions array
  for (int i = 0; i < 6; i++)
  {
    if (motorarray[i] == currentID)
    {
      positions[i] = currentAngle;
      break;
    }
    delay(2);
  }
  
  //Set motor positions
  for (int i = 0; i < 6; i++)
  {
    SetPosition(motorarray[i],positions[i]);
    delay(2);
  }
  
  //reset variables
  //memset(readingarray, 0, sizeof(readingarray));
}
