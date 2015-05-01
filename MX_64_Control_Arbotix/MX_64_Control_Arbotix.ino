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
volatile int currentID1 = 0;
volatile int currentAngle1 = 0;
volatile int currentID2 = 0;
volatile int currentAngle2 = 0;
char readingarray [14];
char idarray [4] = {
  '3','3','4','4'};
char anglearray [8] = {
  '2','0','4','8','2','0','4','8'};
int motorarray [6] = {
  4,5,14,15};
char charArray[14];
int i = 0;

void setup()
{
  //Serial Setup
  Serial.begin(57142);
  //Serial1.begin(57600);
  delay(10);

  //Set Up Motors
  for (int i = 0; i < sizeof(motorarray); i++)
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
  int j = 0;
  boolean stuff = false;
  while (Serial.available() > 0)
  {
    char thevar = Serial.read();     
    readingarray[j] = thevar;
    j++;
    stuff = true;
    delay(10);
  }
  //Serial.println("Hi, I am Arbotix");
  String srd(readingarray);
  if (stuff)
  {
    stuff = false;
    Serial.println(srd);
    //Set char arrays
    idarray [0] = readingarray[0];
    idarray [1] = readingarray[1];
    anglearray [0] = readingarray[3];
    anglearray [1] = readingarray[4];
    anglearray [2] = readingarray[5];
    anglearray [3] = readingarray[6];
    
    idarray [2] = readingarray[7];
    idarray [3] = readingarray[8];
    anglearray [4] = readingarray[10];
    anglearray [5] = readingarray[11];
    anglearray [6] = readingarray[12];
    anglearray [7] = readingarray[13];
  
    //convert char arrays to strings
    String sid(idarray);
    String sid1 = sid.substring(0,2);
    String sid2 = sid.substring(2);
    String sangle(anglearray);
    String sangle1 = sangle.substring(0,4);
    String sangle2 = sangle.substring(4);
  
    //convert strings to numbers
    currentID1 = sid1.toInt();
    currentAngle1 = sangle1.toInt();
    currentID2 = sid2.toInt();
    currentAngle2 = sangle2.toInt();
  
    //Set motor angles
    SetPosition(currentID1,currentAngle1);
    SetPosition(currentID2,currentAngle2);//NOTHIN'
  }
}
