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

int currentID = 0;
int currentAngle = 0;
char readingArray [6];
char idarray [2] = {'3','3'};
char anglearray [4] = {'2','0','4','8'};
int counter = 0;

void setup()
{
  //Motor Setup
  //SetPosition(1,2000); //set the position of servo # 1 to '0'
  
  delay(100);//wait for servo to move
  Serial.begin(57142);
  Serial.println("poop");
  int i = 0;
}

void loop()
{
  while (Serial.available() > 0) 
  {
    //feed string -> $00!0000* .... $id!angle*
    //BE SURE TO SEND THE STRING WITH 0's EVEN IF SINGLE DIGIT
    char rd = Serial.read(); //read in the character
    String srd(rd); //convert character to string for later comparison

    //Action Structure
    if (srd == ("$")) //Beginning of reading
    {
      Serial.println("Cash Confirmed");
      counter = 0;
    }
    else if(srd == ("!")) //start reading angle
    {
      counter = 2;
      Serial.println("Exclamation Confirmed");
    }
    else if(srd == ("*")) //packet over, write angle to motor
    {
      //convert arrays to strings
      String sid(idarray);
      String sangle(anglearray);
     
      //convert strings to numbers
      currentID = sid.toInt();
      currentAngle = sangle.toInt();
      
      SetPosition(currentID,currentAngle);//set angle of motor
      
      //print for debug
      Serial.println("Set Reached");
      Serial.print(sid); Serial.print("|");
      Serial.println(sangle);
      
    }
    else if(true) //put in a number check
    {
      if (counter < 2)
      {
        idarray[counter] = rd; //add number to id array
        Serial.print("idarray value set to: "); Serial.println(idarray[counter]);
        counter = counter + 1;
      }
      else if (counter < 6)
      {
        anglearray[(counter-2)] = rd; //add number to angle array
        Serial.print("anglearray value set to: "); Serial.println(anglearray[(counter-2)]);
        counter = counter + 1;
      }
    }
    else
    {
      Serial.println(srd.toInt());
      Serial.println("Something went wrong");
      PrintDebugInfo("ON");
    }
    
  }
}

void PrintDebugInfo(String bug)
{
  if (bug == "ON")
  { 
    //prints "Debug | counter | idarray | anglearray
    Serial.println("Debug | ");
    Serial.print(counter);Serial.print(" | ");Serial.print(idarray);
    Serial.print(" | ");Serial.print(anglearray);
  }
      
}


  
   




