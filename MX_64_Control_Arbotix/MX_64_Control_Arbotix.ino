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
    char rd = Serial.read();
    String srd(rd);
    //Serial.println(rd);
    //Serial.println(srd);
    if (srd == ("$")) //begin reading
    {
      Serial.println("Cash Confirmed");
      counter = 0;
    }
    else if(srd == ("!")) //start reading number
    {
      counter = 2;
      Serial.println("Exclamation Confirmed");
    }
    else if(srd == ("*")) //packet over, write angle to motor
    {
      Serial.println("Set Reached");
      String sid(idarray);
      //Serial.println(idarray);
      //Serial.print(idarray[0]); Serial.println(idarray[1]);
      Serial.print(sid); Serial.print("|");
      currentID = sid.toInt();
      //Serial.println(currentID + 1);
      String sangle(anglearray);
      Serial.println(sangle);
      //Serial.println(anglearray);
      currentAngle = sangle.toInt();
      SetPosition(currentID,currentAngle);
      
    }
    else if(true) //put in a number check
    {
      if (counter < 2)
      {
        idarray[counter] = rd;
        Serial.print("idarray value set to: "); Serial.println(idarray[counter]);
        counter = counter + 1;
      }
      else if (counter < 6)
      {
        anglearray[(counter-2)] = rd;
        Serial.print("anglearray value set to: "); Serial.println(anglearray[(counter-2)]);
        counter = counter + 1;
      }
    }
    else
    {
      Serial.println(srd.toInt());
      Serial.println("Somethin' wrong ho");
    }
    
  }
}


  
   




