//Version 1.0
//Date: 11/25/2014
//
//Description: For controlling mechatronic arm with joysticks. It features 6 servos controlled by PWM.
//Uses 2 Sainsmart Joysticks. Joysticks have 2 trimpots each and one button each. Each trimpot is used for
//one servo. The two buttons are used for one servo. Rotation of the base is handled by a fifth trimpot.
//Desired Future Updates: Optimization. Mode switch.
//
//Equipment: Arduino Mega

///////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <I2C_Anything.h>
// Servos array in order from bottom of the arm up
// Instantiate array
// shoulderRotate : Servo 1 - motor[0] | No Indicator | Trimpot Rotate | A0
// shoulderBend   : Servo 2 - motor[1] | LEDs 2 & 3 | R. Joystick RL | A1
// elbow          : Servo 3 - motor[2] | LEDs 4 & 5 | R. Joystick UD | A2
// wristBend      : Servo 4 - motor[3] | LEDs 6 & 7 | L. Joystick UD | A3
// wristRotate    : Servo 5 - motor[4] | LEDs 8 & 9 | L. Joystick RL | A4
// gripper        : Servo 6 - motor[5] | LEDs 22 & 23| RL. Joystick Press | 36,37
// gripper        : Servo 6 - motor[5] | LEDs 22 & 23 | A5,A6 (Digital Button Not Working)
//
int LEDS[] = {2,4,6,8,3,5,7,9,22,23,24};  // the number of the LED pin
int BUTTONS[] = {36,37,38,39,40};
long fade[10];
long pos[6];
int p = 0;
int pp = 0;
long curval;
int rightbutton = 0;
int leftbutton = 0;
long gripval = 90;

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  Wire.begin(30);
  Wire.onRequest(requestEvent);
  
 for (int p = 0; p < 11; p++)
 {
   pinMode(LEDS[p],OUTPUT);
 }
 
 for (int pp = 0; pp < 5; pp++)
 {
   pinMode(BUTTONS[pp],INPUT);
 }

 //Serial.begin(9600);
 
 digitalWrite(LEDS[10],HIGH);
}

///////////////////////////////////////////////////////////////////////////////

void loop()
{
  for (int k = 0; k < 5; k++)
  {
    curval = analogRead(k);
    if (k == 0)
    {
      //Serial.println(curval);
      pos[k] = map(curval, 0, 1023, 0, 179); //servo positioning
    }
    else
    {
      pos[k] = map(curval, 0, 1023, 179, 0); //servo positioning
      
      if (curval < 475)
      {
        analogWrite(LEDS[(k-1)],map(curval, 0, 509, 255, 0));
        analogWrite(LEDS[(k+3)],0);
      }
      else if (curval > 520)
      {
        analogWrite(LEDS[(k-1)],0);
        analogWrite(LEDS[(k+3)],map(curval, 513, 1023, 0, 255));
      }
      else
      {
        analogWrite(LEDS[(k-1)],0);
        analogWrite(LEDS[(k+3)],0);
      }
    }
  }
 
  delay(10);
  
  ////// Analog Read Joystick Button ////////
  rightbutton = analogRead(5);
  leftbutton = analogRead(6);

  while (rightbutton < 0.01)
  {
    digitalWrite(LEDS[8],HIGH);
    if (gripval < 179)
    {
      gripval = gripval + 1;
      //Serial.println(gripval);
    }
    rightbutton = analogRead(5);
  }
  
  digitalWrite(LEDS[8],LOW);
  
   while (leftbutton < 0.01)
  {
    digitalWrite(LEDS[9],HIGH);
    if (gripval > 0)
    {
      gripval = gripval - 1;
      //Serial.println(gripval);
    }
    leftbutton = analogRead(6);
  }
  
  digitalWrite(LEDS[9],LOW);
  
  pos[5] = gripval;
  
////// Digital Read Joystick Button - NOT WORKING ////////
//  if (button0 == LOW)
//  {
//    digitalWrite(LEDS[8],HIGH);
//    //Serial.println("Button 0 Low");
//  }
//  else
//  {
//    digitalWrite(LEDS[8],LOW);
//    //Serial.println("Button 0 High");
//  }
//  
//  if (button1 == LOW)
//  {
//    digitalWrite(LEDS[9],HIGH);
//    //Serial.println("Button 1 Low");
//  }
//  else
//  {  
//    digitalWrite(LEDS[9],LOW);
//    /////Serial.println("Button 1 High");
//  }
////// Digital Read Joystick Button - NOT WORKING ////////
  //Serial.println(pos[0]);
  delay(10);
}

void requestEvent()
{
  Serial.print
  long sendie = 100;
  I2C_writeAnything(sendie);
}
///////////////////////////////////////////////////////////////////////////////
