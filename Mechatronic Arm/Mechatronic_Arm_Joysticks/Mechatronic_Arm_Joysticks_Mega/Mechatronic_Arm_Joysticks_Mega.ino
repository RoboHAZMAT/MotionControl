//Version 1.0
//Date: 11/16/2014
//
//Description: For controlling mechatronic arm with joysticks. It features 6 servos controlled by PWM.
//Uses 2 Sainsmart Joysticks. Joysticks have 2 trimpots each and one button each. Each trimpot is used for
//one servo. The two buttons are used for one servo. Rotation of the base is handled by a fifth trimpot.
//Desired Future Updates: Optimization. Mode switch.
//
//Equipment: Arduino Mega

///////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
// Servos array in order from bottom of the arm up
// Instantiate array
// shoulderRotate : Servo 1 - motor[0] | No Indicator | Trimpot Rotate | A0
// shoulderBend   : Servo 2 - motor[1] | LEDs 2 & 3 | R. Joystick RL | A1
// elbow          : Servo 3 - motor[2] | LEDs 4 & 5 | R. Joystick UD | A2
// wristBend      : Servo 4 - motor[3] | LEDs 6 & 7 | L. Joystick UD | A3
// wristRotate    : Servo 5 - motor[4] | LEDs 8 & 9 | L. Joystick RL | A4
// gripper        : Servo 6 - motor[5] | LEDs 22 & 23| RL. Joystick Press | 36,37
//
int LEDS[] = {2,4,6,8,3,5,7,9,22,23,24};  // the number of the LED pin
int BUTTONS[] = {36,37,38,39,40};
long fade[10];
long pos[6];
int p = 0;
int pp = 0;
long curval;
int button0 = 0;
int button1 = 0;
///////////////////////////////////////////////////////////////////////////////

void setup()
{
// for (int p = 0; p < 11; p++)
// {
//   pinMode(LEDS[p],OUTPUT);
// }
// 
// for (int pp = 0; pp < 5; pp++)
// {
//   pinMode(BUTTONS[pp],INPUT);
// }
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(22,OUTPUT);
  pinMode(23,OUTPUT);
  pinMode(24,OUTPUT);
  
  pinMode(36,INPUT);
  pinMode(37,INPUT);
  pinMode(38,INPUT);
  pinMode(39,INPUT);
  pinMode(40,INPUT);
 Serial.begin(9600);
 
}

///////////////////////////////////////////////////////////////////////////////

void loop()
{
  for (int k = 0; k < 5; k++)
  {
    curval = analogRead(k);
    if (k == 0)
    {
      pos[k] = map(curval, 0, 1023, 0, 179); //servo positioning
      button0 = digitalRead(36);
  button1 = digitalRead(37);
  Serial.println(button0);
  Serial.println(button1);
  Serial.println("k=0");
    }
    else
    {
      button0 = digitalRead(36);
  button1 = digitalRead(37);
  Serial.println(button0);
  Serial.println(button1);
  Serial.println("k>0");
  Serial.println(k);
      pos[k] = map(curval, 0, 1023, 179, 0); //servo positioning
      
      if (curval < 500)
      {
        analogWrite(LEDS[(k-1)],map(curval, 0, 509, 255, 0));
        analogWrite(LEDS[(k+3)],0);
        button0 = digitalRead(36);
  button1 = digitalRead(37);
  Serial.println(button0);
  Serial.println(button1);
      }
      else if (curval > 530)
      {
        analogWrite(LEDS[(k-1)],0);
        analogWrite(LEDS[(k+3)],map(curval, 513, 1023, 0, 255));
        button0 = digitalRead(36);
  button1 = digitalRead(37);
  Serial.println(button0);
  Serial.println(button1);
      }
      else
      {
        analogWrite(LEDS[(k-1)],0);
        analogWrite(LEDS[(k+3)],0);
        button0 = digitalRead(36);
  button1 = digitalRead(37);
  Serial.println(button0);
  Serial.println(button1);
      }
    }
  }
  
  delay(10);
  
  button0 = digitalRead(36);
  button1 = digitalRead(37);
  Serial.println(button0);
  Serial.println(button1);
  Serial.println("------");
  delay(10000);
  
  if (button0 == LOW)
  {
    digitalWrite(LEDS[8],HIGH);
    //Serial.println("Button 0 Low");
  }
  else
  {
    digitalWrite(LEDS[8],LOW);
    //Serial.println("Button 0 High");
  }
  
  if (button1 == LOW)
  {
    digitalWrite(LEDS[9],HIGH);
    //Serial.println("Button 1 Low");
  }
  else
  {  
    digitalWrite(LEDS[9],LOW);
    /////Serial.println("Button 1 High");
  }

 delay(10);
}

///////////////////////////////////////////////////////////////////////////////
