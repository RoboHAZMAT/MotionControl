//Version 1.1
//Date: 11/23/2014
//
// Description: For controlling mechatronic arm with joysticks. It features 6 servos controlled by PWM.
// Uses 2 Sainsmart Joysticks. Joysticks have 2 trimpots each and one button each. Each trimpot is used for
// one servo. The two buttons are used for one servo. Rotation of the base is handled by a fifth trimpot.
// This portion of the code handles the movement of the servos. The Mega portion of the code handles the 
// joystick input and the led feedback. This version uses pure I2C. However, wireless communication wlll
// not be possible using I2C and wireless serial communication will be needed in the future.
//
// Desired Future Updates: Optimization. Control Mode switch. Wireless serial communication. Comm mode switch.
//
// Equipment: Arduino Uno, 2 4.7k ohm resisitors (for pull-up on GND and +5V lines)

///////////////////////////////////////////////////////////////////////////////
//Libraries
#include <Servo.h>  //Servo library
#include <Wire.h>   //I2C library
#include <I2C_Anything.h>

// Servos array in order from bottom of the arm up
// shoulderRotate : Servo 1 - motor[0]
// shoulderBend   : Servo 2 - motor[1]
// elbow          : Servo 3 - motor[2]
// wristBend      : Servo 4 - motor[3]
// wristRotate    : Servo 5 - motor[4]
// gripper        : Servo 6 - motor[5]

// Instantiate arrays (servo and position)
Servo motor[6];
long rd[6];
int p = 0;
long rec = 0;
///////////////////////////////////////////////////////////////////////////////

void setup()
{
  //I2C setup
  Wire.begin(30);    //join I2C bus with address #30 
  Wire.onReceive(receiveEvent); //register event
  Serial.begin(9600);
  
  //Connect servos to pwm pins
  motor[0].attach(11);
  motor[1].attach(10);
  motor[2].attach(9);
  motor[3].attach(6);
  motor[4].attach(5);
  motor[5].attach(3);
  
  //I2C
// for (int n = 0; n < 6; n++)
// {
//   rd[n] = 90;
// }
  rd[0] = 90;
  rd[1] = 90;
  rd[2] = 100;
  rd[3] = 40;
  rd[4] = 90;
  rd[5] = 45;
 //initiate array at resting position in case comm does
                           //not start right away
 long rec = 0;  
                         
}

///////////////////////////////////////////////////////////////////////////////

void loop()
{
  
//Loops through each servo reading (except the gripper which is handled digitally)
Wire.requestFrom(30,44);
  for (int k = 0; k < 6; k++) {
    motor[k].write(rd[k]);
  }
  delay(15);
  Serial.println(rd[0]);
  Serial.println(rd[1]);
  Serial.println(rd[2]);
  Serial.println(rd[3]);
  Serial.println(rd[4]);
  Serial.println(rd[5]);
}

///////////////////////////////////////////////////////////////////////////////

void receiveEvent(int howMany)
{
  p=0; //counting variable
  I2C_readAnything(rec);
  Serial.print("poop");
  Serial.print(rec);
//  while(Wire.available() > 0)
//  {
//    rd[p] = Wire.read();
//    p = p+1;
//  }
  // intended to recieve an array indicating the positions of the servos
  
}
