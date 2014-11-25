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

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  //I2C setup
  Wire.begin(30);    //join I2C bus with address #30 
  Wire.onReceive(receiveEvent); //register event
  
  //Connect servos to pwm pins
  motor[0].attach(11);
  motor[1].attach(10);
  motor[2].attach(9);
  motor[3].attach(6);
  motor[4].attach(5);
  motor[5].attach(3);
  
  //I2C
 long rd[] = {90,90,90,90,90,90}; //initiate array at resting position in case comm does
                           //not start right away
}

///////////////////////////////////////////////////////////////////////////////

void loop()
{
  
//Loops through each servo reading (except the gripper which is handled digitally)
  for (int k = 0; k < 6; k++) {
    motor[k].write(rd[k]);
  }
  delay(15);
}

///////////////////////////////////////////////////////////////////////////////

void receiveEvent(int howMany)
{
  p=0; //counting variable
  while(Wire.available() > 0)
  {
    rd[p] = Wire.read();
    p = p+1;
  }
  // intended to recieve an array indicating the positions of the servos
  
}
