#include <Servo.h>    // Use Servo library
//Version 1.0
//Date: 11/12/2014
//
// Cleaned up code for optimizing speed.
//
//Description: This is the first version of the controls code for my 
//mechatronic arm. It features 6 servos controlled by PWM via 6 potentiometers.
//There are no feedback sensors in this version (i.e. it is an Open Feedback 
//system).
//Desired Future Updates: Different controls to enable use of analog pins for
//sensors. Custom Function for each servo reading. Ability to remember patterns.
//Ability to add delays.
//Equipment: Arduino Uno

///////////////////////////////////////////////////////////////////////////////

// Servos array in order from bottom of the arm up
// Instantiate array
// shoulderRotate : Servo 1 - A0
// shoulderBend   : Servo 2 - A1
// elbow          : Servo 3 - A2
// wristBend      : Servo 4 - A3
// wristRotate    : Servo 5 - A4
// gripper        : Servo 6 - A5
Servo motor[6];


///////////////////////////////////////////////////////////////////////////////

void setup()
{
  //Connect servos to pwm pins
  motor[0].attach(11);
  motor[1].attach(10);
  motor[2].attach(9);
  motor[3].attach(6);
  motor[4].attach(5);
  motor[5].attach(3);
}

///////////////////////////////////////////////////////////////////////////////

void loop()
{
  for (int k = 0; k < 6; k++) {
    motor[k].write(map(analogRead(k), 0, 1023, 0, 179));
  }
  delay(50);
}

///////////////////////////////////////////////////////////////////////////////
