#include <Servo.h>    // Use Servo library
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

// Servos array in order from bottom of the arm up
// Instantiate array
// shoulderRotate : Servo 1 - motor[0]
// shoulderBend   : Servo 2 - motor[1]
// elbow          : Servo 3 - motor[2]
// wristBend      : Servo 4 - motor[3]
// wristRotate    : Servo 5 - motor[4]
// gripper        : Servo 6 - motor[5]
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
