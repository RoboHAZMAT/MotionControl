Motion Control
=============

Code for the intuitive motion control for the robot. This repo contains all of the code needed to use the wearable motion control interface. Main features are the Motion Control Glove code and the IMU sensor reading programs.

###**Directories:**

  - **HandMotionControlOpt:** An optimized version of the code needed to operate the robot hands using the flex sensor based gloves. Takes the voltage drop readings from the flex sensor in the analog pins and relates that to servo motor rotation angles. Has a calibration method for different sized hands with a default setting as well. Green, yellow, and red LED status lights are used. Written for an Arduino Lilypad.

  - **IMUReading:** Contains several library files developed by Jeff Rowberg to deal with reading in the MPU 9150 IMU sensor we are using as well as the DMP. Two versions of IMU reading programs are functional, one which uses the DMP and one that uses the raw gyroscope and accelerometer data along with our custom Kalman Filter.
  
  - **Kalman Filter MATLAB:** Old code written for Dr. Tomonari Furukawa's Bayesian Robotics course implementing a one DOF Kalman Filter for accurate yaw estimates and drift cancelling.
  
  - **Mechtronic Arm:** In progress directory for controlling the Mechatronic Arm in various ways for testing.
  
  - **Tactile Feedback:** In progress directory used for tactile feedback code. The plan is to use a pressure sensor at the tips of the gripper fingers that senses when the gripper is touching an object. The pressure is related so some sort of tactile feedback method, either small inflatable pressure bags on the glove fingers or small vibration motors. 
  
  - **I2CDev:** Added standard library for communication over I2C on Arduino.
