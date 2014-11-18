#include <Servo.h>
//========================== Hand Motion Control ===========================
/* RoboHAZMAT Senior Design Team
 * Motion Control Group
 * Gerardo Bledt
 * November 12, 2014
 *
 * Code written for the hand glove motion control glove for the RoboHAZMAT 
 * Senior Design project. Uses flex sensors along each of the fingers to 
 * record the amount of bend for each one and translate that to motion of 
 * the robot fingers by using the Servo motors.
 * 
 * 1. Parameter Setup: Defines and initializes all of the necessary 
 *    variables for the glove.
 *
 * 2. Functions: Contains all of the user created functions to help process
 *    the signals and control the robot.
 *    - Calibrate
 *    - MoveMotor
 *
 * 3. Main Loop: First asks the user to calibrate the flex sensors to their
 *    hand by opening and closing their hand to get the max and min range.
 *    Takes in sensor measurements from the flex sensors and calls the 
 *    function to filter out the signal and then use the filtered value for
 *    the sensor measurement to move the Servo motor position.
 * 
 *    *** TO DO ***
 *     - Optimize code for speed.
 *     - Look at making a generic program for righ and left hand control.
 *     - Documentand comment all code clearly.
 *     - Make a status function for the 3 LEDs. {?}
 *     - Make an LED variable array for easy status definitions.
 *     - Look at naming convention for readings. {?}
 *     - Add thumb sensor for gripping motion.
 */
 
//============================ Parameter Setup =============================
/* Defines and initializes all of the necessary variables, Servos, sensor
 * pins, and reading buffers. 
 * 
 * Finger Naming Definitions:
 *  - Right thumb  | Left pinky  : [0]
 *  - Right index  | Left ring   : [1]
 *  - Right middle | Left middle : [2]
 *  - Right ring   | Left index  : [3]
 *  - Right pinky  | Left thumb  : [4]
 */

// The indexing variable
int i = 0;   

// Number of flex sensors
const int numSensors = 2;

// Filtering constant 
const int numReadings = 10;   

// Status LEDs
int greenLED = 8;
int yellowLED = 9;
int redLED = 10;

// Serv o motor definitions
Servo motor[numSensors];

// Stores each of the Servo motor positions
int motorPos[numSensors]; 

// Sets up pins for each finger
int pin[numSensors];

// Sets the flex sensor max and mins
int maxR[numSensors];
int minR[numSensors];

// Instantiates reading buffers for filtering
int readingBuffer[numSensors][numReadings];

/**
 * Sets up all of the Servo motors, buffers, and Serial communication.
 */
void setup(){
  // Begins Serial communication
  Serial.begin(9600);
  
  // Sets up the LEDs
  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  
  // Red LED lit to show setup ongoing
  digitalWrite(redLED, HIGH);
  
  // Initializes the buffers
  for (int k = 0; k < numSensors; k++) {
    // Sets up the analog pins for sensor readings
    pin[k] = k + 14; // analog pins 14 - 18 (A0 - A4) 
    
    // Attaches the Servo motors to their pins
    motor[k].attach(k + 2);      
    
    // Set up of the min and max values
    maxR[k] = 0;
    minR[k] = 100000;
    
    for (int j = 0; j < numReadings; j++) {
      readingBuffer[k][j] = 0;
    }
  }
  delay(2000);
  
  // Calibrates all of the flex sensor max and mins
  Calibrate(); 
}

//=============================== Functions ================================
/* Defines all of the user created functions to be used in the main loop.
 *
 * Functions:
 * - Calibrate
 * - MoveMotor
 */
 
/**
 * Calibrates the flex sensors to find the max and min range for the user.
 */
void Calibrate() {
  // Red and yellow LED both on to notify user of calibration start
  digitalWrite(yellowLED, HIGH);
  digitalWrite(redLED, HIGH);
  
  // Delay to give user time
  delay(1000);
  
  // Red off so yellow is on signifying calibration ongoing
  digitalWrite(redLED, LOW);

  // Takes 2500 measurements to make sure true values are found
  for (int j = 0; j < 2500; j++) {
    int reading[numSensors]; 
    for (int k = 0; k < numSensors; k++) {
      // Takes reading from each sensor
      reading[k] = analogRead(pin[k]);
    
      // If reading is greater than previous max, it is new max
      // If reading is less than previous min, it is new min
      if (reading[k] > maxR[k]) {maxR[k] = reading[k];} 
      else if (reading[k] < minR[k] || minR[k] < 0) {minR[k] = reading[k];}
    }
    delay(1);
  }
  
  // Green on, red and yellow off to show ready to use
  digitalWrite(greenLED, HIGH);
  digitalWrite(yellowLED, LOW);
}

/**
 * Filters the readings for noise by averaging the previous readings for 
 * smooth sensor data.
 * Moves the corresponding sevor motor to the averaged position.
 */
int MoveMotor(int i,int reading,int readingBuffer[],Servo motor,int prevMotorPos) {
  // Calculates the past history
  int totalReading = 0;
  for (int j = 0; j < numReadings; j++) {
    totalReading = totalReading + readingBuffer[j];
  }
  int prevAverage = totalReading / numReadings;
  
  // Wraps around the buffer
  totalReading = totalReading - readingBuffer[i];
  readingBuffer[i] = reading;
  totalReading = totalReading + readingBuffer[i];
  
  // Calculates the filtered reading and translates to Servo degree range
  int motorPos = constrain(totalReading / numReadings,0,175);
  
  // Moves the motors if the new position is more than 2 degrees from the
  // previous motor position
  if (((motorPos - prevMotorPos) > 2) || ((prevMotorPos - motorPos) > 2)) {
    motor.write(motorPos);
    return motorPos;
  }
  return prevMotorPos;
}

//=============================== Main Loop ================================
/* Main running loop that takes readings for each finger, mapps them and 
 * constrains them within the servo motor's degree range. That reading is 
 * taken and used to move the corresponding Servo motor after being 
 * filtered for noisy signals.
 */
void loop() { 
  // Take the readings from each of the flex sensors on the fingers, map 
  // and constrain them in the Servo motor degree range
  int reading[numSensors];
  for (int k = 0; k < numSensors; k++) {
    reading[k] = constrain(map(analogRead(pin[k]),minR[k],maxR[k],0,175),0,175); 
    motorPos[k] = MoveMotor(i,reading[k],readingBuffer[k],motor[k],motorPos[k]); 
  }  
  // Increase the index variable, wrap around the filter constant
  i++;
  if (i >= numReadings) { i = 0; }
  
  // Delay for stability
  delay(1);
}
