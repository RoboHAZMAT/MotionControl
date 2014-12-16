//========================= IMU Sensor Processing ==========================
/* RoboHAZMAT Senior Design Team
 * Motion Control Group
 * Gerardo Bledt
 * November 19, 2014
 *
 * Makes use of the open source IMU sensor library written for the MPU 6050
 * and the MPU 9150. Communicates over I2C with the arduino to read in all of
 * the necessary values. Calibrates the sensor measurements to correct the 
 * offsets of the sensor. Reads the values, converts them to SI units, does
 * direct integration on the gyroscope data, runs the data through a Kalman
 * Filter to estimate the orientation in 3D space, and prints the results.
 * 
 * 1. Parameter Setup: Defines and initializes all of the necessary 
 *    variables, libraries, and the IMU sensor class.
 *
 * 2. Functions: Contains all of the user created functions to help process
 *    the signals from the IMU.
 *    - Calibrate
 *    - printIMU
 *    - KalmanFilter
 *    - KFPrediction
 *    - KFCorrection
 *
 * 3. Main Loop: 
 * 
 *    *** TO DO ***
 *     - Optimize code for speed.
 *     - Get magnetometer readings to work
 *     - Yaw KF correction
 *     - Fix atan limits
 *     - Adaptive covariance accounting for non gravity acceleration
 *     - Make code more efficient (reduce size: currently 12kB +)
 *     - Filtering method on the raw data to remove noise (more?)
 *     - Possibly change the final gyroAngleKF measurements to int
 *     - Possibly need to use quaternions
 */
 
//============================ Parameter Setup =============================
/* Defines and initializes all of the necessary libraries, variables, and 
 * the IMU class.
 */

// Arduino Wire library is required for the I2C communication
#include <Wire.h>
#include <I2Cdev.h>
#include <Servo.h>

// Libraries added to get IMU specific data
#include "helper_3dmath.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "AK8975.h"
//#include <MPU6050.h>

// Initializes MPU6050 class (default I2C address is 0x68)
MPU6050 IMU;
AK8975 Mag(0x0C);

// Constant for pi
const float pi = 3.141592;

// Number of readings for smoothing noise 
const int numReadings = 10;

// Indexing variable
int i = 0;

// Sets up time history variable
unsigned long timePrev = 0;

// Corrected and converted readings for the accelerometer, gyroscope, and magnetometer
float acc[3] = {0,0,0};
float gyro[3] = {0,0,0};
float mag[3] = {0,0,0};

// Total readings for each of the DOF
long accTotalReading[3] = {0,0,0};
long gyroTotalReading[3] = {0,0,0};
long magTotalReading[3] = {0,0,0};

// Buffers for each of the DOF for smoothing
float accBuffer[numReadings][3];
float gyroBuffer[numReadings][3];
float magBuffer[numReadings][3];

// Direct integration for gyroscopic angle
float gyroAngle[3] = {0,0,0};

// Kalman Filter angle Estimation
float gyroKF[3] = {0,0,0}; // roll, pitch, yaw

// Kalman Filter Covariance 
float sigmaKF[3] = {0,0,0}; // roll, pitch, yaw

// Buffer for the roll, pitch, yaw KF estimates 
float gyroBufferKF[numReadings][3]; // = {0,0,0}; // roll, pitch, yaw

// Additive roll, pitch, yaw values
float rollTotalReading = 0;
float pitchTotalReading = 0;
float yawTotalReading = 0;

// Calibration constants for the accelerometer, gyroscope, and magnetometer
float accCali[3] = {0,0,0};  // may have to add the -g 
float gyroCali[3] = {0,0,0};
float magCali[3] = {0,0,0};

Servo motorYaw;
Servo motorPitch;

Servo elbow;
Servo wrist;
Servo twist;

void setup()
{
  // Joins the I2C bus
  Wire.begin();
  
  // Initialize serial communication
  //Serial.begin(38400);
  
  // Initialize the IMU
  //Serial.println("Initializing IMU...");
  IMU.initialize();
  //Serial.println("Initializing Magnetometer...");
  Mag.initialize();
  
  // Test connection to IMU
  //Serial.println("Testing connections...");
  //Serial.println(IMU.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  //Serial.println(Mag.testConnection() ? "AK8975 connection successful" : "AK8975 connection failed");
  
  // Set up reading buffers to zeros
  for (int x = 0; x < 3; x++) {
    for (int k = 0; k < numReadings; k++) {
      accBuffer[k][x] = 0;
      gyroBuffer[k][x] = 0;
      magBuffer[k][x] = 0; 
      gyroBufferKF[k][x] = 0;
    }  
  }
  motorYaw.attach(10);
  motorPitch.attach(9);
  elbow.attach(6);
  wrist.attach(5);
  twist.attach(3);
  twist.write(105);
  elbow.write(150);
  wrist.write(30);

  // Calibrates the IMU 
  boolean calibrated = false;
  while (!calibrated) {
    //Serial.println("Calibrating IMU Readings...");
    calibrated = Calibrate();
  }
  timePrev = millis();
}

// ========================================================================

/**
 * Calibration method for the IMU. The method assumes the user will keep 
 * the IMU steady throughout calibration. Takes a preset amount of readings
 * and uses those to find an average and estimate the natural offset of 
 * each of the sensor readings.
 */
boolean Calibrate() {
  // Number of readings to take during calibration
  int caliReadings = 5000;
  
  // Readings for the accelerometer, gyroscope, and magnetometer
  int16_t a[3] = {0,0,0}; 
  int16_t g[3] = {0,0,0};
  int16_t m[3] = {0,0,0};
  
  // History vectors for the readings
  long accHist[3] = {0,0,0};
  long gyroHist[3] = {0,0,0};
  //long magHist[3] = {0,0,0};
  
  // Find total readings for the number of readings specified
  for (int k = 0; k < caliReadings; k++) {
    //IMU.getMotion9(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2], &m[0], &m[1], &m[2]);
    IMU.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);  
    for (int x = 0; x < 3; x++) {
      accHist[x] += a[x];
      gyroHist[x] += g[x];
      // magHist[x] += m[x];
    }
    if (k % 1000 == 0) {
      //Serial.print(k/1000);Serial.print("..."); 
    }
  }
  //Serial.println("");
  // Take the average of the readings to determine sensor offsets
  for (int x = 0; x < 3; x++) {
    // NOTE: gyro and mag mag not need cali stuff added maybe
    accCali[x] += accHist[x] / caliReadings;
    gyroCali[x] += gyroHist[x] / caliReadings;
    //magCali[x] += magHist[x] / caliReadings;
  }
  //Serial.println("Done Calibrating.");
  return true;
}

/**
 * Reads the IMU raw data using the MPU6050 library. Uses the resolutions 
 * of +/- 2g for the accelerometer, +/- 250 deg/s for the gyroscope, and 
 * +/- 1200 microT for the magnetometer to convert into usable SI units.
 */
void ReadIMU() {
  // Readings for the accelerometer, gyroscope, and magnetometer
  int16_t a[3] = {0,0,0}; 
  int16_t g[3] = {0,0,0};
  int16_t m[3] = {0,0,0};
  
  // MPU6050 function to parse data
  //IMU.getMotion9(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2], &m[0], &m[1], &m[2]);  
  IMU.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]); 
  //IMU.getMag(&m[0], &m[1], &m[2]); 

  
  for (int x = 0; x < 3; x++) {
    accTotalReading[x] -= accBuffer[i][x];
    accBuffer[i][x] = a[x];
    accTotalReading[x] += a[x]; 
    
    gyroTotalReading[x] -= gyroBuffer[i][x];
    gyroBuffer[i][x] = g[x];
    gyroTotalReading[x] += g[x]; 
    
    magTotalReading[x] -= magBuffer[i][x];
    magBuffer[i][x] = m[x];
    magTotalReading[x] += m[x]; 
    
    // Convert each measurement into SI units  
    acc[x] = ((float)(accTotalReading[x]/numReadings) - accCali[x]) * 9.81 / 16384;    // m/s^2
    gyro[x] = ((float)(gyroTotalReading[x]/numReadings) - gyroCali[x]) * 250 / 32768;  // degrees/s
    mag[x] = ((float)(magTotalReading[x]/numReadings) - magCali[x]) * 1200 / 32768;    // microT
  }
  acc[2] = ((float)accTotalReading[2]/numReadings) * 9.81 / 16384;
}

/**
 * Implements a standard Kalman Filter to integrate acc and gyro data.
 */
void KalmanFilter(int dt) {
  // Prediction noise error
  float w = sq(1*dt*pi/180);
  // Correction noise error
  float v = sq(1*pi/180);
  
  // Roll`
  // Measurement for roll from the accelerometer
  //float roll = atan(acc[1] / sqrt(acc[0]*acc[0] + acc[2]*acc[2]))*180/pi;
  float roll = atan(acc[1] / sqrt(sq(acc[0]) + sq(acc[2])))*180/pi;
  // Run KF prediction and correction with noise reduction
  rollTotalReading -= gyroBufferKF[i][0];
  KFPrediction(0, gyro[0]*dt, w);
  KFCorrection(0, roll, v);
  rollTotalReading += gyroBufferKF[i][0];
  // Final roll estimate
  gyroKF[0] = rollTotalReading/numReadings;
  
  // Pitch
  // Measurement for pitch from accelerometer
  //float pitch = atan(-acc[0] / acc[2])*180/pi;
  float pitch = atan(acc[0] / sqrt(sq(acc[1]) + sq(acc[2])))*180/pi;
  // Run KF prediction and correction with noise reduction
  pitchTotalReading -= gyroBufferKF[i][1];
  KFPrediction(1, gyro[1]*dt, w);
  KFCorrection(1, pitch, v);
  pitchTotalReading += gyroBufferKF[i][1];
  // Final pitch estimate
  gyroKF[1] = pitchTotalReading/numReadings;
  
  // Yaw
  // Measurement for yaw from magnetometer
  float yaw = gyroAngle[2];
  // Run KF prediction and correction with noise reduction
  yawTotalReading -= gyroBufferKF[i][2];
  KFPrediction(2, gyro[2]*dt, w);  
  KFCorrection(2, yaw, v);
  yawTotalReading += gyroBufferKF[i][2];
  // Final yaw estimate
  gyroKF[2] = yawTotalReading/numReadings;
}

/**
 * Prediction method for the Kalman Filter
 * x_k|k0 = A*x_k0 + B*u_k
 * sigma_k|k0 = A*sigma_k0*A' + w
 */
void KFPrediction(int x, float uk, float w) {
  int A = 1;
  int B = 1;
  // Predicted state and covariance
  gyroBufferKF[i][x] = A*gyroBufferKF[i][x] + B*uk;
  sigmaKF[x] = A*sigmaKF[x]*A + w;
}

/**
 * Correction method for the Kalman Filter
 * K = sigma_k|k0*C/(C*sigma_k|k0*C' + v)
 * x_k|k = x_k|k0 + K*(z_k - c*x_k|k0)
 * sigma_k|k = (1 - K*C)*x_k|k
 */
void KFCorrection(int x, float zk, float v) {
  int C = 1;
  // Kalman gain calculation
  float K = sigmaKF[x]*C/(C*sigmaKF[x]*C + v);
  // Corrected state and covariance
  gyroBufferKF[i][x] = gyroBufferKF[i][x] + K*(zk - C*gyroBufferKF[i][x]);
  sigmaKF[x] = (1 - K*C)*gyroBufferKF[i][x];
}

/**
 * Prints off the calibrated and converted data for the IMU.
 */ 
void printIMU() {
  // Prints accelerometer data
  Serial.print("Acc:\t");
  Serial.print(acc[0]); Serial.print("\t");
  Serial.print(acc[1]); Serial.print("\t");
  Serial.print(acc[2]); Serial.print("\t");
  
  // Prints gyroscope angular velocity data
  Serial.print("|\tGyro:\t");
  Serial.print(gyro[0]); Serial.print("\t");
  Serial.print(gyro[1]); Serial.print("\t");
  Serial.print(gyro[2]); Serial.print("\t");
  
  // Prints gyroscope angular position data
  //Serial.print("|\tGyro Angle:\t");
  //Serial.print(gyroAngle[0]); Serial.print("\t");
  //Serial.print(gyroAngle[1]); Serial.print("\t");
  //Serial.print(gyroAngle[2]); Serial.print("\t");
  
  // Prints gyroscope angular position data
  Serial.print("|\tGyro Angle:\t");
  Serial.print(gyroKF[0]); Serial.print("\t");
  Serial.print(gyroKF[1]); Serial.print("\t");
  Serial.print(gyroKF[2]); Serial.print("\t");
  
  // Prints magnetometer data
  Serial.print("|\tMag:\t");
  Serial.print(mag[0]); Serial.print("\t");
  Serial.print(mag[1]); Serial.print("\t");
  Serial.print(mag[2]); Serial.print("\t");
  
  // Prints the time measurements were taken at
  Serial.println(timePrev);
}



// =======================================================================


/**
 * The main program loop that reads in the converted IMU data, integrates
 * the gyroscope data to find the orientation angles of the IMU. Then the 
 * data is run through a Kalman filter to cancel out the IMU drift.
 */
void loop()
{
  // Read the IMU sensor values and convert to SI units
  ReadIMU();
  
  // Finds the time since last loop (dt)
  unsigned long timeCurr = millis();
  int dt = timeCurr - timePrev; // ms
  timePrev = timeCurr;
  
  // Integration for the gyroscope data to find the gyroscope angle
  for (int x = 0; x < 3; x++) {
    gyroAngle[x] += gyro[x]*dt/1000; // degrees
  }
  
  // Kalman Filter on the IMU data
  KalmanFilter(dt);
  motorYaw.write(gyroKF[2]);
  motorPitch.write(gyroKF[1]+90);
  
  // Prints the calculated IMU data
  //printIMU();
  
  // Increase the index variable, wrap around the filter constant
  i++;
  if (i >= numReadings) { i = 0; }
  
  // Delay for stability
  delay(1); 
}
