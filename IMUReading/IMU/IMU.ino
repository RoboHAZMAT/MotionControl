// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <MPU6050.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 IMU;

// sets up time history variable
int timePrev = 0;
  
// Readings for the accelerometer, gyroscope, and magnetometer
int16_t acc[3] = {0,0,0};
int16_t gyro[3] = {0,0,0};
int16_t mag[3] = {0,0,0};

// Direct integration for gyroscopic angle
int16_t gyroAngle[3] = {0,0,0};

// Readings for the accelerometer, gyroscope, and magnetometer
int16_t accCali[3] = {0,0,0};
int16_t gyroCali[3] = {0,0,0};
int16_t magCali[3] = {0,0,0};

void setup()
{
  // Joins the I2C bus
  Wire.begin();
  
  // Initialize serial communication
  Serial.begin(38400);
  
  // Initialize the IMU
  IMU.initialize();
  
  // Test connection to IMU
  Serial.println(IMU.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  boolean calibrated = false;
  
  while (!calibrated) {
    calibrated = Calibrate();
  }
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
  
  // History vectors for the readings
  int16_t accHist[3] = {0,0,0};
  int16_t gyroHist[3] = {0,0,0};
  int16_t magHist[3] = {0,0,0};
  
  // Find total readings for the number of readings specified
  for (int i = 0; i < caliReadings; i++) {
    IMU.getMotion9(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2], &mag[0], &mag[1], &mag[2]);
    for (int x = 0; x < 3; x++) {
      accHist[x] += acc[x];
      gyroHist[x] += gyro[x];
      magHist[x] += mag[x];
    }
  }
  
  // take the average of each of the readings to determine sensor offsets
  for (int x = 0; x < 3; x++) {
    // gyro and mag mag not need cali stuff added
    accCali[x] += accHist[x] / caliReadings;
    gyroCali[x] += gyroHist[x] / caliReadings;
    magCali[x] += magHist[x] / caliReadings;
  }
}

/**
 * Reads the IMU raw data using the MPU6050 library. Uses the resolutions 
 * of +/-2g for the accelerometer, +/- 250 deg/s for the gyroscope, and 
 * ? for the magnetometer to convert into usable SI units.
 */
void ReadIMU() {
  // MPU6050 function to parse data
  IMU.getMotion9(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2], &mag[0], &mag[1], &mag[2]);
  
  // Convert each measurement into SI units
  for (int x = 0; x < 3; x++) {
    acc[x] = (acc[x] - accCali[x]) * 9.81 / 16384;
    gyro[x] = (gyro[x] - gyroCali[x]) * 250 / 32768;
    mag[x] = (mag[x] - magCali[x]);
  }
}

void KalmanFilter() {
  // ***** TO DO *****
}


void printIMU() {
  Serial.print("Acc:\t");
  Serial.print(acc[0]); Serial.print("\t");
  Serial.print(acc[1]); Serial.print("\t");
  Serial.print(acc[2]); Serial.print("\t");
  Serial.print("|\tGyro:\t");
  Serial.print(gyro[0]); Serial.print("\t");
  Serial.print(gyro[1]); Serial.print("\t");
  Serial.print(gyro[2]); Serial.print("\t");
  Serial.print("|\tMag:\t");
  Serial.print(mag[0]); Serial.print("\t");
  Serial.print(mag[1]); Serial.print("\t");
  Serial.print(mag[2]); Serial.print("\t");
  Serial.print("|\tGyro Angle:\t");
  Serial.print(gyroAngle[0]); Serial.print("\t");
  Serial.print(gyroAngle[1]); Serial.print("\t");
  Serial.println(gyroAngle[2]);
}
// =======================================================================
void loop()
{
  // Read the IMU sensor values and convert to SI units
  ReadIMU();
  int timeCurr = millis();
  int dt = timeCurr - timePrev;
  timePrev = timeCurr;
  for (int x = 0; x < 3; x++) {
    gyroAngle[x] += gyro[x]*dt/1000;
  }
  
  printIMU();
  // Kalman Filter on the IMU data
  // KalmanFilter();
}
