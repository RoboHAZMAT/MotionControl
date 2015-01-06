//========================== IMU Quaternion Read ===========================
/* RoboHAZMAT Senior Design Team
 * Motion Control Group
 * Gerardo Bledt
 * December 15, 2014
 *
 * Program written for recieving data from the MPU9150 IMU sensor with an 
 * Arduino microcontroller. Makes use of Jeff Rowberg's library for reading
 * the DMP. IMU raw data is recieved over the I2C bus and the controller 
 * outputs the quaternion values through serial communication. Checks for 
 * and makes sure to avoid FIFO buffer overflow. If the IMU readings become 
 * unstable, make the DMP read rate higher.
 * 
 * 1. Parameter Setup: Defines and initializes all of the necessary 
 *    variables for the IMU and DMP. Sets up Serial and I2C communication.
 *
 * 2. Main Loop: Reads incoming IMU raw data from I2C bus as a quaternion
 *    and outputs it through serial with a custon communication protocol.
 */
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//============================ Parameter Setup =============================
/* Defines and initializes the MPU9150 IMU sensor, the quaternion object, 
 * the reset button pins, and the MPU control/status variables and buffers.
 */
 
// MPU objeject with 0x68 default I2C address
MPU6050 mpu;

// Initialize Quaternion
Quaternion q;           // [w, x, y, z]

// Reset button pins
const int pinOut = 11;
const int pinIn = 13;

// MPU control / status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

/**
 * Connects to and initializes the MPU9150 IMU and the DMP. Begins Serial 
 * communication and joins the I2C bus. Checks for any errors with the IMU 
 * sensor and initializes the reset button.
 */
void setup() {
    // Begin I2C communication
    Wire.begin();
    
    // Initialize reset button
    pinMode(pinOut, OUTPUT);
    pinMode(pinIn, INPUT);
    digitalWrite(pinOut, HIGH);

    // Initialize serial communication
    Serial.begin(9600);

    // Initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // Load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
    }
}

//=============================== Main Loop ================================
/* Reads the incoming values from the IMU as a quaternion. Checks for any 
 * issues such as FIFO buffer overflow.
 */
void loop() {
  // Resets length
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // Check for overflow (this should never happen unless our code is too inefficient)
  if (fifoCount >= 1024) {
      // Reset so we can continue cleanly
      mpu.resetFIFO();
  } 
  
  // Wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // Read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
       
  // Track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;

  // Display quaternion values in easy matrix form: w x y z
  mpu.dmpGetQuaternion(&q, fifoBuffer);

  // Reset button status
  int reset = digitalRead(pinIn);
  
  // Creates the communication protocol
  Serial.print("$");
  Serial.print(q.w);Serial.print("#");
  Serial.print(q.x);Serial.print("%");
  Serial.print(q.y);Serial.print("&");
  Serial.print(q.z);Serial.print("@");
  Serial.print(reset);Serial.println("!");
}
