// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]

const int pinOut = 11;
const int pinIn = 13;

void setup() {
    // Begin I2C communication
    Wire.begin();
    pinMode(pinOut, OUTPUT);
    pinMode(pinIn, INPUT);
    digitalWrite(pinOut, HIGH);

    // Initialize serial communication
    Serial.begin(9600);

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
    }
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // resets length
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if (fifoCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
  } 
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
       
  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;

  // Display quaternion values in easy matrix form: w x y z
  mpu.dmpGetQuaternion(&q, fifoBuffer);
            
  // Conversion from quaternions back to yaw, pitch, roll
  if (q.w >= 2) q.w = -4 + q.w;
  if (q.x >= 2) q.x = -4 + q.x;
  if (q.y >= 2) q.y = -4 + q.y;
  if (q.z >= 2) q.z = -4 + q.z;

  int reset = digitalRead(pinIn);
  
  // Creates the communication protocol
  Serial.print("$");
  Serial.print(q.w);Serial.print("#");
  Serial.print(q.x);Serial.print("%");
  Serial.print(q.y);Serial.print("&");
  Serial.print(q.z);Serial.print("@");
  Serial.print(reset);Serial.println("!");
}
