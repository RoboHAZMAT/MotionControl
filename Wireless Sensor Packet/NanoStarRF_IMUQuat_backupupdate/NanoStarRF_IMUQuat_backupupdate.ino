  // ########## Wireless Sensor Packet ##########
/*
John Gardiner & Gerardo Bledt for the 2014-2015 Virginia Tech Mechanical
Engineering Senior Design RoboHazMat Project.
 */

// #### Libraries ####
//From Starping
#include <SPI.h>
#include <EEPROM.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//from IMU Quat
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);

// Instantiate arrays with comm pipes
//const uint64_t talking_pipes[5] = { 0xABCDABCD71LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL };
//const uint64_t listening_pipes[5] = { 0xABCDABCD71LL, 0x3A3A3A3AC3LL, 0x3A3A3A3AB4LL, 0x3A3A3A3AA5LL, 0x3A3A3A3A96LL };

const uint64_t talking_pipes[5] = { 0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL };
const uint64_t listening_pipes[5] = { 0x3A3A3A3AD2LL, 0x3A3A3A3AC3LL, 0x3A3A3A3AB4LL, 0x3A3A3A3AA5LL, 0x3A3A3A3A96LL };

// Where in EEPROM is the address stored?
const uint8_t address_at_eeprom_location = 0;

// What is our address (SRAM cache of the address from EEPROM)
// Note that zero is an INVALID address.  The pong back unit takes address
// 1, and the rest are 2-6
uint8_t node_address; //Set the WSP ID Here

// #### Structures ####
// In the future, verification characters need to be added to ensure
// that, even with packet loss, the right numbers are being sent to 
// the right channels.

// Wireless Packet Structure - quaternion and flex sensor
// Since the biceps do not have flex sensors, the will send a 
// specified null value

typedef struct{
  int N; //Sender ID
  float W;// quaternion begin
  float X;
  float Y;
  float Z;// quaternion end
  int F;// Flex sensor
  int reset;
  int T; //Time sent
}
A_t;

//variable declarations
A_t wirelesspacket;

//LEDs
int redled = 6;
int greenled = 7;
int blueled = 8;

//int reset = 0;

//IMU Quat stuff
//============================ Parameter Setup =============================
/* Defines and initializes the MPU9150 IMU sensor, the quaternion object, 
 * the reset button pins, and the MPU control/status variables and buffers.
 */
 
// MPU objeject with 0x68 default I2C address
MPU6050 mpu;

// Initialize Quaternion
Quaternion q;           // [w, x, y, z]

// Reset button pins
const int pinOut = 4; //JRG - Had to change to 1 & 2, 11&13 used by SPI
const int pinIn = 5;

// MPU control / status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Flex Sensor Stuff
// Filtering constant 
const int numReadings = 10;
int sensorPin = A0;
int sensorValue = 0;
int maxR;
int minR;
int readingBuffer[numReadings];
int motorPos;
int i = 0;

void setup(void)
{
  // LEDs
  pinMode(6, OUTPUT); //Red
  pinMode(7, OUTPUT); //Green
  pinMode(8, OUTPUT); //Blue

  // Wireless Setup //
  // Read the address from EEPROM
    uint8_t reading = EEPROM.read(address_at_eeprom_location);

    // If it is in a valid range for node addresses, it is the address.
    if ( reading >= 2 && reading <= 6 )
      node_address = reading;

    // Otherwise, it is invalid, so set our address to 0. Readings cannot be sent.
    else
    {
      node_address = 0;
    }

  // Print information
  Serial.begin(115200);
  Serial.print("This is a Wireless Sensor Packet. It's address is: ");
  Serial.println(node_address);

  // Setup and configure rf radio
  radio.begin();
  radio.setDataRate(RF24_2MBPS); // Both endpoints must have this set the same
  radio.setAutoAck(false);       // Either endpoint can set to false to disable ACKs

  // Write on our talking pipe
  radio.openWritingPipe(talking_pipes[node_address-2]);
  // Listen on our listening pipe
  radio.openReadingPipe(1,listening_pipes[node_address-2]);
  
  // Show the details of the radio
  printf_begin();
  radio.printDetails();
  
  //Initializing values - will be sent as confirmation
  wirelesspacket.N = node_address;
  wirelesspacket.W = 1;
  wirelesspacket.X = 2;
  wirelesspacket.Y = 3;
  wirelesspacket.Z = 4;
  wirelesspacket.F = 5;
  
  // ##### IMU Quat #####
  // Begin I2C communication
  Wire.begin();
  
  // Initialize reset button
  //pinMode(pinOut, OUTPUT);
  pinMode(pinIn, INPUT);
  digitalWrite(pinOut, HIGH);

  // Initialize IMU
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  digitalWrite(redled, HIGH);

  // Load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  digitalWrite(redled, LOW);
  delay(10);
  //digitalWrite(8, HIGH);
  analogWrite(blueled,220);
  
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
  // Send confirmation of IMU initialization
  Serial.println("Sending setup confirmation...");
  radio.write( &wirelesspacket, sizeof(wirelesspacket) );
  Serial.println("Sent");
  //digitalWrite(8, LOW);
  analogWrite(blueled,150);
  delay(10);
  //digitalWrite(7, HIGH);
  analogWrite(greenled,150);
  
  //Set Up Flex Sensor Stuff
  maxR = 450;
  minR = 200;
  
  for (int j = 0; j < numReadings; j++) 
  {
    readingBuffer[j] = 0;
  }
  
}

void loop(void)
{
  //Flex Sensors
  int reading;
  reading = constrain(map(analogRead(sensorPin),minR,maxR,175,0),0,175);
  //Serial.println(analogRead(sensorPin)); 
  motorPos = MoveMotor(i,reading,readingBuffer,motorPos);
  //\Serial.println(motorPos);
  
  // IMU Communication // 
  // Resets length
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  //Serial.print("FIFO Count Over | ");Serial.println(millis());
  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // Check for overflow (this should never happen unless our code is too inefficient)
  if (fifoCount >= 1024) {
      // Reset so we can continue cleanly
      mpu.resetFIFO();
      //Serial.print("FIFO Reset | ");Serial.println(millis());
  } 
  
  //Serial.print("Start Wait | ");Serial.println(millis());
  // Wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  //Serial.print("Finish Wait | ");Serial.println(millis());

  // Read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  //Serial.print("Packet Read | ");Serial.println(millis());
       
  // Track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;

  // Display quaternion values in easy matrix form: w x y z
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  //Serial.print("Got Quaternion | ");Serial.println(millis());

  // Reset button status
  int reset = digitalRead(pinIn);
  
  if (reset == HIGH) {     
  // turn LED on:    
  analogWrite(blueled, 150);
  analogWrite(greenled, 0);  
  } 
  else 
  {
   // turn LED off:
   analogWrite(blueled, 0);
   analogWrite(greenled, 150); 
  }
  
  /* Serial debug
  // Creates the communication protocol
  Serial.print("$");
  Serial.print(q.w);Serial.print("#");
  Serial.print(q.x);Serial.print("%");
  Serial.print(q.y);Serial.print("&");
  Serial.print(q.z);Serial.print("@");
  Serial.print(reset);Serial.println("!");
  */
   
  if (node_address != 0){
    // Wireless Comm //
    //Packaging the data into the data structure
    wirelesspacket.N = node_address;
    wirelesspacket.W = q.w;
    wirelesspacket.X = q.x;
    wirelesspacket.Y = q.y;
    wirelesspacket.Z = q.z;
    wirelesspacket.F = motorPos; //Will be changed to analog readn
    wirelesspacket.reset = reset;
    wirelesspacket.T = 0;
    
    //Send the data packet
    //Serial.print("Attempting to Send Packet | "); Serial.println(millis());
    radio.write( &wirelesspacket, sizeof(wirelesspacket) );
    //Serial.print("Packet Sent | ");Serial.println(millis());
  }
  //Setting the address
  if (Serial.available())
  {
    // If the character on serial input is in a valid range...
    char c = Serial.read();
    if ( c >= '1' && c <= '6' )
    {
      // It is our address
      EEPROM.write(address_at_eeprom_location,c-'0');

      // And we are done right now (no easy way to soft reset)
      printf("\n\rManually reset address to: %c\n\rPress RESET to continue!",c);
    }
  }
  
  // Increase the index variable, wrap around the filter constant
  i++;
  if (i >= numReadings) { i = 0; }
  
  // Delay for stability
  delay(1);
  
}

/**
 * Filters the readings for noise by averaging the previous readings for 
 * smooth sensor data.
 * Moves the corresponding sevor motor to the averaged position.
 */
int MoveMotor(int i,int reading,int readingBuffer[],int prevMotorPos) {
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
    //motor.write(motorPos);
    return motorPos;
  }
  return prevMotorPos;
}
