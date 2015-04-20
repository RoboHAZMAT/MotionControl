// ########## Wireless Sensor Packet ##########
/*
John Gardiner & Gerardo Bledt for the 2014-2015 Virginia Tech Mechanical
Engineering Senior Design RoboHazMat Project.
 */

// #### Libraries ####
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "ctype.h"

// Set up nRF24L01 radio on SPI bus plus pins 9 & 53 (normally 9 & 10 on regualr Arduinos)
RF24 radio(9,10);
//RF24 radio(53,48);

//comm pipes
//const uint64_t talking_pipes[5] = { 0xABCDABCD71LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL };
//const uint64_t listening_pipes[5] = { 0xABCDABCD71LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL };

const uint64_t talking_pipes[5] = { 0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL };
const uint64_t listening_pipes[5] = { 0x3A3A3A3AD2LL, 0x3A3A3A3AC3LL, 0x3A3A3A3AB4LL, 0x3A3A3A3AA5LL, 0x3A3A3A3A96LL };

// Wireless Packet Structure - quaternion and flex sensor
// Since the biceps do not have flex sensors, the will send a 
// specified null value
typedef struct{
  int N; //Packet ID
  float W;// quaternion begin
  float X;
  float Y;
  float Z;// quaternion end
  float F;// Flex sensor
  int reset; //reset button depressed
  int T; //Time
}
A_t;
A_t wirelesspacket,P2,P3,P4,P5,P6; //Instantiate packets. 

//array holding the bools of system and wsp's indicating whether or not they
//are ready to be read.
bool idready [6] = {false, false, false, false, false, false};
//idready -> {whole system, id2, id3, id4, id5, id6} description of vars

//the array indicating what systems need to be initialized in order for readings
//to start being sent to matlab. Set the id's that you are using as true. All 
//should be true when using the full system.
bool readyconfig [6] = {true, false, false, false, false, false};

void setup(void)
{
  // Print preamble
  Serial.begin(115200);
  //Serial.println("This is the wireless receiver.");
  printf_begin();

  // Setup and configure rf radio
  radio.begin();
  radio.setDataRate(RF24_2MBPS); // Both endpoints must have this set the same
  radio.setAutoAck(false);       // Either endpoint can set to false to disable ACKs

  // Open pipes to other nodes for communication 
  // The pong node listens on all the ping node talking pipes
  radio.openReadingPipe(1,talking_pipes[0]);
  radio.openReadingPipe(2,talking_pipes[1]);
  radio.openReadingPipe(3,talking_pipes[2]);
  radio.openReadingPipe(4,talking_pipes[3]);
  radio.openReadingPipe(5,talking_pipes[4]);

  //Start listening
  radio.startListening();
  
  //Dump the configuration of the rf unit for debugging
  radio.printDetails();
  
  //debug initiate
  wirelesspacket.N = 0;
  wirelesspacket.W = 1;
  wirelesspacket.X = 2;
  wirelesspacket.Y = 3;
  wirelesspacket.Z = 4;
  wirelesspacket.F = 5;
  wirelesspacket.reset = 0;
  wirelesspacket.T = 60;
  
}

void loop(void)
{

  // if there is data ready
  uint8_t pipe_num; //pipe number variable
  if ( radio.available(&pipe_num) )
  {
    Serial.print("I've got something ");
    Serial.println(pipe_num);
    // Dump the payloads until we've gotten everything
    bool done = false;
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
        done = radio.read( &wirelesspacket, sizeof(wirelesspacket) );
      }
    
    Serial.print("*");Serial.print(wirelesspacket.N);
    Serial.print("^");Serial.print(wirelesspacket.F);
    Serial.print("$");
    Serial.print(wirelesspacket.W);Serial.print("#");
    Serial.print(wirelesspacket.X);Serial.print("%");
    Serial.print(wirelesspacket.Y);Serial.print("&");
    Serial.print(wirelesspacket.Z);Serial.print("@");
    Serial.print(wirelesspacket.reset);Serial.println("!");
    
    // Serial.print("Got a packet");Serial.println(millis());
    /*
    switch (wirelesspacket.N) {
    case 0:
      Serial.print("What the hell, this packet shouldn't have been sent");
      break;
    case 1:
      Serial.print("ID 1 is supposed to be the receiver. Something's wrong.");
      break;
    case 2:
      P2.N = wirelesspacket.N;
      P2.W = wirelesspacket.W;
      P2.X = wirelesspacket.X;
      P2.Y = wirelesspacket.Y;
      P2.Z = wirelesspacket.Z;
      P2.F = wirelesspacket.F;
      P2.reset = wirelesspacket.reset;
      P2.T = wirelesspacket.T;
      //SendPacketInfo(2);
      //Serial.println(idready[0]); Serial.println(idready[1]);
      if (!idready[0] && !idready[1]){UpdateSystemStatus(2);}
      break;
    case 3:
      P3.N = wirelesspacket.N;
      P3.W = wirelesspacket.W;
      P3.X = wirelesspacket.X;
      P3.Y = wirelesspacket.Y;
      P3.Z = wirelesspacket.Z;
      P3.F = wirelesspacket.F;
      P3.reset = wirelesspacket.reset;
      P3.T = wirelesspacket.T;
      SendPacketInfo(3);
      if (!idready[0] && !idready[2]){UpdateSystemStatus(3);}
      break;
    case 4:
      P4.N = wirelesspacket.N;
      P4.W = wirelesspacket.W;
      P4.X = wirelesspacket.X;
      P4.Y = wirelesspacket.Y;
      P4.Z = wirelesspacket.Z;
      P4.F = wirelesspacket.F;
      P4.reset = wirelesspacket.reset;
      P4.T = wirelesspacket.T;
      //SendPacketInfo(4);
      if (!idready[0] && !idready[3]){UpdateSystemStatus(4);}
      break;
    case 5:
      P5.N = wirelesspacket.N;
      P5.W = wirelesspacket.W;
      P5.X = wirelesspacket.X;
      P5.Y = wirelesspacket.Y;
      P5.Z = wirelesspacket.Z;
      P5.F = wirelesspacket.F;
      P5.reset = wirelesspacket.reset;
      P5.T = wirelesspacket.T;
      //SendPacketInfo(5);
      if (!idready[0] && !idready[4]){UpdateSystemStatus(5);}
      break;
    case 6:
      P6.N = wirelesspacket.N;
      P6.W = wirelesspacket.W;
      P6.X = wirelesspacket.X;
      P6.Y = wirelesspacket.Y;
      P6.Z = wirelesspacket.Z;
      P6.F = wirelesspacket.F;
      P6.reset = wirelesspacket.reset;
      P6.T = wirelesspacket.T;
      //SendPacketInfo(6);
      if (!idready[0] && !idready[5]){UpdateSystemStatus(6);}
      break;
    }
     */ 
  }
  else
  {
    //Serial.println("Nothing Available");
    
  }
  if (Serial.available()>0)
  {
    char theid = Serial.read();
    String stheid(theid);
    int id = 0;
    id = stheid.toInt();
    if (idready[0]){
    //Serial.println(id);
    //Serial.println("Sending things your way");
    SendPacketInfo(id);
    }
    else {Serial.println("System not ready!");}
    delay(10);
  }
  
}

void SendPacketInfo(int packetid)
{
  switch (packetid)
  {
    case 1:
    Serial.print("*");Serial.print(wirelesspacket.N);
    Serial.print("^");Serial.print(wirelesspacket.F);
    Serial.print("$");
    Serial.print(wirelesspacket.W);Serial.print("#");
    Serial.print(wirelesspacket.X);Serial.print("%");
    Serial.print(wirelesspacket.Y);Serial.print("&");
    Serial.print(wirelesspacket.Z);Serial.print("@");
    Serial.print(wirelesspacket.reset);Serial.println("!");
    break;
    
    case 2:
    //Serial.println("Here comes packet two");
    Serial.print("*");Serial.print(P2.N);
    Serial.print("^");Serial.print(P2.F);
    Serial.print("$");
    Serial.print(P2.W);Serial.print("#");
    Serial.print(P2.X);Serial.print("%");
    Serial.print(P2.Y);Serial.print("&");
    Serial.print(P2.Z);Serial.print("@");
    //Serial.print(P2.reset);Serial.println("!");
    Serial.print(P2.reset);Serial.print("!");Serial.println(P2.T);
    break;
    
    case 3:
    Serial.print("*");Serial.print(P3.N);
    Serial.print("^");Serial.print(P3.F);
    Serial.print("$");
    Serial.print(P3.W);Serial.print("#");
    Serial.print(P3.X);Serial.print("%");
    Serial.print(P3.Y);Serial.print("&");
    Serial.print(P3.Z);Serial.print("@");
    Serial.print(P3.reset);Serial.println("!");
    break;
    
    case 4:
    Serial.print("*");Serial.print(P4.N);
    Serial.print("^");Serial.print(P4.F);
    Serial.print("$");
    Serial.print(P4.W);Serial.print("#");
    Serial.print(P4.X);Serial.print("%");
    Serial.print(P4.Y);Serial.print("&");
    Serial.print(P4.Z);Serial.print("@");
    Serial.print(P4.reset);Serial.println("!");
    break;
    
    case 5:
    Serial.print("*");Serial.print(P5.N);
    Serial.print("^");Serial.print(P5.F);
    Serial.print("$");
    Serial.print(P5.W);Serial.print("#");
    Serial.print(P5.X);Serial.print("%");
    Serial.print(P5.Y);Serial.print("&");
    Serial.print(P5.Z);Serial.print("@");
    Serial.print(P5.reset);Serial.println("!");
    break;
    
    case 6:
    Serial.print("*");Serial.print(P6.N);
    Serial.print("^");Serial.print(P6.F);
    Serial.print("$");
    Serial.print(P6.W);Serial.print("#");
    Serial.print(P6.X);Serial.print("%");
    Serial.print(P6.Y);Serial.print("&");
    Serial.print(P6.Z);Serial.print("@");
    Serial.print(P6.reset);Serial.println("!");
    break;
  }
}

void UpdateSystemStatus(int idcheck)
{
  //updates the idready array. If all imu's are ready, the system is set to ready
  //and a confirmation is sent to the computer.
  switch (idcheck){
    case 2:
    //Serial.println("We in");
    if (P2.W == 1.00 && P2.X == 2.00 && P2.Y == 3.00 && P2.Z == 4.00 && P2.F == 5.00)
    {idready[1] = true;} //Serial.println("Set to True");}
    break;
    
    case 3:
    if (P3.W == 1.00 && P3.X == 2.00 && P3.Y == 3.00 && P3.Z == 4.00 && P3.F == 5.00)
    {idready[2] = true;}
    break;
    
    case 4:
    if (P4.W == 1 && P4.X == 2 && P4.Y == 3 && P4.Z == 4 && P4.F == 5)
    {idready[3] = true;}
    break;
    
    case 5:
    if (P5.W == 1 && P5.X == 2 && P5.Y == 3 && P5.Z == 4 && P5.F == 5)
    {idready[4] = true;}
    break;
    
    case 6:
    if (P6.W == 1 && P6.X == 2 && P6.Y == 3 && P6.Z == 4 && P6.F == 5)
    {idready[5] = true;}
    break;
  }
  
  //Checks if the IMU's are all set up according to the setup conditions
  //If everything is correct, the systemready variable is set to true.
  if (idready[1] == readyconfig[1] && idready[2] == readyconfig[2] &&
      idready[3] == readyconfig[3] && idready[4] == readyconfig[4] &&
      idready[5] == readyconfig[5]) 
      {
        idready[0] = true;
        Serial.println("System is ready");
      }
  
  
}
