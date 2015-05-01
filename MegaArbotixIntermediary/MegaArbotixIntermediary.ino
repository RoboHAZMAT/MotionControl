//Mega Arbotix Intermediary
//This code was written to work around the Arbotix/USB2Dynamixel
//Confliction when connecting to the same computer.
//John Gardiner
//4/27/2015

//Variables
String rd = "";
int LED = 13;
int checker = 0;
String theread = "";

//Setup
void setup()
{
  Serial.begin(115200);
  Serial2.begin(57142);
  String starter = "";
}

void loop()
{
  // nothing
}

//Transfers the serial from the matlab to the arbotix
void serialEvent()
{
  Serial.println("Serial Event triggered");
  while (Serial.available() > 0)
  {
    rd = Serial.readStringUntil('\n');
  }
  Serial.print("Got: ");Serial.println(rd);
  
  checker = Serial2.println(rd);
  Serial.println("Sent Instructions");
  Serial.print("Wrote "); Serial.print(checker);Serial.println(" bytes");  digitalWrite(LED,HIGH);
  rd = "";
  
}

void serialEvent2()
{
  String theread;
  while (Serial2.available() > 0)
  {
    theread = Serial2.readStringUntil('\n');    
  }
  
  Serial.print("The arbotix got this: ");
  Serial.println(theread);
  theread = "";
}
