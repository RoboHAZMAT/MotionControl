int in;

void setup()
{
  Serial.begin(38400);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop()
{
  // send data only when you receive data:
  if (Serial.available() > 0) {

    digitalWrite(13,HIGH);
    // read the incoming byte:
    in = Serial.read();
    if (in == 1) {
      digitalWrite(12,HIGH);
    }
    else if (in == 0) {
      digitalWrite(12, LOW);
    }
  }
}




