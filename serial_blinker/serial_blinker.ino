int ledPin = 13;

char dataString[50] = {0};
int a = 0;

String incomingByte;
void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  
  if (Serial.available() > 0) {
    incomingByte = Serial.readString();

    Serial.print("I received: ");
    Serial.println(incomingByte);

    incomingByte = incomingByte.substring(0,2);
  
    if(incomingByte == "on"){
      digitalWrite(ledPin, HIGH);
      Serial.println("Light on...");
    }
    else if(incomingByte == "of"){
      digitalWrite(ledPin, LOW);
      Serial.println("Light off...");
    }
    else{
      Serial.print("Got ");
      Serial.println(incomingByte);
    }
    //delay(5000);
    Serial.readString();
  }
  Serial.println("Not getting any message");
} 
