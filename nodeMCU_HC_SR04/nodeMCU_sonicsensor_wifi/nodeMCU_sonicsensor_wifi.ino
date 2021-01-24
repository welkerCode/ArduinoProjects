#include <ESP8266WiFi.h>
#include "ThingSpeak.h"


// Conversion from NodeMCU Amica pins to Arduino GPIO
int D_twelve = 10;
int D_eleven = 9;
int D_ten = 1;
int D_nine = 3;
int D_eight = 15;
int D_seven = 13;
int D_six = 12;
int D_five = 14;
int D_four = 2;
int D_three = 0;
int D_two = 4;
int D_one = 5;
int D_zero = 16;

// These pins are designated to interact with the HC-SR04 sensor
const int trigPin = D_four;
const int echoPin = D_three;

// ThingSpeak Settings
byte server[]  = { 184, 106, 153, 149 }; // IP Address for the ThingSpeak API
String writeAPIKey = "YHK17B88G86UQOJ4";   // Write API Key for a ThingSpeak Channel
const int updateThingSpeakInterval = 30000;        // Time interval in milliseconds to update ThingSpeak (30000 = 30 seconds)


// defines variables
float duration;
float distance;

const char* ssid = "STARK Industries";
const char* password = "language";
int status = WL_IDLE_STATUS;
WiFiClient client;

boolean lastConnected = false;
long lastConnectionTime = 0;
int resetCounter = 0;
 
void setup() {
  // Set up the networking stuff
  Serial.begin(115200);
  delay(10);
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
  ThingSpeak.begin(client);


  // Then, setup the sensor stuff
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}
 
void loop() {

  float distance = getDistance();

  // Print Update Response to Serial Monitor
  if (client.available())
  {
    char c = client.read();
    Serial.print(c);
  }
  
  // Disconnect from ThingSpeak
  if (!client.connected() && lastConnected)
  {
    Serial.println();
    Serial.println("...disconnected.");
    Serial.println();
    
    client.stop();
  }

  // Update ThingSpeak Channel
  if(!client.connected() && (millis() - lastConnectionTime > updateThingSpeakInterval))
  {
    updateThingSpeak("field1="+String(distance));
  }

  lastConnected = client.connected();
}

float getDistance()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = (duration/2) / 29.1 / 100.0;

  // Prints the distance on the Serial Monitor
  return distance;
}

void updateThingSpeak(String tsData)
{
  if (client.connect(server, 80))
  { 
    Serial.println("Connected to ThingSpeak...");
    Serial.println();
        
    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: "+writeAPIKey+"\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(tsData.length());
    client.print("\n\n");

    client.print(tsData);
    
    lastConnectionTime = millis();
    
    resetCounter = 0;
    
  }
  else
  {
    Serial.println("Connection Failed.");   
    Serial.println();
    
    resetCounter++;
    
    //if (resetCounter >=5 ) {resetEthernetShield();}

    lastConnectionTime = millis(); 
  }
}

 
