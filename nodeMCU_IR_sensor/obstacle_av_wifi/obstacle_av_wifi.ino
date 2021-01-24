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

// ThingSpeak and WiFi Settings
byte server[]  = { 184, 106, 153, 149 }; // IP Address for the ThingSpeak API
String writeAPIKey = "YHK17B88G86UQOJ4";   // Write API Key for a ThingSpeak Channel
//const int updateThingSpeakInterval = 30000;        // Time interval in milliseconds to update ThingSpeak (30000 = 30 seconds)
const int updateThingSpeakInterval = 0;  // Time interval in millisecondsto update ThingSpeak (30000 = 30 seconds)
const char* ssid = "STARK Industries";
const char* password = "language";
int status = WL_IDLE_STATUS;
WiFiClient client;

// Variables that are used in getDistance()
const int ir_pin = D_one;
const int led_pin = D_seven;
int isObstacle = HIGH;

// Additional variables usedin communicating with ThingSpeak
boolean lastConnected = false;
long lastConnectionTime = 0;



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
  pinMode(led_pin,OUTPUT);
  pinMode(ir_pin,INPUT);
}
 
void loop() {

  int obstacle = checkObstacle();
    
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

   //Read the reply from the server
   while (client.connected())
    if (client.available())
      client.read();
  
   client.stop();
   //delay(2000);
   ESP.deepSleep(2000);
  }

  if(obstacle == 1){
    // Update ThingSpeak Channel
    if(!client.connected() && (millis() - lastConnectionTime > updateThingSpeakInterval))
    {
      updateThingSpeak("field1="+String(obstacle));
    }

    lastConnected = client.connected();    
  }

 


  
}

int checkObstacle()
{
  isObstacle = digitalRead(ir_pin);
  if(isObstacle == LOW)
  {
    digitalWrite(led_pin, HIGH);
    return 1;
  }
  else
  {
    digitalWrite(led_pin, LOW);
    return 0;
  }
}

void updateThingSpeak(String tsData)
{
  if (client.connect(server, 80))
  { 
    Serial.println("Connected to ThingSpeak...");
    Serial.println(tsData);
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
    delay(500);
  }
  else
  {
    Serial.println("Connection Failed.");   
    Serial.println();

    lastConnectionTime = millis(); 
  }
}

 
