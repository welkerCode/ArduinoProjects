#include <ESP8266WiFi.h>
#include "ThingSpeak.h"
#include <SimpleDHT.h>

// for DHT11, 
//      VCC: 5V or 3V
//      GND: GND
//      DATA: 2

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

// ThingSpeak Settings
byte server[]  = { 184, 106, 153, 149 }; // IP Address for the ThingSpeak API
String writeAPIKey = "G88AGC4R4Y7NC3P8"; // Write API Key for a ThingSpeak Channel
const int updateThingSpeakInterval = 1000;  // Time interval in milliseconds to update ThingSpeak (30000 = 30 seconds)

const char* ssid = "STARK Industries";
const char* password = "language";
int status = WL_IDLE_STATUS;
WiFiClient client;

boolean lastConnected = false;
long lastConnectionTime = 0;
int resetCounter = 0;

int pinDHT11 = D_one;
SimpleDHT22 dht22;

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
}

void loop() {  
  
  // read without samples.
  byte temperature = 0;
  byte humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht22.read(pinDHT11, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); 
    Serial.println(err);
    //delay(1000);
  }

  Serial.print("Sample OK: ");
  Serial.print((int)temperature); Serial.print(" *C, "); 
  Serial.print((int)humidity); Serial.println(" H");
  
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
    ESP.deepSleep(600e6); // 20e6 is 20 microseconds
  }

  // Update ThingSpeak Channel
  if(!client.connected() && (millis() - lastConnectionTime > updateThingSpeakInterval))
  {
    updateThingSpeak("field1="+String((int)temperature)+"&field2="+String((int)humidity));
    
  }

  lastConnected = client.connected();
  
  
  Serial.print("Time diff: ");
  Serial.println(millis() - lastConnectionTime);
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
    
  }
  else
  {
    Serial.println("Connection Failed.");   
    Serial.println();

    lastConnectionTime = millis(); 
  }
}
