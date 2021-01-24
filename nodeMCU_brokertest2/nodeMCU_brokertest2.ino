#include <ESP8266WiFi.h>
#include <MQTTClient.h>
 
 
const char* ssid     = "STARK Industries";
const char* password = "language";

channelID

topic = "channels/" + channelID + "/publish/" + writeAPIKey

WiFiClient WiFiclient;
MQTTClient client;
 
unsigned long lastMillis = 0;
 
void setup() {
 Serial.begin(115200);
 delay(10);
 Serial.println();
 Serial.println();
 Serial.print("Connecting to ");
 Serial.println(ssid);
 
 WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
   delay(500);
   Serial.print(".");
 }
 
 Serial.println("");
 Serial.println("WiFi connected");  
 Serial.println("IP address: ");
 Serial.println(WiFi.localIP());
 delay(2000);
 
 Serial.print("connecting to MQTT broker...");
 client.begin("192.168.8.101", WiFiclient);
 connect();
}
 
void connect() {
 while (!client.connect("test_channel", "try", "try")) {
   Serial.print(".");
 }
 
 Serial.println("\nconnected!");
 client.subscribe("test_channel");
}
 
void loop() {
 int val = analogRead(A0);
 client.loop();
 if(!client.connected()) {
   connect();
 }
 
 if(millis() - lastMillis > 1000) {
   lastMillis = millis();
   client.publish("test_channel", (String)val);
 }
}
