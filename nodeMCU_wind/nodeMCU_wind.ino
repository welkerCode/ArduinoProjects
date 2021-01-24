#include <LaCrosse_TX23.h>
#include <ESP8266WiFi.h>
#include <MQTT.h>

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

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

// Define the network the nodeMCU will be communicating through
const char ssid[] = "<insert_ssid>";
const char pass[] = "<insert_router_password>";

String ip_string = "<insert_IP_of_raspi_here>";

// Create the client objects
WiFiClient net;
MQTTClient client;

// This is used to evaluate times between publishes
unsigned long lastMillis = 0;

// This function connects the board to the chosen network and broker channel
void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("breccia_windstation", "try", "try")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("weather/wind");
  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

//DATA wire connected to arduino port 10
LaCrosse_TX23 anemometer = LaCrosse_TX23(D_one);

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

// Only the wifi and broker needs setup, not the wind sensor
void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid, pass);


  char ip[16];
  ip_string.toCharArray(ip, 16);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported by Arduino.
  // You need to set the IP address directly.
  client.begin(ip, net);
  client.onMessage(messageReceived);

  connect();
}

void loop()
{
  // First, read the wind sensor
  String dirTable[]= {"N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW"};
	float degTable[]= {0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5};
	float speed;
	int direction;
  
	if(anemometer.read(speed, direction))
  {
    Serial.println("Speed = " + String(speed,1) + " m/s");
    Serial.println("Dir = " + dirTable[direction]);    
  }
  else
  {
    Serial.println("Read error");
  }

  // Then, publish the data to the broker channel
  if (!client.connected()) {
    connect();
  }

  String station_number = "wind";

  String spacing = " ";
  String speed_string = String(speed);
  String direction_string = String(degTable[direction]);

  String speed_concat = station_number + spacing + speed_string;
  String direc_concat = station_number + spacing + direction_string;

  Serial.println("Speed message" + speed_concat);
  Serial.println("Direc message" + direc_concat);
  

  // publish a message roughly every 30 seconds.
  client.publish("weather/wind/spd", speed_concat);
  client.publish("weather/wind/dir", direc_concat);
  client.disconnect();
  delay(2000);
  ESP.deepSleep(300e6); // 20e6 is 20 seconds
}

