#include <SimpleDHT.h>

// This example uses an Adafruit Huzzah ESP8266
// to connect to shiftr.io.
//
// You can check on your device after a successful
// connection here: https://shiftr.io/try.
//
// by Joël Gähwiler
// https://github.com/256dpi/arduino-mqtt

#include <ESP8266WiFi.h>
#include <MQTT.h>


String id_base = "<insert_network_name>";
String station_number = "1";
String id = id_base + station_number;

String ip_string = "<insert_IP_of_rapsi_here>";
const char ssid[] = "<insert_ssid>";
const char pass[] = "<insert_router_password>";

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

WiFiClient net;
MQTTClient client;

unsigned long lastMillis = 0;

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  
  char id_array[15];
  id.toCharArray(id_array, 14);
  Serial.print("\nconnecting...");
  while (!client.connect(id_array, "try", "try")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("weather/temp");
  client.subscribe("weather/hum");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

int pinDHT22 = D_one;
SimpleDHT22 dht22;

void setup() {
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

void loop() {
  
  // read without samples.
  byte temperature = 0;
  byte humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht22.read(pinDHT22, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); 
    Serial.println(err);
    delay(1000);
    return;
  }

  Serial.print("Sample OK: ");
  Serial.print((int)temperature); Serial.print(" *C, "); 
  Serial.print((int)humidity); Serial.println(" H");

  if (!client.connected()) {
    connect();
  }

  String spacing = " ";
  String temp = String(int(temperature));
  String hum = String(int(humidity));

  String temp_concat = station_number + spacing + temp;
  String hum_concat = station_number + spacing + hum;

  //char temperature_array[10];
  //char humidity_array[10];
  //temp_concat.toCharArray(temperature_array, 10);
  //hum_concat.toCharArray(humidity_array, 10);

  client.publish("weather/temp", temp_concat);
  client.publish("weather/hum", hum_concat);
  client.disconnect();
  delay(2000);
  ESP.deepSleep(300e6); // 20e6 is 20 seconds
  //}
}
