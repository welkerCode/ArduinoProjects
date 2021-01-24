#include <ThingSpeak.h>

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

const int trigPin = D_four;
const int echoPin = D_three;
const int ledPin = D_seven;

// defines variables
float duration;
float distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  digitalWrite(ledPin, !digitalRead(ledPin));
  
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
  
  Serial.print("Distance: ");
  Serial.println(distance,2);
  delay(250);
}
