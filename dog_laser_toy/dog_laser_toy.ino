#include "Servo.h"

Servo servo_vertical;  // create servo object to control a servo
Servo servo_horizontal;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

int door_hor = 125;
int door_ver = 50;

int median_hor = 135;
int median_ver = 50;

int bedroom_ver = 55;
int bedroom_hor = 50;

int floor_ver = 120;
int floor_hor = 120;

int couch_ver = 80;
int couch_hor = 180;

int kennel_ver = 53;
int kennel_hor = 148;

int wait_time = 4000;

int iterator = 0;
int num_pos = 6;

int hor_pos [6] = {door_hor, median_hor, kennel_hor, bedroom_hor, couch_hor, floor_hor};
int ver_pos [6] = {door_ver, median_ver, kennel_ver, bedroom_ver, couch_ver, floor_ver};



void setup() {
  servo_vertical.attach(8);  // attaches the servo on pin 9 to the servo object
  servo_horizontal.attach(9);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
}

void loop() {

  iterator++;
  if(iterator > 5){
    iterator = 0;
  }
  servo_vertical.write(ver_pos[iterator]);
  servo_horizontal.write(hor_pos[iterator]);
  delay(wait_time);

  /*
  for (pos = 30; pos <= 130; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo_vertical.write(pos);              // tell servo to go to position in variable 'pos'
    servo_horizontal.write(pos);
    delay(200);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 130; pos >= 30; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo_vertical.write(pos);              // tell servo to go to position in variable 'pos'
    servo_horizontal.write(pos);
    delay(200);                       // waits 15ms for the servo to reach the position
  }
  */
  
}
