#include "Servo.h"

Servo servo_vertical;  // create servo object to control a servo
Servo servo_horizontal;
// twelve servo objects can be created on most boards

// Pin layout
int buzzer_pin = 6;
int led_pin = 7;
int servo_ver_pin = 8;
int servo_hor_pin = 9;

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


char incoming_value = 0;

void buzz_on(){
    digitalWrite(buzzer_pin, HIGH);
    delay(500);
    digitalWrite(buzzer_pin, LOW);
}

void buzz_off(){
    digitalWrite(buzzer_pin, HIGH);
    delay(100);
    digitalWrite(buzzer_pin, LOW);
    delay(100);
    digitalWrite(buzzer_pin, HIGH);
    delay(100);
    digitalWrite(buzzer_pin, LOW);
}



void setup() {
  servo_vertical.attach(servo_ver_pin);  // attaches the servo on pin 9 to the servo object
  servo_horizontal.attach(servo_hor_pin);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(buzzer_pin, HIGH);
  digitalWrite(led_pin, LOW);
  Serial.begin(9600);
}

void loop() {

  if(Serial.available() > 0){
    incoming_value = Serial.read();
    Serial.print(incoming_value);
    Serial.print("\n");

    if (incoming_value == '1'){
      buzz_on();
    }
    else{
      buzz_off();
    }
  }

  if(incoming_value == '1'){

    digitalWrite(led_pin, HIGH);

    iterator++;
    if(iterator > (num_pos - 1)){
      iterator = 0;
    }
    
    servo_vertical.write(ver_pos[iterator]);
    servo_horizontal.write(hor_pos[iterator]);
    delay(wait_time);
  }

  else{
    digitalWrite(led_pin, LOW);
    digitalWrite(buzzer_pin, LOW);
  }

  // Get waypoints
  // Increment current_counter
  // if current_counter is equal to num_points
    // reset to 0
    
  // Increment next_counter
  // If current_counter is equal to num_points
    //reset to 0
  
  // Get current_pos
  // int current_pos_ver = ver_pos[current_counter];
  // int current_pos_hor = hor_pos[current_counter];
  // int next_pos_ver = ver_pos[next_counter];
  // int next_pos_hor = hor_pos[next_counter];
  
  // Until current_pos_hor is equal to the next_waypoint_hor and
  // current_pos_ver is equal to the next_waypoint_ver
    // if current_pos_hor is not equal to next_hor
      // increment current_pos_hor
      // Determine direction
      // if positive
        // ++
      // else
        // --
    // if current_pos_ver is not equal to next_ver
      // increment current_pos_ver

    // Update the servo

    

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
