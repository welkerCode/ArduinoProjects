

// Global flag variables
bool on_guard = false; // Determines if our 'shift' has ended
bool detected = false; // Determines if we detected movement

// Global timer variables
float setup_timer = 0.1; // (minutes) Time before the device activates, time to 'set it up'
float guard_timer = 1.0; // (minutes) Length of guard shift, how long it is watching after setup timer


// Define pin numbers
int trigPin = 2;
int echoPin = 3;
int ledPin = 7;
int buttonPin = 9;

// Global variables for the sensor
long duration; //variable for the duration of sound wave travel
int distance; //variable for the distance measurement
unsigned long delayStart = 0; // the time the delay started
int guard_timer_ms;
float guard_timer_min;

int max_safe_height = 40;

void toggle_led(){
  Serial.print("LED: ");
  Serial.println(digitalRead(ledPin));
  //int led_state = digitalRead(ledPin);
  if(digitalRead(ledPin) == HIGH){
    digitalWrite(ledPin, LOW);
  }
  else{
    digitalWrite(ledPin, HIGH);
  }
  //digitalWrite(ledPin, !digitalRead(ledPin));
}

float check_sensor(){
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void setup() {

  // Set up your LED Pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  // Set up your button Pin
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);
  
  // Set up your sensor Pins
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin,INPUT);
  
  // Set up the setup_timer
  delay(setup_timer * 1000);

  // Set up the guard_timer
  float guard_timer_sec = guard_timer * 1000;
  guard_timer_min = guard_timer_sec * 60;
  

  // Set up the global flag variables
  on_guard = true;
  detected = false;

  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor HC-SR04 Test");
  Serial.println("With Ardunio UNO");
}

void loop(){

  float time_elapsed = (millis() - delayStart);
  Serial.print("Guard timer min: ");
  Serial.println(guard_timer_min);
  Serial.print("Time elapsed: ");
  Serial.println(time_elapsed);
  
  Serial.println(time_elapsed);
  Serial.print("On guard: ");
  Serial.println(on_guard);
  Serial.print("Detected: ");
  Serial.println(detected);

  // check if delay has timed out after guard_timer
  if (time_elapsed >= guard_timer_min){
    on_guard = false;
  }

  // If we are on duty
  if(on_guard == true){
    // Check the sensor
    distance = check_sensor();
    if(distance > max_safe_height){
      detected = true;
    }
  }

  // If we are off duty
  else {
    // Exiting guard time, only poll for button
    if(digitalRead(buttonPin) == LOW){
      if(detected == true){
        toggle_led();
        delay(1000);
      }
      else{
        digitalWrite(ledPin, HIGH);
        delay(1000);
      }
    } 
    else{
      digitalWrite(ledPin, LOW);
    }
   
  }
  

}
