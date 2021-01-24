
//=====================================================================================================================================//
//                     __________________   ____     ______________        __________________   _____         ______                   //
//                    /                 /  /    |   /              \      /                 /   \    \      _/    _/                   //
//                   /____      _______/  /_   /   /    _______     \    /    _____________/     \    \   _/    _/                     //
//                        /    /            | /   /    /      /     /   /    /_____               \    \_/    _/                       //
//                       /    /             |/   /    /______/     /   /          /              _/         _/                         //
//                      /    /                  /    ___     _____/   /    ______/             _/    _     /                           //
//                     /    /                  /    /   \    \       /    /____________      _/    _/ \    \                           //
//                    /    /                  /    /     \    \     /                 /    _/    _/    \    \                          //
//                   /____/                  /____/       \____\   /_________________/    /_____/       \____\                         //
//                                                                                                                                     //
//                 T'REX robot controller designed and programmed by Russell Cameron for DAGU Hi-Tech Electronics                      //
//=====================================================================================================================================//


#include <Wire.h>                                      // interrupt based I2C library
#include <Servo.h>                                     // library to drive up to 12 servos using timer1
#include <EEPROM.h>                                    // library to access EEPROM memory
#include "IOpins.h"                                    // defines which I/O pin is used for what function
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#include "TimerObject.h";

ros::NodeHandle nh;

// This is the callback function to use the killswitch...it doesn't work completely yet
void killswitch_cb( const std_msgs::Int16& cmd_msg){
  if(cmd_msg.data != 0){
    Shutdown();  
    while(1){}
  }
}




////////////////
// ROS Topics //
////////////////

// Topics to subscribe to
ros::Subscriber<std_msgs::Int16> left_motor_spd_sub("rubble/motor/left/spd", left_motor_spd_cb);
ros::Subscriber<std_msgs::Int16> right_motor_spd_sub("rubble/motor/right/spd", right_motor_spd_cb);
ros::Subscriber<std_msgs::Int16> killswitch_sub("rubble/killswitch", killswitch_cb);
//ros::Subscriber<std_msgs::Int16> gimbal_yaw_sub("rubble/gimbal/yaw", gimbal_yaw_cb);
//ros::Subscriber<std_msgs::Int16> gimbal_pitch_sub("rubble/gimbal/pitch", gimbal_pitch_cb);

// Topics to publish to
std_msgs::UInt16 batteryvolt;
std_msgs::UInt16 axis_x;
std_msgs::UInt16 axis_y;
std_msgs::UInt16 axis_z;
std_msgs::Int16 lmcur_data;
std_msgs::Int16 rmcur_data;

// Publishing objects
ros::Publisher battery_pub("rubble/battery/volt", &batteryvolt);
//ros::Publisher axis_x_pub("rubble/accelerometer/x", &axis_x);
//ros::Publisher axis_y_pub("rubble/accelerometer/y", &axis_y);
//ros::Publisher axis_z_pub("rubble/accelerometer/z", &axis_z);
ros::Publisher lmcur_pub("rubble/motor/left/current", &lmcur_data);
ros::Publisher rmcur_pub("rubble/motor/right/current", &rmcur_data);

void motor_timer_callback(){
  Motors();                             // Tell the motors to act on their new signal
}

/*
void gimbal_timer_callback(){
  Servos();                             // Tell the motors to act on their new signal
}
*/
/*
void axis_timer_callback(){
  
  axis_x.data = analogRead(axisxpin);
  axis_y.data = analogRead(axisypin);
  axis_z.data = analogRead(axiszpin);

  // Publish that accelerometer data
  axis_x_pub.publish( &axis_x );
  axis_y_pub.publish( &axis_y );
  axis_z_pub.publish( &axis_z );
  
}
*/
//const int numReadings = 50;
//int readIndex = 0;
//float total = 0;
//float average = 0;
  
//float readings[numReadings];

////////////////////
// TREX Constants //
////////////////////

// define constants here
#define startbyte 0x0F  // for serial communications each datapacket must start with this byte

// define global variables here
byte mode=0;                                           // mode=0: I2C / mode=1: Radio Control / mode=2: Bluetooth / mode=3: Shutdown
int  lowbat=550;                                       // default low battery voltage is 5.5V
byte errorflag;                                        // non zero if bad data packet received
byte pwmfreq;                                          // value from 1-7
byte i2cfreq;                                          // I2C clock frequency can be 100kHz(default) or 400kHz
byte I2Caddress;                                       // I2C slave address
int lmspeed,rmspeed;                                   // left and right motor speeds -255 to +255
byte lmbrake,rmbrake;                                  // left and right brakes - non zero values enable brake
int lmcur,rmcur;                                       // left and right motor current
int lmenc,rmenc;                                       // left and right encoder values
int volts;                                             // battery voltage*10 (accurate to 1 decimal place)
int xaxis,yaxis,zaxis;                                 // X, Y, Z accelerometer readings
int deltx,delty,deltz;                                 // X, Y, Z impact readings 
int magnitude;                                         // impact magnitude
byte devibrate=50;                                     // number of 2mS intervals to wait after an impact has occured before a new impact can be recognized
int sensitivity=50;                                    // minimum magnitude required to register as an impact

byte RCdeadband=35;                                    // RCsignal can vary this much from 1500uS without controller responding
unsigned long time;                                    // timer used to monitor accelerometer and encoders

byte servopin[6]={7,8,12,13,5,6};                      // array stores IO pin for each servo
int servopos[6];                                       // array stores position data for up to 6 servos
Servo servo[6];                                        // create 6 servo objects as an array

const int MOTOR_TS = 10;
const int GIMBAL_TS = 10;
const int AXIS_TS = 10;

TimerObject* motor_timer = 0;
//TimerObject* gimbal_timer = 0;
//TimerObject* axis_timer = 0;

boolean first_loop = true;

void setup()
{
  //========================================== Choose your desired motor PWM frequency ================================================//
  //                       Note that higher frequencies increase inductive reactance and reduce maximum torque                         //
  //                               Many smaller motors will not work efficiently at higher frequencies                                 //
  //                      The default is 122Hz. This provides relatively low noise and relatively smooth torque                        //
  //                                    This setting can be changed using I2C or Bluetooth                                             //
  //                                                                                                                                   //
  //     Thanks to macegr - http://forum.arduino.cc/index.php?PHPSESSID=n1691l4esq4up52krpcb77bgm1&topic=16612.msg121031#msg121031     //
  //===================================================================================================================================//

  //TCCR2B = TCCR2B & B11111000 | B00000001; pwmfreq=1;    // set timer 2 divisor to    1 for PWM frequency of  31250.000000000 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010; pwmfreq=2;    // set timer 2 divisor to    8 for PWM frequency of   3906.250000000 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011; pwmfreq=3;    // set timer 2 divisor to   32 for PWM frequency of    976.562500000 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100; pwmfreq=4;    // set timer 2 divisor to   64 for PWM frequency of    488.281250000 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000101; pwmfreq=5;    // set timer 2 divisor to  128 for PWM frequency of    244.140625000 Hz
    TCCR2B = TCCR2B & B11111000 | B00000110; pwmfreq=6;    // set timer 2 divisor to  256 for PWM frequency of    122.070312500 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111; pwmfreq=7;    // set timer 2 divisor to 1024 for PWM frequency of     30.517578125 Hz


  //all IO pins are input by default on powerup --------- configure motor control pins for output -------- pwm autoconfigures -----------

  pinMode(lmpwmpin,OUTPUT);                            // configure left  motor PWM       pin for output
  pinMode(lmdirpin,OUTPUT);                            // configure left  motor direction pin for output
  pinMode(lmbrkpin,OUTPUT);                            // configure left  motor brake     pin for output
  
  pinMode(rmpwmpin,OUTPUT);                            // configure right motor PWM       pin for output
  pinMode(rmdirpin,OUTPUT);                            // configure right motor direction pin for output
  pinMode(rmbrkpin,OUTPUT);                            // configure right motor brake     pin for output
  
  //----------------------------------------------------- Test for RC inputs ------------------------------------------------------------

  digitalWrite(RCspeedpin,1);                          // enable weak pullup resistor on input to prevent false triggering                   
  digitalWrite(RCsteerpin,1);                          // enable weak pullup resistor on input to prevent false triggering
  delay(100);

  ////////////////////////
  // Node Handler Setup //
  ////////////////////////

  // Initialize the node handler
  nh.initNode();

  // Subscribe to the proper topics
  nh.subscribe(left_motor_spd_sub);
  nh.subscribe(right_motor_spd_sub);
  nh.subscribe(killswitch_sub);
  //nh.subscribe(gimbal_yaw_sub);
  //nh.subscribe(gimbal_pitch_sub);

  // Be sure you can publish to the battery
  //nh.advertise(battery_pub);
  //nh.advertise(axis_x_pub);
  //nh.advertise(axis_y_pub);
  //nh.advertise(axis_z_pub);
  nh.advertise(lmcur_pub);
  nh.advertise(rmcur_pub);
  nh.advertise(battery_pub);

  //////////////////////////
  // Debug & Verification //
  //////////////////////////

  MotorBeep(3); // generate 3 beeps to prove we have made it through setup

  //for(int thisReading = 0; thisReading < numReadings; thisReading++){
  //  readings[thisReading] = 0;
  //}

  //Serial.begin(115200);

  
  motor_timer = new TimerObject(MOTOR_TS);
  //gimbal_timer = new TimerObject(GIMBAL_TS);
  //axis_timer = new TimerObject(AXIS_TS);
  /*
  motor_timer->setOnTimer(&motor_timer_callback);
  //motor_timer->Start();

  motor_timer->setOnTimer(&gimbal_timer_callback);
  //motor_timer->Start();

  motor_timer->setOnTimer(&axis_timer_callback);
  //motor_timer->Start();

  

  motor_timer_callback();
  gimbal_timer_callback();
  axis_timer_callback();

  */
}




void loop()
{

/*
  if(first_loop == true){
    motor_timer->Start();
    gimbal_timer->Start();
    axis_timer->Start();
    first_loop = false;
  }
*/
  //nh.spinOnce();   
  //total = total - readings[readIndex];
  //readings[readIndex] = float(analogRead(voltspin)*.1/3.357);
  //total = total + readings[readIndex];
  //readIndex = readIndex + 1;

  //if (readIndex >= numReadings) {
  //  readIndex = 0;
  //}






  
  //Serial.println(int(analogRead(voltspin)*10/3.357));
                       // This is what allows communication to pass serially between the pi and the arduino
  //average = total / float(numReadings);

  // Evaluate the state of the power systems on the rover
  batteryvolt.data = int(analogRead(voltspin)*.1/3.357);           // Grab the voltage data to return to base
  int lmcurpindata = (analogRead(lmcurpin)-511)*48.83;          // read  left motor current sensor and convert reading to mA;
  int rmcurpindata = (analogRead(rmcurpin)-511)*48.83;          // read  right motor current sensor and convert reading to mA;
  total_current = lmcurpindata + rmcurpindata;
  lmcur_data.data = lmcurpindata;
  rmcur_data.data = rmcurpindata;

  // If we are in dangerous territory power-wise, then shut down
  if((batteryvolt.data < lowbat) || (total_current > 15000)){
    while(1){
      Shutdown();
    }
  }

  //Serial.println(analogRead(axisxpin));
  //Serial.println(analogRead(axisypin));
  //Serial.println(analogRead(axiszpin));
  
  
  
  //axis_timer->Update();
  //motor_timer->Update();
  //gimbal_timer->Update();

  //axis_x.data = analogRead(axisxpin);
  //axis_y.data = analogRead(axisypin);
  //axis_z.data = analogRead(axiszpin);
  

  // Publish that voltage data
  //axis_x_pub.publish( &axis_x );
  //axis_y_pub.publish( &axis_y );
  //axis_z_pub.publish( &axis_z );

  // Publish the data to rostopics, run the motors and servos
  nh.spinOnce();
  lmcur_pub.publish( &lmcur_data );
  rmcur_pub.publish( &rmcur_data );
  battery_pub.publish( &batteryvolt.data );
  Motors();
  Servos();

  // This delay helps us to not fill up our queue to publish to a rostopic and create serious delays
  // in updating the ros topic.
  delay(10); 
}
