/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

//#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;


// Topics to publish to
std_msgs::UInt16 leftmotorspd;
std_msgs::UInt16 leftmotordir;
std_msgs::UInt16 rightmotorspd;
std_msgs::UInt16 rightmotordir;
std_msgs::UInt16 gimbalyaw;
std_msgs::UInt16 gimbalpitch;

ros::Publisher left_motor_spd_rsp("rubble/response/leftmotorspd", &leftmotorspd);
ros::Publisher left_motor_dir_rsp("rubble/response/leftmotordir", &leftmotordir);
ros::Publisher right_motor_spd_rsp("rubble/response/rightmotorspd", &rightmotorspd);
ros::Publisher right_motor_dir_rsp("rubble/response/rightmotordir", &rightmotordir);
ros::Publisher gimbal_yaw_rsp("rubble/response/gimbalyaw", &gimbalyaw);
ros::Publisher gimbal_pitch_rsp("rubble/response/gimbalpitch", &gimbalpitch);


void left_motor_spd_cb( const std_msgs::UInt16& cmd_msg){
  left_motor_spd_rsp.publish( &cmd_msg );
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
}

void left_motor_dir_cb( const std_msgs::UInt16& cmd_msg){
  left_motor_dir_rsp.publish( &cmd_msg );
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
}

void right_motor_spd_cb( const std_msgs::UInt16& cmd_msg){
  right_motor_spd_rsp.publish( &cmd_msg );
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
}

void right_motor_dir_cb( const std_msgs::UInt16& cmd_msg){
  right_motor_dir_rsp.publish( &cmd_msg );
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
}

void gimbal_yaw_cb( const std_msgs::UInt16& cmd_msg){
  gimbal_yaw_rsp.publish( &cmd_msg );
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
}

void gimbal_pitch_cb( const std_msgs::UInt16& cmd_msg){
  gimbal_pitch_rsp.publish( &cmd_msg );
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
}

// Topics to subscribe to
ros::Subscriber<std_msgs::UInt16> left_motor_spd_sub("rubble/motor/left/spd", left_motor_spd_cb);
ros::Subscriber<std_msgs::UInt16> left_motor_dir_sub("rubble/motor/left/dir", left_motor_dir_cb);
ros::Subscriber<std_msgs::UInt16> right_motor_spd_sub("rubble/motor/right/spd", right_motor_spd_cb);
ros::Subscriber<std_msgs::UInt16> right_motor_dir_sub("rubble/motor/right/dir", right_motor_dir_cb);
ros::Subscriber<std_msgs::UInt16> gimbal_yaw_sub("rubble/gimbal/yaw", gimbal_yaw_cb);
ros::Subscriber<std_msgs::UInt16> gimbal_pitch_sub("rubble/gimbal/pitch", gimbal_pitch_cb);

void setup(){

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  nh.initNode();
  nh.subscribe(left_motor_spd_sub);
  nh.subscribe(left_motor_dir_sub);
  nh.subscribe(right_motor_spd_sub);
  nh.subscribe(right_motor_dir_sub);
  nh.subscribe(gimbal_yaw_sub);
  nh.subscribe(gimbal_pitch_sub);

  nh.advertise(left_motor_spd_rsp);
  nh.advertise(left_motor_dir_rsp);
  nh.advertise(right_motor_spd_rsp);
  nh.advertise(right_motor_dir_rsp);
  nh.advertise(gimbal_yaw_rsp);
  nh.advertise(gimbal_pitch_rsp);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
