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

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

#define SCALE 180

ros::NodeHandle  nh;

Servo servo;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
char base_link[] = "/servo";
char odom[] = "/odom";

void servo_cb( const sensor_msgs::Joy& cmd_msg){
  uint16_t value = (cmd_msg.axes[0]*SCALE)+90;
  servo.write(value); //set servo angle, should be from 0-180   
  float angle = servo.read();
  publish(angle);
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<sensor_msgs::Joy> sub("joy", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
}

void publish(float angle)
{
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = 0.0;
  t.transform.rotation.x = 1.0/(1+(angle*angle));
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = angle/(1+(angle*angle));
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
}

//seems to be required
void loop()
{
  nh.spinOnce();
}
