// Function that subscribes to a rostopic that opens the tells the servo when to open the arm and when to close it 
/*
#include <Servo.h>

Servo myservo;

void setup() 
{ 
  myservo.attach(6);  // Set the PWM to PIN 6.
  
}

void loop() 
{
  myservo.write(90);
  delay(1000);
  myservo.write(135);   //claws open
  delay(1000);
}
*/

#include <Servo.h>
#include <ros.h>  //library needed to use rosserial
#include <std_msgs/Int16.h> //Int16 standard message format

//object for the servo
Servo myservo;

int state = 1;

ros::NodeHandle nh; //create ROS node handle

//Callback for the servo subscriber
void servo_callback(const std_msgs::Int16& serv_msg)
{
  state = serv_msg.data;
}

ros::Subscriber<std_msgs::Int16> s1("servo", &servo_callback);  // topic = servo;

void setup()
{
  Serial.begin(115200);
  nh.initNode();
  nh.subscribe(s1);
  myservo.attach(6);  // Set the PWM to Pin 6.
}

void loop()
{
  nh.spinOnce();
  if (state == 0) //Natural state
  {
      // Arms open.
      myservo.write(135);
  }
  else 
  {
      // Arms close
      myservo.write(90);
  }
}
