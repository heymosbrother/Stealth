/*
 * rosserial motor PID control test 
 * publish and subscribe on one node
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include "Motor.h"

ros::NodeHandle nh; // Declare the nodeHandler
ros::Publisher motorSpeedReport("motorSpeedReport", &currentSpeed_msg);
ros::Subscriber<std_msgs::Float32> sub("motorSpeedAssign", &callBack);

Motor testMotor(10, 9, 8, 2, 11); // PWM, IN1, IN2, encoderA, encoderB

float motorSpeed = 0;

// When message is received, execute this
void callBack(const std_msgs::Float32& assignedSpeed_msg)
{
	float motorSpeed = assignedSpeed_msg.data;
	// do somthing, use motor class later
}


// Define a float message (current mototr velocity) to publish
std_msgs::Float32 currentSpeed_msg;

void setup()
{
	nh.initNode(); // Initialize the ROS node
	nh.advertise(motorSpeedReport); // Advertise the publisher
	nh.subscribe(motorSpeedAssign); // Subscribe to the subscriber

	currentSpeed_msg.data = 0.0;

	// Interrupt function for encoder control
	attachInterupt(digitalPinToInterrupt(testMotor.ENC_A), readEncoder, RISING);
}

void loop()
{
	testMotor.SetVelocity(motorSpeed);

	currentSpeed_msg.data = testMotor.ShowVelocity();
	pub.publish(&currentSpeed_msg);	
	nh.spinOnce();
}

void readEncoder()
{
	testMotor.SetEncoderPosition(digitalRead(testMotor.ENC_B));
}
