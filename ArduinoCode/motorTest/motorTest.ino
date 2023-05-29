/*
 * rosserial motor PID control test 
 * publish and subscribe on one node
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include "Motor.h"

void callBack(const std_msgs::Float32& assignedSpeed_msg);

ros::NodeHandle nh; // Declare the nodeHandler
// Define a float message (current mototr velocity) to publish
std_msgs::Float32 currentSpeed_msg;
ros::Publisher GetMotorVelocity("GetMotorVelocity_topic", &currentSpeed_msg);
ros::Subscriber<std_msgs::Float32> SetMotorVelocity("SetMotorVelocity_topic", &callBack);

Motor testMotor(10, 9, 8, 2, 11); // PWM, IN1, IN2, encoderA, encoderB

float motorSpeed = 0;

// When message is received, execute this
void callBack(const std_msgs::Float32& assignedSpeed_msg)
{
	float motorSpeed = assignedSpeed_msg.data;
	// do somthing, use motor class later
}

void setup()
{
	nh.initNode(); // Initialize the ROS node
	nh.advertise(GetMotorVelocity); // Advertise the publisher
	nh.subscribe(SetMotorVelocity); // Subscribe to the subscriber

	currentSpeed_msg.data = 0.0;

	// Interrupt function for encoder control
	attachInterrupt(digitalPinToInterrupt(testMotor.ENC_A), readEncoder, RISING);
}

void loop()
{
	testMotor.SetVelocity(motorSpeed);

	currentSpeed_msg.data = testMotor.ShowVelocity();
	GetMotorVelocity.publish(&currentSpeed_msg);	
	nh.spinOnce();
}

void readEncoder()
{
	testMotor.SetEncoderPosition(digitalRead(testMotor.ENC_B));
}
