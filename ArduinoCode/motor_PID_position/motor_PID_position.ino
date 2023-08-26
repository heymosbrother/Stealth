// ROS serial libraries
#include <ros.h>
#include <std_msgs/Float32.h>

// pin settings
#define IN1 14
#define IN2 15
#define ENA 2
#define ENB 3
#define PWM 9
#define STB 11

// Encoder variables
long long int ticks = 0;
// PID variables
float error, error_prev, error_sum = 0.0;
float Kp = 0.76, Ki = 0.0, Kd = 0.0;
// Motor characteristics variables
float gearRatio = 100.0;
float encoderResolution = 12.0;
// ROS serial variables
float targetAngle = 0.0;

// ROS serial functions
void targetAngleCallback(const std_msgs::Float32& msg)
{
	targetAngle = msg.data;
}
// ROS node handle
ros::NodeHandle nh;
// Publish messgae
std_msgs::Float32 motorAngleMsg;
// Subscriber for the "targetAngle_topic" topic
ros::Subscriber<std_msgs::Float32> sub("targetAngle_topic", &targetAngleCallback);
// Publisher for the "motorAngle_topic" topic
ros::Publisher motor_angle_pub("motorAngle_topic", &motorAngleMsg);


void setup()
{
	// pin settings
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, INPUT);
    pinMode(ENB, INPUT);
    pinMode(PWM, OUTPUT);
    pinMode(STB, OUTPUT);
    digitalWrite(STB, HIGH);
	// For tuning PID parameters
    Serial.begin(9600);
    //interrupt setup
    attachInterrupt(digitalPinToInterrupt(ENA), readEncoder, RISING);

    // PID setup
    error_prev = 0.0;

	// ROS serial inits
	nh.initNode();		// Initialize the ROS node
	nh.subscribe(sub); 	// Subscribe to the "targetAngle_topic" topic
	nh.advertise(motor_angle_pub);
}

void loop()
{
    PID_position_control(targetAngle); // rotate 90 degrees
	// Update the current motor angle
	motorAngleMsg.data = (float)ticks / gearRatio / encoderResolution * 360.0;
	motor_angle_pub.publish(&motorAngleMsg);	// Publish the current motor angle
	nh.spinOnce();
}

// The target value is in the unit of angle
void PID_position_control(float target)
{
    // transform the target value to the unit of ticks
    error = target / 360.0 * gearRatio * encoderResolution - (float)ticks;
    error_sum += error;
    // the driving signal
    float u = Kp * error + Ki * error_sum + Kd * (error - error_prev);
    // determine the direction of the motor
    int direction = 1;
    if (u < 0)
    {
        direction = -1;
        u = -1 * u;
    }
    else
    {
        direction = 1;
    }
    // saturate the driving signal to the range of 0 to 255
    if (u > 255) u = 255;
    // drive the motor
    driveMotor((int)u, direction);
    // update the error for D controller
    error_prev = error;
    // print the current position in angle
    Serial.println((float)ticks / gearRatio / encoderResolution * 360.0);
}

void driveMotor(int pwmSignal, int direction)
{
    if (direction == 1)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(PWM, pwmSignal);
    }
    else if (direction == -1)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(PWM, pwmSignal);
    }
    else
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(PWM, 0);
    }
}

void readEncoder()
{
    // if the ENB pin is high, the motor is rotating in the positive direction
    if (digitalRead(ENB)) ticks++;
    else ticks--;
}
