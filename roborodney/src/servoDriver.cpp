#include <ros/ros.h>
#include <pigpio.h>
#include "roborodney/pozicijaServoMotorja.h"

void premikNaPozicijo(const roborodney::pozicijaServoMotorja::ConstPtr& msg)
{
	ROS_INFO("%d, %d",msg->pin, msg->pozicija);
	gpioServo(msg->pin, msg->pozicija);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servoDriver");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("servomotorPozicija", 1000, premikNaPozicijo);

	if(gpioInitialise()<0) {ROS_INFO("Motor ne deluje"); return 1;}else ROS_INFO("Motor pripravljen.");

	ros::spin();
}
