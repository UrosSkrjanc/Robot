#include "ros/ros.h"
#include "roborodney/pozicijaServoMotorja.h"
#include <stdio.h>
#include <termios.h>
#include "std_msgs/String.h"
#include <string>
#include "roborodney/pozicijaServoMotorja.h"


int levo=1300;
int sredina=1500;
int desno=1700;
int trenutnaPozicija=0;
int korak=10;
int deluje=0;

roborodney::pozicijaServoMotorja sporocilo;
roborodney::pozicijaServoMotorja sporociloPozicijaMotorja;
ros::Publisher posiljanje;
ros::Publisher pozicijaMotorja;


void senzorlevodesno(const std_msgs::String::ConstPtr& msg)
{
	std::string ukaz= msg->data.c_str();
	if (ukaz.compare("start")==0)
	{
		ROS_INFO("Startam levo desno");
		deluje=1;
	}
	if (ukaz.compare("stop")==0)
	{
		ROS_INFO("Ustavljam levo desno");
		deluje=0;
		sporocilo.pozicija=1500;
		sporocilo.cas=ros::Time::now();
		posiljanje.publish(sporocilo);
	}
}


int main(int argc, char **argv){

	ros::init(argc, argv, "SenzorLevoDesno");
	ros::NodeHandle nh;
	posiljanje = nh.advertise<roborodney::pozicijaServoMotorja>("servomotorPozicija",1000);
	pozicijaMotorja = nh.advertise<roborodney::pozicijaServoMotorja>("pozicijaMotorja",1000);
	ros::Rate loop_rate(1);
	
	ros::Subscriber sub = nh.subscribe("senzorlevodesno", 1000, senzorlevodesno);
	
	ros::Rate desetinka(10);


	sporocilo.pin=25;
	sporocilo.cas=ros::Time::now();
	sporocilo.pozicija=1000;
	

	int c=0;
	
	while(nh.ok()) {
		if(deluje==1){
		for(int i=levo; i<desno; i=i+30) {
			if(deluje==0){break;}
			sporocilo.pozicija=i;
			sporocilo.cas=ros::Time::now();
			posiljanje.publish(sporocilo);
			sporociloPozicijaMotorja.pozicija=i;
			sporociloPozicijaMotorja.cas=ros::Time::now();
			pozicijaMotorja.publish(sporociloPozicijaMotorja);
			ros::spinOnce();
			desetinka.sleep();
		}
		for(int i=desno; i>levo; i=i-30) {
			if(deluje==0){break;}
			sporocilo.pozicija=i;
			sporocilo.cas=ros::Time::now();
			posiljanje.publish(sporocilo);
			sporociloPozicijaMotorja.pozicija=i;
			sporociloPozicijaMotorja.cas=ros::Time::now();
			pozicijaMotorja.publish(sporociloPozicijaMotorja);
			ros::spinOnce();
			desetinka.sleep();
		}
		}
		ros::spinOnce();
		desetinka.sleep();
	}
}
