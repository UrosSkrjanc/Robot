#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include <string>
#include "roborodney/ukazVoznje.h"
#include <iostream>
#include <stdio.h>



std::string smer="stop";
std::string ukazJoy="";
ros::Publisher ukaz_smeri;
roborodney::ukazVoznje ukaz;

//funkcija za izpis trenutnih nastavitev
void izpisPodatkov() {
	ROS_INFO("Hitrost: %d  Trajanje: %d ", ukaz.hitrost, ukaz.trajanje);
}


void joypadmess(const sensor_msgs::Joy& msg)
{
  //ROS_INFO("AU!!!");
  //ROS_INFO ("%d %d %d %d",msg.buttons[13],msg.buttons[14],msg.buttons[15],msg.buttons[16]);
  if(msg.buttons[15]==1) {ROS_INFO("Obracanje levo"); smer="levo";}
  if(msg.buttons[16]==1) {ROS_INFO("Obracanje desno"); smer="desno";}
  if(msg.buttons[13]==1) {ROS_INFO("Voznja naprej"); smer="naprej";}
  if(msg.buttons[14]==1) {ROS_INFO("Voznja nazaj"); smer="nazaj";}
  if(msg.buttons[2]==1) {/*ROS_INFO("Povecanje hitrosti");*/ ukazJoy="povecanjeHitrosti";}
  if(msg.buttons[0]==1) {/*ROS_INFO("Zmanjsanje hitrosti");*/ ukazJoy="zmanjsanjeHitrosti";}
  if(msg.buttons[3]==1) {/*ROS_INFO("INFO");*/ ukazJoy="informacije";izpisPodatkov();}
  if(msg.buttons[6]==1) {/*ROS_INFO("Povecanje hitrosti");*/ ukazJoy="pladenjLevo";}
  if(msg.buttons[7]==1) {/*ROS_INFO("Povecanje hitrosti");*/ ukazJoy="pladenjDesno";}
  if(msg.buttons[1]==1) {/*ROS_INFO("INFO");*/ ukazJoy="konec";}
  if(msg.buttons[13]==0 && msg.buttons[14]==0 && msg.buttons[15]==0 && msg.buttons[16]==0 && msg.buttons[2]==0 && msg.buttons[0]==0 && msg.buttons[1]==0 && msg.buttons[6]==0 && msg.buttons[7]==0) {/*ROS_INFO("STOP!");*/ smer="stop";ukazJoy="";}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joypadnav");
	ros::NodeHandle nh;
	
	ros::Rate sekundniPostanek01(1);

	
	ros::Subscriber joypad = nh.subscribe("joy", 1, joypadmess);
	ukaz_smeri = nh.advertise<roborodney::ukazVoznje>("ukaz_smeri",1000);
	
	
	
	ukaz.smer=smer;
	ukaz.hitrost=50;
	ukaz.trajanje=100;

	sekundniPostanek01.sleep();
	ros::spinOnce();
	
	ros::Rate loop_rate(10);

	
	ROS_INFO("Delujem!");
	
	while (nh.ok()) {
		//ROS_INFO("%s",smer.c_str());
		ukaz.smer=smer;
		ukaz_smeri.publish(ukaz);
		ros::spinOnce();
		
		//se preberem ukaz za zmanjsevanje/vecanje hitrosti
		if(ukazJoy=="povecanjeHitrosti"){
			ukaz.hitrost=ukaz.hitrost+1;
			if(ukaz.hitrost>100){ukaz.hitrost=100;}
			ROS_INFO("Povecal hitrost na %d",ukaz.hitrost);
		}
		if(ukazJoy=="zmanjsanjeHitrosti"){
			ukaz.hitrost=ukaz.hitrost-1;
			if(ukaz.hitrost<0){ukaz.hitrost=0;}
			ROS_INFO("Zmanjsal hitrost na %d",ukaz.hitrost);
		}
		
		/*
		if(ukazJoy=="pladenjLevo"){
			ROS_INFO("Premikam pladen proti levi.");
		}
		if(ukazJoy=="pladenjDesno"){
			ROS_INFO("Premikam pladen proti desni.");
		}
		*/
		
		if(ukazJoy=="konec"){
			ROS_INFO("Izhod iz programa.");
			return 0;
		}
		
		loop_rate.sleep();
		
	}
}
