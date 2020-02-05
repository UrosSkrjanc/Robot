#include <ros/ros.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <std_msgs/String.h>
#include "roborodney/ukazVoznje.h"
#include "roborodney/razdaljaSenzor.h"
#include "roborodney/pozicijaServoMotorja.h"


using namespace std;

ros::Publisher ukaz_smeri;

//kot za obrat
int kotRotacije=30;
string smer="stop";

float min=570;
float max=2430;
float pozicija=1500;
float sredina=1500;
int pinSignala=25;

int varnostnaRazdalja=50;

//za posiljanje ukaza voznje - naprej, nazaj, levo desno
roborodney::ukazVoznje ukaz;

//funkcija, ki s topica razdalja_senzor (ultrasonic in laser) dobiva vrednosti - oddaljenost robota od ovire 
void dobiRazdaljoSenzorja(const roborodney::razdaljaSenzor& msg)
{
	std_msgs::String msgg;
	//ROS_INFO("Razdalja %f",msg.razdalja);
	
	if(msg.razdalja<varnostnaRazdalja)
	{		
		ROS_INFO("Robot je blizu ovire %f",msg.razdalja);
	}
}


//funkciji za branje iz tipkovnice, kjer držim tipko dolgo časa
void set_mode(int want_key)
{
	static struct termios old;
    static struct termios nov;
	if (!want_key) {
		tcsetattr(STDIN_FILENO, TCSANOW, &old);
		return;
	}
 
	tcgetattr(STDIN_FILENO, &old);
	nov = old;
	nov.c_lflag &= ~(ICANON);
	tcsetattr(STDIN_FILENO, TCSANOW, &nov);
}
 
int get_key()
{
	int c = 0;
	fd_set fs;
 
	FD_ZERO(&fs);
	FD_SET(STDIN_FILENO, &fs);
	select(STDIN_FILENO + 1, &fs, 0, 0, 0);
 
	if (FD_ISSET(STDIN_FILENO, &fs)) {
		c = getchar();
		set_mode(0);
	}
	return c;
}

//funkcija za izpis trenutnih nastavitev
void izpisPodatkov() {
	ROS_INFO("Hitrost: %d  Trajanje: %d ", ukaz.hitrost, ukaz.trajanje);
}


int main(int argc, char **argv)
{
	
	//int stevec=0;
	
	ros::init(argc, argv, "voznja_Robota_tipkovnica");
	ros::NodeHandle nh;
	
	ros::Rate sekundniPostanek01(1);
	ros::Rate desetinkasekundePostanek01(10);
	ros::Rate stotinkasekundePostanek01(100);
	ros::Rate dvajsetinkasekundePostanek01(20);
	
	ukaz_smeri = nh.advertise<roborodney::ukazVoznje>("ukaz_smeri",10);
	ros::Subscriber razdaljaSenzor = nh.subscribe("razdalja_senzor", 1, dobiRazdaljoSenzorja);
	ros::Publisher posiljanje = nh.advertise<roborodney::pozicijaServoMotorja>("servomotorPozicija",1000);
	
	roborodney::pozicijaServoMotorja sporocilo;
	sporocilo.pin=pinSignala;
	
	ros::Rate postanek(1);

	
	std_msgs::String msg;
	msg.data="start";
	sekundniPostanek01.sleep();
	ros::spinOnce();
	
	ukaz.smer=smer;
	ukaz.hitrost=50;
	ukaz.trajanje=100;

	sekundniPostanek01.sleep();
	ros::spinOnce();
	
 
	ROS_INFO("Delujem!");

	
	  int c=0;
  while(c !=27)
  {

			
	set_mode(1);
    tcflush(STDIN_FILENO, TCIFLUSH);
    fflush(stdout);
	
	c = tolower(get_key());
	

	
	
	
	//brezpogojno naprej
	if (c=='t')
	{
		//stevec++;
		ROS_INFO("Voznja naprej");
		ukaz.smer="naprej";
		//ukaz.hitrost=25;
		//ukaz.trajanje=50;
		ukaz_smeri.publish(ukaz);
		//zaklenjen=0;
	} 
	
	//brezpogojno levo
	if (c=='f')
	{
		//stevec++;
		ROS_INFO("Obracanje levo");
		ukaz.smer="levo";
		//ukaz.hitrost=50;
		//ukaz.trajanje=30;
		ukaz_smeri.publish(ukaz);
	}
	
	//brezpogojno desno
	if (c=='h')
	{
		//stevec++;
		ROS_INFO("Obracanje desno");
		ukaz.smer="desno";
		//ukaz.hitrost=50;
		//ukaz.trajanje=30;
		ukaz_smeri.publish(ukaz);
	}
	
	//brezpogojno nazaj
	if (c=='b')
	{
		//stevec++;
		ROS_INFO("Voznja nazaj");
		ukaz.smer="nazaj";
		//ukaz.hitrost=25;
		//ukaz.trajanje=50;
		ukaz_smeri.publish(ukaz);
	}
	


	//izpis podatkov o hitrosti
	if(c=='y') {izpisPodatkov();}
	
	if(c=='a') {
		ukaz.hitrost=ukaz.hitrost+1;
		if(ukaz.hitrost>100){ukaz.hitrost=100;}
		ROS_INFO("Povecam hitrost na %d",ukaz.hitrost);
	}
	if(c=='s') {
		ukaz.hitrost=ukaz.hitrost-1;
		if(ukaz.hitrost<0){ukaz.hitrost=0;}
		ROS_INFO("Zmanjsam hitrost na %d",ukaz.hitrost);
	}
	
	/*
	if(c=='q') {
		ROS_INFO("Premikam pladen proti levi.");
		sporocilo.cas=ros::Time::now();
		pozicija=pozicija-20;
		sporocilo.pozicija=pozicija;
		posiljanje.publish(sporocilo);
		ros::spinOnce();
		postanek.sleep();
	}
	if(c=='w') {
		ROS_INFO("Premikam pladen proti desni.");
		sporocilo.cas=ros::Time::now();
		pozicija=pozicija+20;
		sporocilo.pozicija=pozicija;
		posiljanje.publish(sporocilo);
		ros::spinOnce();
		postanek.sleep();
	}
	if(c=='e') {
		ROS_INFO("Premikam pladen na sredino.");
		sporocilo.cas=ros::Time::now();
		pozicija=sredina;
		sporocilo.pozicija=sredina;
		posiljanje.publish(sporocilo);
		ros::spinOnce();
		postanek.sleep();
	}
	*/

	
	ros::spinOnce();
	stotinkasekundePostanek01.sleep();
	
	ros::spinOnce();
	stotinkasekundePostanek01.sleep();
  }
  
  printf("Izhod iz programa.");
  return 0;
  
}
