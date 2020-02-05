#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <termios.h>
#include "std_msgs/String.h"
#include <string>
#include "roborodney/ukazVoznje.h"

//pini na L298N konotrolerju 
//levi in desni motor se gledata glede na levo in desno stran robota
//kontroler je postavljen tako, da gleda hladilnik nazaj, konektorji so spredaj
//sprednja stran robota je tam, kjer je kamera in raspberry pi na plosci
//int in1 = 23;  //desni motor
//int in2 = 24;  //desni motor
//int in3 = 27;  //levi motor
// int in4 = 22;  //levi motor 
int in1 = 5;  //desni motor
int in2 = 6;  //desni motor
int in3 = 19;  //levi motor
int in4 = 13;  //levi motor 
int hitrostNaprejNazaj=100;
int hitrostLevoDesno=100;
int trajanjeNaprejNazaj=100; // v tisocinkah
int trajanjeLevoDesno=100;  // v tisocinkah
float razdaljaDoOvireSenzor1=0.0;
float razdaljaDoOvireSenzor2=0.0;




void naprej(int trajanje, int hitrost)
{
  softPwmWrite (in1, hitrost);  
  softPwmWrite (in3, hitrost);
  delay(trajanje);
  softPwmWrite (in1, 0);  
  softPwmWrite (in3, 0);
}

void nazaj(int trajanje, int hitrost)
{
  softPwmWrite (in2, hitrost);  
  softPwmWrite (in4, hitrost);
  delay(trajanje);
  softPwmWrite (in2, 0);  
  softPwmWrite (in4, 0);
}

void levo(int trajanje, int hitrost)
{
  softPwmWrite (in2, hitrost);  
  softPwmWrite (in3, hitrost);
  delay(trajanje);
  softPwmWrite (in2, 0);  
  softPwmWrite (in3, 0);
  ROS_INFO("SEL LEVO!!!");
}

void desno(int trajanje, int hitrost)
{
  softPwmWrite (in4, hitrost);  
  softPwmWrite (in1, hitrost);
  delay(trajanje);
  softPwmWrite (in4, 0);  
  softPwmWrite (in1, 0);
}

void krog(int trajanje, int hitrost)
{
  for (int i=0;i<72;i++){
  softPwmWrite (in4, hitrost);  
  softPwmWrite (in1, hitrost);
  delay(trajanje);
  softPwmWrite (in4, 0);  
  softPwmWrite (in1, 0);
  delay(500);
  }
}


void izlusciUkaz(const roborodney::ukazVoznje& msg)
{
  std::string smer = msg.smer.c_str();
  int hitrost = msg.hitrost;
  int trajanje = msg.trajanje;

  if (smer.compare("levo")==0){ROS_INFO("LEVO!!!");levo(trajanje,hitrost);}
  if (smer.compare("desno")==0){ROS_INFO("DESNO!!!");desno(trajanje,hitrost);}
  if (smer.compare("naprej")==0){ROS_INFO("NAPREJ!!!");naprej(trajanje,hitrost);}
  if (smer.compare("nazaj")==0){ROS_INFO("NAZAJ!!!");nazaj(trajanje,hitrost);}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  ros::Subscriber ukaz_smeri = nh.subscribe("ukaz_smeri", 1, izlusciUkaz);


  setenv("WIRINGPI_GPIOMEM", "1", 1);

  wiringPiSetupGpio();
  pinMode(in4,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in1,OUTPUT);

  softPwmCreate (in4, 0, 100);
  softPwmCreate (in3, 0, 100);
  softPwmCreate (in2, 0, 100);
  softPwmCreate (in1, 0, 100);

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  printf("GREMO!!! \n");


  int c=0;

  ros::spin();

  printf("CIAO!!! \n");
  return 0;
}




