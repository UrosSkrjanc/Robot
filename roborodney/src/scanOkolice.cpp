#include <ros/ros.h>
#include "roborodney/pozicijaServoMotorja.h"
#include "roborodney/razdaljaSenzor.h"
#include <iostream>
#include <sstream>
#include <string>
#include "std_msgs/String.h"
#include "roborodney/tockeOkolje.h"


#define LASER
//#define ULTRASONIC

#if defined ULTRASONIC
	int steviloVeljavnihSkenov=3;
#endif

#if defined LASER
	int steviloVeljavnihSkenov=1;
#endif

int stevecVelajvnihSkenov=0;

#if defined LASER
float posamezneMeritve[1]={};
#endif

#if defined ULTRASONIC
float posamezneMeritve[3]={};
#endif

float razdaljaDoOvireSenzor1=0.0;
float razdaljaIzracunana=0;
std::string ukaz="";
int pinSignala=25;

int c=0;
int i=0;
int j=0;

int skeniraj=0;

void sprozisken(const std_msgs::String::ConstPtr& msg){
	std::string ukaz= msg->data.c_str();
	if (ukaz.compare("start")==0)
	{skeniraj=1;}
}

void dobiRazdaljoUltrasonica(const roborodney::razdaljaSenzor& msg)
{
 	razdaljaDoOvireSenzor1=msg.razdalja+11.0;  //zadnja stevilka je razdalja od gredi motorja do senzorja
	posamezneMeritve[j]=razdaljaDoOvireSenzor1;
	j++;
	stevecVelajvnihSkenov++;
	razdaljaIzracunana=razdaljaIzracunana+msg.razdalja;
	ROS_INFO("Razdalja %d:%f",j,msg.razdalja);
}


void dobiRazdaljoLaserja(const roborodney::razdaljaSenzor& msg)
{
 	razdaljaDoOvireSenzor1=msg.razdalja+7.0;
	posamezneMeritve[j]=razdaljaDoOvireSenzor1;
	j++;
	stevecVelajvnihSkenov++;
	razdaljaIzracunana=razdaljaIzracunana+msg.razdalja;
	ROS_INFO("Razdalja %d:%f",j,msg.razdalja);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "scanOkolice");
	ros::NodeHandle nh;
	
	ros::Rate sekundniPostanek(1);
	ros::Rate desetinciPostanek(10);
	
	#if defined LASER
	ROS_INFO("LASER!!!");
	ros::Subscriber razdaljaSenzor;
	#endif
	
	#if defined ULTRASONIC
	ROS_INFO("ULTRASONIC!!!");
	ros::Subscriber razdaljaSenzor;
	#endif
	
	ros::Publisher posiljanje = nh.advertise<roborodney::pozicijaServoMotorja>("servomotorPozicija",1000);
	ros::Subscriber sub = nh.subscribe("skenukaz", 1000, sprozisken);
	ros::Publisher tockeOkolje = nh.advertise<roborodney::tockeOkolje>("tocke_okolje", 10);
	
	roborodney::pozicijaServoMotorja sporocilo;
	sporocilo.pin=pinSignala;
	
	sekundniPostanek.sleep();
	ros::spinOnce();

	//za 180 stopinj od 0 do 180 - motor HITEC HS-322HD
	//https://www.servocity.com/hs-322hd-servo
	float min=570;
	float max=2430;
	float maxkot=180;
	float sredina=((max-min)/2)+min;
	float korakVms=(max-min)/maxkot;
	
	//SAMO testiranje izracunanih vrednosti
	ROS_INFO("%f  %f  %f  %f  %f",min,max,maxkot,sredina,korakVms);
	
	float kotSkena=180;
	float kotPremika=3; 
	float zacetniKot = (maxkot-kotSkena)/2;
	float zacetni = min + (korakVms*zacetniKot);
	float koncni = zacetni + (kotSkena*korakVms);
	
	//po vsakem premiku motorja pocaka za 0.005 sekunde na kot
	//se pravi, za 5 stopnij bi pocakal 0.025 sekundi 
	//se pravi za 180 stopnij 0.9 sekunde - kar bi moralo biti dovolj
	float postanekMotorja=0.005*kotPremika;
	float frekvenca=1/postanekMotorja;
	ros::Rate postanek(frekvenca);
	
	ROS_INFO("%f %f %f %f %f",kotSkena,kotPremika,zacetniKot,zacetni,koncni);
	
	int steviloMeritev = (kotSkena/kotPremika)+1;
	//tabela za shranjevanje razdalj
	float razdalje[(int)steviloMeritev]={};
	float razdaljeKorogirano[(int)steviloMeritev]={};
	float meritve[(int)steviloMeritev][steviloVeljavnihSkenov];
	float koti[(int)steviloMeritev]={};
	float odmikX[(int)steviloMeritev]={};
	float odmikY[(int)steviloMeritev]={};
	float radiani[(int)steviloMeritev]={};
	float pozicijaMotorja[(int)steviloMeritev]={};
	int veljavne[(int)steviloMeritev]={};
	
	
	ROS_INFO("Stevilo meritev: %d", steviloMeritev);
	
	while(nh.ok()){
	
	if(skeniraj==1){
	
	//na zacetku posljem na sredino
	sporocilo.pin=pinSignala;
	sporocilo.cas=ros::Time::now();
	sporocilo.pozicija=zacetni;
	posiljanje.publish(sporocilo);
	ros::spinOnce();
	sekundniPostanek.sleep();
	
	
	//razdaljaSenzor.shutdown();
	stevecVelajvnihSkenov=0;
	j=0;
	
	
	
	//----------------------------------------------------------------------
	for(i=0;i<=steviloMeritev;i++){
		j=0;
		ROS_INFO("Zaporedna st %d",i);
		ROS_INFO("Kot %f - pozicija %f", zacetniKot+(kotPremika*i), zacetni+(korakVms*i*kotPremika));
		razdaljaSenzor.shutdown();
		sporocilo.cas=ros::Time::now();
		sporocilo.pozicija=zacetni+(korakVms*i*kotPremika); 
		posiljanje.publish(sporocilo);
		ros::spinOnce();
		postanek.sleep();
		
		#if defined LASER
			razdaljaSenzor = nh.subscribe("razdalja_senzor", 1, dobiRazdaljoLaserja);
		#endif
	
		#if defined ULTRASONIC
			razdaljaSenzor = nh.subscribe("razdalja_senzor", 1, dobiRazdaljoUltrasonica);
		#endif
		
		while(stevecVelajvnihSkenov<steviloVeljavnihSkenov)
		{
			ros::spinOnce();
		}
		j=0; 
		
		//----------------------------------------------------------------
		//laser je toliko natančen do enega metra, da vzamem vsako meritev
		//morda sprobati kaj se zgodi, ce vzamem tri veljavne meritve tudi za laser
		#if defined LASER
			razdaljeKorogirano[i]=posamezneMeritve[0];
			razdalje[i]==posamezneMeritve[0];
			meritve[i][0]=posamezneMeritve[0];
			veljavne[i]=1;	
		#endif
		
		
		//----------------------------------------------------------------
		
		#if defined ULTRASONIC
		//Za ultrasonic vzamem tri meritve, izločim tiste, ki so od ostalih v sklopu ene pozicije, razlikujejo več kot za 10cm
		for(j=0;j<steviloVeljavnihSkenov;j++){meritve[i][j]=posamezneMeritve[j];}
		razdalje[i]=(razdaljaIzracunana/steviloVeljavnihSkenov);
		
		float zgornjaPrva=meritve[i][0]+10;
		float spodnjaPrva=meritve[i][0]-10;
		float zgornjaDruga=meritve[i][1]+10;
		float spodnjaDruga=meritve[i][1]-10;
		float zgornjaTretja=meritve[i][2]+10;
		float spodnjaTretja=meritve[i][2]-10;
		
		razdaljeKorogirano[i]=0;
		veljavne[i]=0;
		
		//ce so vse OK
		if( (meritve[i][1]<zgornjaPrva) &&  (meritve[i][1]>spodnjaPrva) && (meritve[i][2]<zgornjaPrva) &&  (meritve[i][2]>spodnjaPrva) )
		{
			razdaljeKorogirano[i]=(meritve[i][0]+meritve[i][1]+meritve[i][2])/3;
			ROS_INFO("Vse tri meritve OK");
			veljavne[i]=1;
		}
		
		//ce prva ni OK
		if( ((meritve[i][0]>zgornjaDruga) ||  (meritve[i][0]<spodnjaDruga)) && (meritve[i][2]<zgornjaDruga) &&  (meritve[i][2]>spodnjaDruga) )
		{
			razdaljeKorogirano[i]=(meritve[i][1]+meritve[i][2])/2;
			ROS_INFO("Prva ni OK OK");
			veljavne[i]=1;
		}
		
		//ce druga ni OK
		if( ((meritve[i][1]>zgornjaPrva) ||  (meritve[i][1]<spodnjaPrva)) && (meritve[i][2]<zgornjaPrva) &&  (meritve[i][2]>spodnjaPrva) )
		{
			razdaljeKorogirano[i]=(meritve[i][0]+meritve[i][2])/2;
			ROS_INFO("Druga ni OK");
			veljavne[i]=1;
		}
		
		//ce tretja ni OK
		if( ((meritve[i][2]>zgornjaPrva) ||  (meritve[i][2]<spodnjaPrva)) && (meritve[i][1]<zgornjaPrva) &&  (meritve[i][1]>spodnjaPrva) )
		{
			razdaljeKorogirano[i]=(meritve[i][0]+meritve[i][1])/2;
			ROS_INFO("Tretja ni OK");
			veljavne[i]=1;
		} 
		
		if(veljavne[i]==0){ROS_INFO("Ni blo kul!!!! - %f",razdaljeKorogirano[i]);}
		#endif
		
		//----------------------------------------------------------------
		 
		
		pozicijaMotorja[i] = zacetni+(korakVms*i*kotPremika);
		koti[i]=round((((pozicijaMotorja[i])-min)/(korakVms)) - ((maxkot-180)/2));
		radiani[i]=koti[i]*(M_PI/180);
		odmikY[i]=sin(radiani[i])*razdaljeKorogirano[i];
		odmikX[i]=-cos(radiani[i])*razdaljeKorogirano[i];
		if(razdaljeKorogirano[i]>100 || razdaljeKorogirano[i]<10 ){veljavne[i]=0;}
		ROS_INFO("%d - %f",i,razdalje[i]);
		ROS_INFO(" ");
		ROS_INFO(" ");
		ROS_INFO(" ");
		stevecVelajvnihSkenov=0;
		razdaljaIzracunana=0;
	}
	
	
	razdaljaSenzor.shutdown();
	
	//na koncu merjenj poslem na sredino
	sporocilo.cas=ros::Time::now();
	sporocilo.pozicija=sredina;
	posiljanje.publish(sporocilo);
	ros::spinOnce();
	
	std::string tocke="";
	
	//zbrisem vrednosti
	roborodney::tockeOkolje sporocilo_tocke;
	sporocilo_tocke.tocke.clear();
	sporocilo_tocke.kotiVRadianih.clear();
	sporocilo_tocke.razdalje.clear();

	
	for(i=0; i<steviloMeritev; i++)
	{
		ROS_INFO("Zap. st: %d",i);
		ROS_INFO("Razdalja: %f",razdalje[i]);
		#if defined ULTRASONIC
		ROS_INFO("Posamezne merive %f  %f  %f", meritve[i][0],meritve[i][1],meritve[i][2]);
		#endif
		#if defined LASER
		ROS_INFO("Posamezne merive %f ", meritve[i][0]);
		#endif
		ROS_INFO("Korigirana Razdalja: %f",razdaljeKorogirano[i]);
		ROS_INFO("Kot: %f",koti[i]);
		ROS_INFO("Pozicija motorja: %f",pozicijaMotorja[i]);
		ROS_INFO("Radian: %f",radiani[i]);
		ROS_INFO("Odmik y: %f",odmikY[i]);
		ROS_INFO("Odmik x: %f",odmikX[i]);
		ROS_INFO("Velajvno: %d",veljavne[i]);
		ROS_INFO(" ");
		//if(veljavne[i]==1){
			std::stringstream ss;
			ss << ",(" << odmikX[i] << "," << odmikY[i] << ")";
			tocke = tocke + ss.str();
		//}
		geometry_msgs::Point tocka;
		tocka.x = odmikX[i];
		tocka.y = odmikY[i];
		sporocilo_tocke.tocke.push_back(tocka);
		sporocilo_tocke.kotiVRadianih.push_back(radiani[i]);
		sporocilo_tocke.razdalje.push_back(razdaljeKorogirano[i]);
	}
	
	skeniraj=0;
	
	//še pošlje točke na topic tocke_okolje
	sporocilo_tocke.cas = ros::Time::now();
	tockeOkolje.publish(sporocilo_tocke);
	
	//------------------------------------------------------------------
	}
	
	ros::spinOnce();
	sekundniPostanek.sleep();
	
}

	return 0;
}
