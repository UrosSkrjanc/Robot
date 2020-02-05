#include <ros/ros.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>

#include "roborodney/razdaljaSenzor.h"
#include "roborodney/ukazVoznje.h"
#include "roborodney/tockeOkolje.h"
// #include "roborodney/pozicijaMotorja.h"
#include "roborodney/pozicijaObraza.h"
#include "roborodney/pozicijaServoMotorja.h"


using namespace cv;
using namespace std;


//globalni publisherji in subscriberji
ros::Publisher senzorlevodesno;
ros::Publisher skenUkaz;
ros::Publisher ukaz_smeri;


// globalne spremenljivke
int zaklenjen=0;
int potekaSken=0;
float trenutnaRazdalja=0; //za merjenje razdalje od robota do ovire
//float zadnjaPozicijaMotorja=0; //ta spremenljivka samo za testno verzijo, kjer je predlagal smer glede na pozicijo servo motorja, kjer je zaznal oviro - nepotrebna, tako kot funkcija dobiPozicijoMotorja
float trenutniKot=0;
string smer="stop";
int kotRotacije=30; //samo za testiranje obrata, tipki q (obrat levo za 30 stopinj) in w (obrat levo za 30 stopinj)
//spremenljivka, ki doloca smer
float kotcilja=0;
//spremenljivka ki pobere zadnjo smer, preden se zacne nekontrolirano obracati
float zadnjiDobriKot=0;
//spremenljivka, ki dobi vrednost, ce se je robot predolgo vrtel
int predolgoVrtenje=0;
//spremenljivka, ki steje koliko korakov gre zaporedoma naprej in na 200 pregleda, ce je v pravi smeri
int korakiNaprejStevec=0;


//spremenljivke ki povesta, ali je robot pred kratkim zaznal QR kodo ali obraz
string ukazQRkode="";
int naletelNaOviro=0;
int naletelNaQR=0;
double xx1,xx2,yy1,yy2, resx, resy, xcenter, xcenterslike;
bool zaznanObraz=false; 


//uposteva vse razdalja pri zaznavanju ovire, ki so med tema dvema vrednostima
int maxVarnostnaRazdalja=30; 
int minVarnostnaRazdalja=5;

//za posiljanje ukaza voznje - naprej, nazaj, levo desno
roborodney::ukazVoznje ukaz;



//za zaznavanje okolice - spremenljivke v funkciji dobiTockeOkolja
cv::Mat matrika(400,400,CV_8UC1,cv::Scalar(255));
cv::Mat matrikaUtezi(400,400,CV_8UC1,cv::Scalar(255));
cv::Mat matrikaUteziInv(400,400,CV_8UC1,cv::Scalar(255));
std::vector<float>kotiGraf;
std::vector<float>vrednostiGraf;
int stKotovGraf = 19;
std::vector<Mat> matrikeUtezi;
std::string tocke="";
std::string razdalje="";
vector<geometry_msgs::Point> Tocke;
vector<float>kotiScan;
vector<float>razdaljaScan;

//samo napoved funkcije, prototip - morda bi naredil prototipe za vse funkcije
void rotacijaRobota(int kot);


//funkcija, ki s topica razdalja_senzor (ultrasonic in laser) dobiva vrednosti - oddaljenost robota od ovire 
void dobiRazdaljoSenzorja(const roborodney::razdaljaSenzor& msg)
{
	trenutnaRazdalja = msg.razdalja;
	std_msgs::String msgg;
	if(msg.razdalja>minVarnostnaRazdalja && msg.razdalja<maxVarnostnaRazdalja && zaklenjen==0 && potekaSken==0)
	{
		//robot se zaklene
		zaklenjen=1;
		naletelNaOviro=1;
		
		//ustavi glavo
		msgg.data="stop";
		senzorlevodesno.publish(msgg);
	}
}


//funkcija ki dobiva vrednost qr kode
void funkcijaqrKoda(const std_msgs::String& msg){ 
	naletelNaQR=1;
		
	ROS_INFO("Dobil ukaz z QR kode: %s",msg.data.c_str());
	
	if(msg.data=="LEFT"){

		ukazQRkode="LEFT";
	}
	if(msg.data=="RIGHT"){

		ukazQRkode="RIGHT";
	}
	if(msg.data=="TURN"){

		ukazQRkode="TURN";
	}
	if(msg.data=="CONTINUE"){

		ukazQRkode="CONTINUE";
	}
	if(msg.data=="STOP"){

		ukazQRkode="STOP";
	}	
}

//funkcija, ki dobi tocke okolja in jih preracuna v sliko iz katere se odloci v katero smer bi sel
void dobiTockeOkolja(const roborodney::tockeOkolje& msg)
{
	ros::Rate desetinkasekundePostanek01(10);
	
	kotiGraf.clear();
	vrednostiGraf.clear();
	matrikeUtezi.clear();
	Tocke.clear();
	razdaljaScan.clear();
	tocke="";
	razdalje="";
	
	//zanka prebere dobljene tocke iz topica in jih spremeni v strin - za potrebe zapisovanja prejetih tock v datoteko
	for(int i=0;i<msg.tocke.size();i++)
	{
		Tocke.push_back(msg.tocke[i]);
		razdaljaScan.push_back(msg.razdalje[i]);
		std::stringstream ss;
		std::stringstream ss0;
		ss << ",(" << msg.tocke[i].x << "," << msg.tocke[i].y << ")";
		ss0 << ","<< msg.razdalje[i];
		tocke = tocke + ss.str();
		razdalje = razdalje + ss0.str();
	}
	potekaSken=0;
	std::stringstream ss1,ss2;
	ros::Time moment = ros::Time::now();
	ss1 << "meritev" << moment.sec << ".txt";
	std::string imefajla = ss1.str();
	std::ofstream out(imefajla.c_str());
    out << tocke;
	out << "\n\n" << razdalje;
	out << " ";
	out.close();
	
	//zanka, ki narise slike s pomocjo katerih se odloca v katero smer gre
	//te slike zapise tudi na disk kot jpg
	matrika = cv::Scalar(255);
	matrikaUtezi = cv::Scalar(255);
	matrikaUteziInv = cv::Scalar(255);
	for(int i=0;i<Tocke.size();i++) {
		if(Tocke[i].x<200 && Tocke[i].x>-200 && Tocke[i].y<200 && Tocke[i].y>5) {
			int x=(int)(Tocke[i].x+200); 
			int y=(int)(400-Tocke[i].y);
			matrika.at<uchar>(y,x)=0;
			int utezRazdalje=(fabs(100-razdaljaScan[i])/100)*30;
			matrikaUtezi.col(x).row(y)=matrikaUtezi.col(x).row(y)-15;
			for(int j=x-1; j<=x+1; j++){
				for(int k=y-1;k<=y+1; k++){
					matrikaUtezi.col(j).row(k)=matrikaUtezi.col(j).row(k)-utezRazdalje;
				}
			}
			for(int j=x-2; j<=x+2; j++){
				for(int k=y-2;k<=y+2; k++){
					matrikaUtezi.col(j).row(k)=matrikaUtezi.col(j).row(k)-utezRazdalje;
				}
			}
			for(int j=x-3; j<=x+3; j++){
				for(int k=y-3;k<=y+3; k++){
					matrikaUtezi.col(j).row(k)=matrikaUtezi.col(j).row(k)-utezRazdalje;
				}
			}
			for(int j=x-4; j<=x+4; j++){
				for(int k=y-4;k<=y+4; k++){
					matrikaUtezi.col(j).row(k)=matrikaUtezi.col(j).row(k)-utezRazdalje;
				}
			}
		}
	}
	ss1.str("");
	ss1 << "original_" << moment.sec << ".jpg";
	imefajla = ss1.str();
	imwrite(imefajla,matrika);
	ss1.str("");
	ss1 << "utezi_" << moment.sec << ".jpg";
	imefajla = ss1.str();
	imwrite(imefajla,matrikaUtezi);
	
	
	
	//Narejen kartezijski histogram
	//Naredit polarni histogram - razdelit kartezijski na 18 delov, po 10 stopinj in potem sešteti vse celice zbotraj
	//To bom naredil tako, da bom najprej naredil inverz poalrnega histograma - zato, ker je črna enaka 0, jaz pa potebujem ravno utez
	//Se pravi, kjer je bela je vecja vrednost - tako da naredim inverz in potem razdelim
	for(int i=0;i<400;i++){
		for(int j=0;j<400;j++){
			matrikaUteziInv.row(i).col(j) = 255-matrikaUtezi.row(i).col(j);
		}
	}
	
	
	//Tukaj imam inverzno matriko z utezmi. Sedaj bi moral kopije, 18, spraviti v vector 
	std::vector<float>radiani;
	for(int i=0;i<stKotovGraf;i++){
		Mat zacasna;
		matrikaUteziInv.copyTo(zacasna);
		matrikeUtezi.push_back(zacasna);
		radiani.push_back(i*180/((float)stKotovGraf)*(M_PI/180));
	}
	
	
	//tukaj imam v vektroju matrike z utezmi in radiane.
	//sedaj matrikam dolocim trikotnik
	Mat matrikaUteziInvCrte;
	matrikaUteziInv.copyTo(matrikaUteziInvCrte);
	vector <float>exYi;
	vector <float>Yi;
	vector <float>YiMax;
	exYi.clear();	
	Yi.clear();
	for(int i=0;i<matrikeUtezi.size();i++){
		exYi.push_back(0);
		YiMax.push_back(0);
	}
	
	
	// zanka, ki naredi vse posamezne slike prostora, iz katerih potem racuna ovire v okolici
	for(int i=0;i<200;i++){
		//izracunam vse aktualne y-e
		exYi=Yi;
		Yi.clear();
		for(int j=0;j<matrikeUtezi.size();j++){
			float yTocka=tan(radiani[j])*i;
			Yi.push_back(yTocka);
			if(fabs(Yi[j])>YiMax[j]&&fabs(Yi[j])<399){YiMax[j]=fabs(Yi[j]);}
		}
		Yi.push_back(0);
		
		
		for(int j=0;j<fabs(matrikeUtezi.size()/2);j++){
			if(Yi[j]<399 && Yi[j]>-399){
				matrikaUteziInvCrte.col(200-i).row(399-fabs(Yi[j]))=255;
				matrikaUteziInvCrte.col(199+i).row(399-fabs(Yi[j]))=255;
				matrikeUtezi[j].colRange(0,200-i).row(399-fabs(Yi[j]))=0;
				matrikeUtezi[stKotovGraf-1-j].colRange(199+i,399).row(399-fabs(Yi[j]))=0;
				for(int k=fabs(exYi[j])+1;k<fabs(Yi[j]);k++) {
					matrikaUteziInvCrte.col(200-i).row(399-k)=255;
					matrikaUteziInvCrte.col(199+i).row(399-k)=255;
					matrikeUtezi[j].colRange(0,200-i).row(399-k)=0;
					matrikeUtezi[stKotovGraf-1-j].colRange(199+i,399).row(399-k)=0;
				}
				if(Yi[j+1]<399 && Yi[j+1]>-399){
					matrikeUtezi[j].colRange(200-i,399).row(399-fabs(Yi[j+1]))=0;
					matrikeUtezi[stKotovGraf-1-j].colRange(0,199+i).row(399-fabs(Yi[j+1]))=0;
					for(int k=fabs(exYi[j+1])+1;k<fabs(Yi[j+1]);k++) {
						matrikeUtezi[j].colRange(200-i,399).row(399-k)=0;
						matrikeUtezi[stKotovGraf-1-j].colRange(0,199+i).row(399-k)=0;
					}
				}
			}
		}
		
		
		//se sredinsko sliko naredit
		int j = fabs(matrikeUtezi.size()/2);
		if(Yi[j]<399 && Yi[j]>-399){
			matrikaUteziInvCrte.col(200-i).row(399-fabs(Yi[j]))=255;
			matrikaUteziInvCrte.col(199+i).row(399-fabs(Yi[j]))=255;
			matrikeUtezi[j].colRange(0,200-i).row(399-fabs(Yi[j]))=0;
			for(int k=fabs(exYi[j])+1;k<fabs(Yi[j]);k++) {
				matrikaUteziInvCrte.col(200-i).row(399-k)=255;
				matrikaUteziInvCrte.col(199+i).row(399-k)=255;
				matrikeUtezi[j].colRange(0,200-i).row(399-k)=0;
			}
			if(Yi[j+1]<399 && Yi[j+1]>-399){
				matrikeUtezi[j].colRange(200+i,399).row(399-fabs(Yi[j+1]))=0;
				for(int k=fabs(exYi[j+1])+1;k<fabs(Yi[j+1]);k++) {
					matrikeUtezi[j].colRange(200+i,399).row(399-k)=0;
				}
			}
		}
	}
	
	

	//zbrise se zgornji del, posebej pomembno pri robnih slikah
	for(int j=0;j<fabs(matrikeUtezi.size()/2);j++){
		matrikeUtezi[j].colRange(0,399).rowRange(0, 399-YiMax[j+1])=0;
		matrikeUtezi[stKotovGraf-1-j].colRange(0,399).rowRange(0, 399-YiMax[stKotovGraf-1-j])=0;
	}
	
	int j = fabs(matrikeUtezi.size()/2);
	matrikeUtezi[j].colRange(0,399).rowRange(0, 399-YiMax[j])=0;
	
	//shrani slike kjer so na originalni sliki in inverzni sliki narisane mejne crte
	ss1.str("");
	ss1 << "inverz_" << moment.sec << ".jpg";
	imefajla = ss1.str();
	imwrite(imefajla,matrikaUteziInv);
	ss1.str("");
	ss1 << "inverzCrte_" << moment.sec << ".jpg";
	imefajla = ss1.str();
	imwrite(imefajla,matrikaUteziInvCrte);
	
	//shrani slike posameznih delov iz katerih izracuna ovire v prostoru
	for(int j=0;j<matrikeUtezi.size();j++){
		ss1.str("");
		if(j<10){
			ss1 << "inverz_0"<<j<<"_" << moment.sec <<".jpg";
		} else {
			ss1 << "inverz_"<<j<<"_" << moment.sec <<".jpg";
		}
		imefajla = ss1.str();
		imwrite(imefajla,matrikeUtezi[j]);
	}
	

	
			
	https://stackoverflow.com/questions/21874774/sum-of-elements-in-a-matrix-in-opencv
	
	
	ROS_INFO("TESTNI IZPIS - NOV NACIN");
	kotiGraf.clear();
	vrednostiGraf.clear();
	
	//zapisem se v datoteko
	ofstream myfile;
	ss2 << "izbiraKot" << moment.sec << ".txt";
	std::string imefajla2 = ss2.str();
	std::ofstream out2(imefajla2.c_str());	
	out2 << "Razdelim na " << stKotovGraf << "\n\n";
	
	for(int i=0;i<matrikeUtezi.size();i++){
		kotiGraf.push_back(i*(180.0/matrikeUtezi.size()));
		vrednostiGraf.push_back(sum(matrikeUtezi[i])[0]);
		out2 << i << " " << kotiGraf[i] << " " << vrednostiGraf[i] << "\n";
	}
	
	//shranjevanje tabel v excel za test - po potrebi
	
	/*
	cv::FileStorage file("tabela.txt", cv::FileStorage::WRITE);
	cv::FileStorage file01("tabela01.txt", cv::FileStorage::WRITE);
	cv::FileStorage file02("tabela02.txt", cv::FileStorage::WRITE);
	cv::FileStorage file03("tabela03.txt", cv::FileStorage::WRITE);
	cv::FileStorage file04("tabela04.txt", cv::FileStorage::WRITE);
	cv::FileStorage file05("tabela05.txt", cv::FileStorage::WRITE);
	cv::FileStorage file06("tabela06.txt", cv::FileStorage::WRITE);
	cv::FileStorage file07("tabela07.txt", cv::FileStorage::WRITE);
	cv::FileStorage file08("tabela08.txt", cv::FileStorage::WRITE);
	cv::FileStorage file09("tabela09.txt", cv::FileStorage::WRITE);
	cv::FileStorage file10("tabela10.txt", cv::FileStorage::WRITE);
	cv::FileStorage file11("tabela11.txt", cv::FileStorage::WRITE);
	cv::FileStorage file12("tabela12.txt", cv::FileStorage::WRITE);
	cv::FileStorage file13("tabela13.txt", cv::FileStorage::WRITE);
	cv::FileStorage file14("tabela14.txt", cv::FileStorage::WRITE);
	cv::FileStorage file15("tabela15.txt", cv::FileStorage::WRITE);
	cv::FileStorage file16("tabela16.txt", cv::FileStorage::WRITE);
	cv::FileStorage file17("tabela17.txt", cv::FileStorage::WRITE);
	*/
	
	/*
	file << "matName" << matrikaUteziInv;
	file01 << "matName01" << matrikaUteziInv1;
	file02 << "matName02" << matrikaUteziInv2;
	file03 << "matName03" << matrikaUteziInv3;
	file04 << "matName04" << matrikaUteziInv4;
	file05 << "matName05" << matrikaUteziInv5;
	file06 << "matName06" << matrikaUteziInv6;
	file07 << "matName07" << matrikaUteziInv7;
	file08 << "matName08" << matrikaUteziInv8;
	file09 << "matName09" << matrikaUteziInv9;
	file10 << "matName10" << matrikaUteziInv10;
	file11 << "matName11" << matrikaUteziInv11;
	file12 << "matName12" << matrikaUteziInv12;
	file13 << "matName13" << matrikaUteziInv13;
	file14 << "matName14" << matrikaUteziInv14;
	file15 << "matName15" << matrikaUteziInv15;
	file16 << "matName16" << matrikaUteziInv16;
	file17 << "matName17" << matrikaUteziInv17;
	*/
	
	
	//sedaj se izracunam kje je najmanjsa vrednost, razen krajnih
	float minimum=100000;
	int pozicija=0;
	
	//berem iz teh
	//	kotiGraf
	//  vrednostiGraf
	
	out2 << "\n\n";
	
	for(int i=4;i<kotiGraf.size()-4;i++){
		float trenutnaVrednost = (0.5*vrednostiGraf[i-1])+(2*vrednostiGraf[i])+(0.5*vrednostiGraf[i+1]);
		ROS_INFO("%d %d %d - %f min-%f",i-1,i,i+1,trenutnaVrednost,minimum);
		out2 << i-1 << " " << i << " " << i+1 << " " << trenutnaVrednost << " " << minimum << "\n";
		if(minimum>trenutnaVrednost){
			minimum=trenutnaVrednost;
			pozicija=i;
		}
	}
	
	ROS_INFO("%d %f",pozicija,minimum);
	ROS_INFO("Med kotoma %f %f",kotiGraf[pozicija-1],kotiGraf[pozicija+1]);
	ROS_INFO("Kot %f",((kotiGraf[pozicija+1]-kotiGraf[pozicija])/2)+kotiGraf[pozicija]);
	out2 << "\n\n";
	out2 << pozicija << " " << minimum << "\n";
	out2 << "Med kotoma " <<  kotiGraf[pozicija-1] << " " << kotiGraf[pozicija+1] << "\n";
	out2 << "Kot glede na robota " <<  ((kotiGraf[pozicija+1]-kotiGraf[pozicija])/2)+kotiGraf[pozicija] << "\n";
	
	float kotPremika = 90 - (((kotiGraf[pozicija+1]-kotiGraf[pozicija])/2)+kotiGraf[pozicija]);
	out2 << "Robota premaknem za kot " << kotPremika << "\n";
	if(kotPremika>0){ROS_INFO("levo za kot %f",kotPremika);}
	if(kotPremika==0){ROS_INFO("Naravnost");}
	if(kotPremika<0){ROS_INFO("desno za kot %f",kotPremika);}
	
	out2.close();
	
	int kotObrata = int(kotPremika);
	ROS_INFO("OBRNEM ZA %d",kotObrata);
	
	ROS_INFO("Naj ga obrnem? (y/n)");
	char c;
	cin.get(c);
	
	if(c=='y'){
		ROS_INFO("potem ga obrnem");
		ros::spinOnce();
		desetinkasekundePostanek01.sleep();
		desetinkasekundePostanek01.sleep();
		desetinkasekundePostanek01.sleep();
		zadnjiDobriKot = trenutniKot;
		rotacijaRobota(kotObrata);
	} else {
		ROS_INFO("ga ne obrnem");
	} 
	
	ROS_INFO("Stanje PREDOLGO VRTENJE!!! %d",predolgoVrtenje);
	
	if(predolgoVrtenje==1){
		ros::spinOnce();
		desetinkasekundePostanek01.sleep();
		desetinkasekundePostanek01.sleep();
		desetinkasekundePostanek01.sleep();
		ROS_INFO("V funkcijo dobi tocke sem zaznal predolgo vrtenje. Moral bi ga zavrteti nazaj");
		ROS_INFO("Zadnji dobri kot % f  TrenutniKot %f", zadnjiDobriKot,trenutniKot);
		predolgoVrtenje=0; 
		
		ROS_INFO("Naj ga zavrtim v zadnjo dobro smer? y/n");
		char c;
		cin >> c;
		if(c=='y'){
			int obrat=zadnjiDobriKot-trenutniKot;
			int obrat2=obrat;
			if(abs(obrat)>180){
				obrat2 = 360-abs(obrat);
				if(obrat>0){
					obrat2=-obrat2;
				}
			}
			ROS_INFO("Obrat za kot %d",obrat2);
			rotacijaRobota(obrat2);
		}
	}
	
	zaklenjen=0;
	naletelNaOviro=0;
		
}

//tukaj bi dodal se houghtransform

/*
void dobiPozicijoMotorja(const izogib::pozicijamotorja& msg)
{
	zadnjaPozicijaMotorja = msg.pozicija;
}
*/


void dobiKotVProstoru(const sensor_msgs::Imu& msg)
{
	ros::Rate sekundniPostanek01(1);
	
	//racunam se yaw
	double siny_cosp = +2.0 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y);
	double cosy_cosp = +1.0 - 2.0 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z);
	double kot = atan2(siny_cosp, cosy_cosp);
	double kotkot = (kot*180)/3.14;
	trenutniKot=kotkot;
	
	if(isnan(msg.orientation.x)){
		ROS_INFO("BNO055 DOWN!!!!");
		ROS_INFO("Naj ga restartam? (y/n)");
		char c;
		cin.get(c);		
		if(c=='y'){
			//system("reboot");
			system ("rosnode kill bno055_i2c_node");
			sekundniPostanek01.sleep();
			system("rosrun bno055 bno055_i2c_node");
			sekundniPostanek01.sleep();
		}
	}
}

void rotacijaRobota(int kot)
{
	ros::Rate sekundniPostanek01(1);
	ros::Rate desetinkasekundePostanek01(10); 
	ros::Rate stotinkasekundePostanek01(100);
	
	ros::spinOnce();
	desetinkasekundePostanek01.sleep();
	ros::spinOnce();
	
	float zacetniKot = trenutniKot;
	float ciljniKot = 0;
	//varnostni kot je zato, da se robot ne obrne prevec
	//kolicina, ki jo je potrebno nastaviti rocno
	float varnostniKot=5;
	int steviloObratov=0;
	
	ukaz.hitrost=50; 
	ukaz.trajanje=20;  
	
	//ocitno je potrebno, ce se robot preveckrat zavrti, pocakati kako sekundo in nekajkrat zagnati spinonce
	//
		
	if(kot > 0)
	{
		ROS_INFO("ZASUK V LEVO");
		ciljniKot = trenutniKot+kot-varnostniKot;
		if(ciljniKot>180){ciljniKot=ciljniKot-360;}
		
		ROS_INFO("CILJNI KOT - %f",ciljniKot);
		
		if(zacetniKot>ciljniKot)
		{
			while(trenutniKot>0){
				if(steviloObratov>200){
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					ROS_INFO("Predolgo se obracam!!");
					ROS_INFO("Zadnji dobri kot %f",zadnjiDobriKot);
					ROS_INFO("Trenutni kot %f",trenutniKot);
					predolgoVrtenje=1;
					return;
				}
				ukaz.smer="levo";
				ukaz_smeri.publish(ukaz);
				steviloObratov++;
				ros::spinOnce();
				for(int i=0;i<3;i++){ros::spinOnce();stotinkasekundePostanek01.sleep();} 
				ROS_INFO("A");
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
			}
		}
		
		while(trenutniKot<ciljniKot)
		{
			ROS_INFO("%d - BB %f",steviloObratov,trenutniKot);
			if(steviloObratov>200){ 
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				ros::spinOnce();
				ROS_INFO("Predolgo se obracam!!");
				ROS_INFO("Zadnji dobri kot %f",zadnjiDobriKot);
				ROS_INFO("Trenutni kot %f",trenutniKot);
				predolgoVrtenje=1;
				return;
			} 
			ukaz.smer="levo";
			ukaz_smeri.publish(ukaz);
			steviloObratov++;
			ros::spinOnce();
			for(int i=0;i<3;i++){ros::spinOnce();stotinkasekundePostanek01.sleep();}
			ros::spinOnce();
			stotinkasekundePostanek01.sleep();
		}

	} else 
	{
		ROS_INFO("ZASUK V DESNO");
		ciljniKot = trenutniKot+kot+varnostniKot;
		if(ciljniKot<-180){ciljniKot=ciljniKot+360;} 
		
		ROS_INFO("CILJNI KOT - %f",ciljniKot);
		
		if(zacetniKot<ciljniKot)
		{
			while(trenutniKot<0){
				ROS_INFO("%d - CC %f",steviloObratov,trenutniKot);
				if(steviloObratov>100){
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					ros::spinOnce();
					ROS_INFO("Predolgo se obracam!!");
					ROS_INFO("Zadnji dobri kot %f",zadnjiDobriKot);
					ROS_INFO("Trenutni kot %f",trenutniKot);
					predolgoVrtenje=1;
					return;
				}
				ukaz.smer="desno";
				ukaz_smeri.publish(ukaz);
				steviloObratov++;
				ros::spinOnce();
				for(int i=0;i<3;i++){ros::spinOnce();stotinkasekundePostanek01.sleep();}
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
			}
		}
		
		while(trenutniKot>ciljniKot)
		{
			ROS_INFO("%d - DD %f",steviloObratov,trenutniKot);
			if(steviloObratov>100){
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				stotinkasekundePostanek01.sleep();
				ros::spinOnce();
				ROS_INFO("Predolgo se obracam!!");
				ROS_INFO("Zadnji dobri kot %f",zadnjiDobriKot);
				ROS_INFO("Trenutni kot %f",trenutniKot);
				predolgoVrtenje=1;
				return;
			}
			ukaz.smer="desno";
			ukaz_smeri.publish(ukaz);
			steviloObratov++;
			ros::spinOnce();
			for(int i=0;i<3;i++){ros::spinOnce();stotinkasekundePostanek01.sleep();}
			ros::spinOnce();
			stotinkasekundePostanek01.sleep();
		}
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
	ROS_INFO("Hitrost: %d  Trajanje: %d Kot: %f Ciljni kot: %f", ukaz.hitrost, ukaz.trajanje, trenutniKot, kotcilja);
}

//funkcija z pridobivanje pozicije zaznanega obraza
void funkcijaPozicijaObraza(const roborodney::pozicijaObraza& msg)
{
    if (msg.p2.x>0 && msg.p2.y>0) 
    {
        ROS_INFO("Zaznal obraz!!!");
        zaznanObraz=true;
        xx1=msg.p1.x;
        yy1=msg.p1.y;
        xx2=msg.p2.x;
        yy2=msg.p2.y;     
        resx=msg.res.x;
        resy=msg.res.y;
    }
}




int main(int argc, char **argv)
{
	
	int stevec=0;
	
	ros::init(argc, argv, "voznja_Robota");
	ros::NodeHandle nh;
	
	ros::Rate sekundniPostanek01(1);
	ros::Rate desetinkasekundePostanek01(10);
	ros::Rate stotinkasekundePostanek01(100);
	ros::Rate dvajsetinkasekundePostanek01(20);

	
	ukaz_smeri = nh.advertise<roborodney::ukazVoznje>("ukaz_smeri",10);
	senzorlevodesno = nh.advertise<std_msgs::String>("senzorlevodesno", 10);
	skenUkaz = nh.advertise<std_msgs::String>("skenukaz", 10);
	ros::Subscriber razdaljaSenzor = nh.subscribe("razdalja_senzor", 1, dobiRazdaljoSenzorja);
	ros::Subscriber tockeOkolja = nh.subscribe("tocke_okolje", 1, dobiTockeOkolja);
	//ros::Subscriber pozicijaMotorja = nh.subscribe("pozicijaMotorja", 1, dobiPozicijoMotorja);
	ros::Subscriber kotVProstoru = nh.subscribe("data", 1, dobiKotVProstoru);
	ros::Subscriber pozicijaObraza = nh.subscribe("pozicija_obraza", 1, funkcijaPozicijaObraza);
	ros::Publisher posiljanjeServo = nh.advertise<roborodney::pozicijaServoMotorja>("servomotorPozicija",1000);
	ros::Subscriber qrKoda = nh.subscribe("barcode", 1, funkcijaqrKoda);
	ros::Publisher ukazSkener = nh.advertise<std_msgs::String>("ukazskener", 10);
	
	//dobiKotVProstoru

	
	std_msgs::String msg;
	msg.data="start";
	sekundniPostanek01.sleep();
	ros::spinOnce();
	
	ukaz.smer=smer;
	ukaz.hitrost=30;
	ukaz.trajanje=20;

	sekundniPostanek01.sleep();
	ros::spinOnce();
	
 
	ROS_INFO("Delujem!");
	//začne šteti čas v sekundah
	double startSekunde = ros::Time::now().toSec();
	
	kotcilja=trenutniKot;
	
	  int c=0;
  while(c !=27)
  {
	if(potekaSken==0)
	{
		
			
	set_mode(1);
    tcflush(STDIN_FILENO, TCIFLUSH);
    fflush(stdout);
	
	c = tolower(get_key());
	
	//ukaz naprej
	if (c=='i')
	{
		//zaznanObraz=false;
		korakiNaprejStevec=0;
		if(zaklenjen==0)
		{
			stevec++;ROS_INFO("NAPREJ!!! %d", stevec);
			msg.data="start";
			senzorlevodesno.publish(msg);
			ros::spinOnce();
			desetinkasekundePostanek01.sleep();
			for(int i=0;i<20;i++){ros::spinOnce();desetinkasekundePostanek01.sleep();}
			//if(zaklenjen==0)
			//{
				while(zaklenjen==0){
					//if(zaznanObraz){ROS_INFO("ZAZNAL OBRAZ!!!!");break;}
					ROS_INFO("Korak naprej %d",korakiNaprejStevec);
					ukaz.smer="naprej";
					ukaz.hitrost=20;
					ukaz.trajanje=20;
					ukaz_smeri.publish(ukaz);
					ros::spinOnce();
					stotinkasekundePostanek01.sleep();
					korakiNaprejStevec++;
					if(korakiNaprejStevec==300){
						korakiNaprejStevec=0;
						if(abs(kotcilja-trenutniKot)>5){
							ROS_INFO("Trenutni:%f  Koncni:%f",trenutniKot,kotcilja);
							ROS_INFO("Ga obrnem v ciljno smer?"); 
							char c;
							cin >> c;
							if(c=='y'){
								//----------------------------------------
								ros::spinOnce();
								desetinkasekundePostanek01.sleep();
								desetinkasekundePostanek01.sleep();
								desetinkasekundePostanek01.sleep();
								ROS_INFO("obrnil bi ga v pravo smer");
								ROS_INFO("Ciljni kot % f  TrenutniKot %f", kotcilja,trenutniKot);
								int novZacetni=trenutniKot;
								int novKoncni=kotcilja;
								int obrat=novKoncni-novZacetni;
								//if(trenutniKot<0){novZacetni=-trenutniKot;} else {novZacetni=trenutniKot;}
								//if(kotcilja<0){novKoncni=-kotcilja;} else {novKoncni=kotcilja;}
								//obrat=novKoncni-novZacetni;
								ROS_INFO("Obrat za kot %d",obrat);
								rotacijaRobota(obrat);
							//----------------------------------------
							}
						}
					}
				}
			
			/*
			//gre se malce nazaj
			ukaz.smer="nazaj";
			ukaz.hitrost=50;
			ukaz.trajanje=50;
			for(int i=0;i<10;i++){
				ukaz_smeri.publish(ukaz);
				ros::spinOnce();
				dvajsetinkasekundePostanek01.sleep();
			}
			*/
			
			
			//ROS_INFO("ZADNJA POZICIJA MOTORJA: %f", zadnjaPozicijaMotorja);
			/*if(zadnjaPozicijaMotorja<1500)
			{
				ROS_INFO("PREDLAGAM DESNO");
			} else
			{
				ROS_INFO("PREDLAGAM LEVO");
			}
			} else 
			{
				ROS_INFO("Robot je zaklenjen!!!");
				ROS_INFO("ZADNJA POZICIJA MOTORJA: %f", zadnjaPozicijaMotorja);
				if(zadnjaPozicijaMotorja<1500){ROS_INFO("PREDLAGAM DESNO");} else {ROS_INFO("PREDLAGAM LEVO");}
			}*/
			
			msg.data="stop";
			senzorlevodesno.publish(msg);
			ros::spinOnce();
			stotinkasekundePostanek01.sleep();
		} else 
		{
			//string ukazQRkode="";
			//int naletelNaOviro=0; 
			//int naletelNaQR=0;
			ROS_INFO("Robot je zaklenjen!!!");
			ROS_INFO("QR: %d, Ovire: %d",naletelNaQR,naletelNaOviro);
			
			if(naletelNaQR==1){
				ROS_INFO("Ustavil se je, ker je naletel na QR kodo. Moral bi poiskati QR kodo in jo prebrati ter izvrsiti ukaz");
				ROS_INFO("Ukaz: %s",ukazQRkode);
				
				//-------------------------------------
				if(ukazQRkode=="LEFT"){
					ROS_INFO("Sedaj bi moral na levo. y/n");
					char c;
					cin >> c;
					if(c=='y'){rotacijaRobota(90);}
				}
				if(ukazQRkode=="RIGHT"){
					ROS_INFO("Sedaj bi moral na desno.");
					char c;
					cin >> c;
					if(c=='y'){rotacijaRobota(-90);}
				}
				if(ukazQRkode=="TURN"){
					ROS_INFO("Sedaj bi se moral obrniti.");
					char c;
					cin >> c;
					if(c=='y'){rotacijaRobota(180);}
				}
				if(ukazQRkode=="CONTINUE"){
					ROS_INFO("Sedaj bi moral naprej.");
					char c;
					cin >> c;
					if(c=='y'){}
				}
				if(ukazQRkode=="STOP"){
					ROS_INFO("Sedaj bi se moral ustaviti.");
					char c;
					cin >> c;
					if(c=='y'){exit(0);}
				}	
				 
				ukazQRkode==""; 
				zaklenjen=0;
				naletelNaQR=0;
				
				//-------------------------------------
			}
			
			
			if(naletelNaOviro==1){
				ROS_INFO("Ustavil se je, ker je naletel na oviro. Moral bi skenirati. y/n");
				
				char c;
				cin >> c;
				if(c=='y')
				{
					msg.data="start";
					skenUkaz.publish(msg);
					ROS_INFO("Skeniranje");
					potekaSken=1;
					ros::spinOnce();
				}
				
				
			}
			
		}
	}
	

	//ukaz levo
	if (c=='j')
	{
		stevec++;ROS_INFO("LEVO!!! %d", stevec);
		
		//najprej obrne senzor za 20 stopinj v desno
		roborodney::pozicijaServoMotorja sporocilo;
		sporocilo.pin=25;
		sporocilo.cas=ros::Time::now();
		sporocilo.pozicija=1593;
		posiljanjeServo.publish(sporocilo);
		ros::spinOnce();
		desetinkasekundePostanek01.sleep();
		
		ukaz.smer="levo";
		ukaz_smeri.publish(ukaz);
		ros::spinOnce();
		stotinkasekundePostanek01.sleep();
		while(trenutnaRazdalja<maxVarnostnaRazdalja && trenutnaRazdalja>minVarnostnaRazdalja){
			ukaz.smer="levo";
			ukaz.hitrost=50;
			ukaz.trajanje=20;
			ukaz_smeri.publish(ukaz);
			ros::spinOnce();
			stotinkasekundePostanek01.sleep();
		}
		for (int i=0;i<10;i++){
			//ROS_INFO("%d",i);
			ukaz.smer="levo";
			ukaz.hitrost=50;
			ukaz.trajanje=20;
			ukaz_smeri.publish(ukaz);
			ros::spinOnce();
			stotinkasekundePostanek01.sleep();
		}
		zaklenjen=0;
		
		//senzor postavim nazaj
		//servo::pozicijaServoMotorja sporocilo;
		//sporocilo.pin=25;
		sporocilo.cas=ros::Time::now();
		sporocilo.pozicija=1500;
		posiljanjeServo.publish(sporocilo);
		ros::spinOnce();
		desetinkasekundePostanek01.sleep();
	}
	
	//ukaz desno
	if (c=='l')
	{
		stevec++;ROS_INFO("DESNO!!! %d", stevec);
		
		//najprej obrne senzor za 20 stopinj v desno
		roborodney::pozicijaServoMotorja sporocilo;
		sporocilo.pin=25;
		sporocilo.cas=ros::Time::now();
		sporocilo.pozicija=1407;
		posiljanjeServo.publish(sporocilo);
		ros::spinOnce();
		desetinkasekundePostanek01.sleep();
		
		ukaz.smer="desno";
		ukaz_smeri.publish(ukaz);
		ros::spinOnce();
		stotinkasekundePostanek01.sleep();
		while(trenutnaRazdalja<maxVarnostnaRazdalja && trenutnaRazdalja>minVarnostnaRazdalja){
			ukaz.smer="desno";
			ukaz.hitrost=50;
			ukaz.trajanje=20;
			ukaz_smeri.publish(ukaz);
			ros::spinOnce();
			stotinkasekundePostanek01.sleep();
		}
		
		for (int i=0;i<10;i++){
			//ROS_INFO("%d",i);
			ukaz.smer="desno";
			ukaz.hitrost=50;
			ukaz.trajanje=20;
			ukaz_smeri.publish(ukaz);
			ros::spinOnce();
			stotinkasekundePostanek01.sleep();
		}
		zaklenjen=0;
		
		//senzor postavim nazaj
		//servo::pozicijaServoMotorja sporocilo;
		//sporocilo.pin=25;
		sporocilo.cas=ros::Time::now();
		sporocilo.pozicija=1500;
		posiljanjeServo.publish(sporocilo);
		ros::spinOnce();
		desetinkasekundePostanek01.sleep();
	}
	
	//ukaz nazaj
	if (c==',')
	{
		stevec++;ROS_INFO("NAZAJ!!! %d", stevec);
		ukaz.smer="nazaj";
		ukaz.hitrost=25;
		ukaz.trajanje=50;
		ukaz_smeri.publish(ukaz);
		zaklenjen=0;
	} 
	
	
	
	
	
	
	
	//brezpogojno naprej
	if (c=='t')
	{
		stevec++;ROS_INFO("BREZPOGOJNO NAPREJ!!! %d", stevec);
		ukaz.smer="naprej";
		ukaz.hitrost=25;
		ukaz.trajanje=50;
		ukaz_smeri.publish(ukaz);
		zaklenjen=0;
	} 
	
	//brezpogojno levo
	if (c=='f')
	{
		stevec++;ROS_INFO("BREZPOGOJNO LEVO!!! %d", stevec);
		ukaz.smer="levo";
		ukaz.hitrost=50;
		ukaz.trajanje=30;
		ukaz_smeri.publish(ukaz);
	}
	
	//brezpogojno desno
	if (c=='h')
	{
		stevec++;ROS_INFO("BREZPOGOJNO DESNO!!! %d", stevec);
		ukaz.smer="desno";
		ukaz.hitrost=50;
		ukaz.trajanje=30;
		ukaz_smeri.publish(ukaz);
	}
	
	//brezpogojno nazaj
	if (c=='b')
	{
		stevec++;ROS_INFO("BREZPOGOJNO NAZAJ!!! %d", stevec);
		ukaz.smer="nazaj";
		ukaz.hitrost=25;
		ukaz.trajanje=50;
		ukaz_smeri.publish(ukaz);
	}
	
	//brezpogojno 10 korak naprej 
	if (c=='r')
	{
		stevec++;ROS_INFO("BREZPOGOJNO 10 KORAKOV NAPREJ %d", stevec);
		ukaz.smer="naprej";
		ukaz.hitrost=25;
		ukaz.trajanje=50;
		for(int i=0;i<10;i++){
			ukaz_smeri.publish(ukaz);
			ros::spinOnce();
			dvajsetinkasekundePostanek01.sleep();
		}
		zaklenjen=0;
	}
	
	
	
	
	
	
	
	
	//sken okolice
	if(c=='s') 
	{
		msg.data="start";
		skenUkaz.publish(msg);
		ROS_INFO("Skeniranje");
		potekaSken=1;
		ros::spinOnce();
	}
	
	//preverjanje tock okolice
	if(c=='a')
	{
		int prehod=1;
		ROS_INFO("PREVERJAM TOCKE PRED SABO!!!");
		//Najprej preveri, če so sploh kake točke v pomnilniku
		if(Tocke.empty())
		{
			ROS_INFO("Ni se poskeniranih tock!");
		} else {
			ROS_INFO("Tocke so v pomnilniku, jih bom nekako preracunal.");
			ROS_INFO("Zapisanih je %d tock", Tocke.size());
			for(int i=0;i<Tocke.size();i++)
			{
				ROS_INFO("Tocka x:%f y:%f", Tocke[i].x, Tocke[i].y);
				if(Tocke[i].y<100)
				{
					ROS_INFO("Veljavna!");
					if(Tocke[i].x>-17 && Tocke[i].x<17)
					{
						prehod=0;
						ROS_INFO("Tocka na poti!!");
					}
				} else 
				{
					ROS_INFO("Neveljavna!");
				}
				ROS_INFO(" ");
			}
		}
		ROS_INFO("Prehod %d", prehod);
	}
	
	
	
	//start in stop obračanja pladnja s senzorji
	if(c=='x') {msg.data="start";senzorlevodesno.publish(msg);ROS_INFO("Startam levo desno");ros::spinOnce();}
	if(c=='c') {msg.data="stop";senzorlevodesno.publish(msg);ROS_INFO("Ustavljam levo desno");ros::spinOnce();}
	//izpis podatkov o hitrosti
	if(c=='y') {izpisPodatkov();}
	
	//rotacija levo
	if(c=='q') 
	{
		rotacijaRobota(kotRotacije);
	}
	
	//rotacija desno
	if(c=='w')
	{
		rotacijaRobota(-kotRotacije);
	}
	
	//odklenem robota
	if(c=='o')
	{
		zaklenjen=0;
		ROS_INFO("Odklenjen robot!");
	}
	
	//kotcilja=trenutniKot;
	if(c=='d')
	{
		ros::spinOnce();
		desetinkasekundePostanek01.sleep();
		desetinkasekundePostanek01.sleep();
		desetinkasekundePostanek01.sleep();
		kotcilja=trenutniKot;
		ROS_INFO("Dolocim nov ciljni kot %f", kotcilja); 
	}
	
	if(c=='p')
	{
		ROS_INFO("OK, ugasnem");
		//system("shutdown");
	}
	
	if(c=='v')
	{
		ROS_INFO("OK, restart");
		//system("reboot");
	}
	
	if(c=='e')
	{
		ros::spinOnce();
		desetinkasekundePostanek01.sleep();
		desetinkasekundePostanek01.sleep();
		desetinkasekundePostanek01.sleep();
		ROS_INFO("obrnil bi ga v pravo smer");
		ROS_INFO("Ciljni kot % f  TrenutniKot %f", kotcilja,trenutniKot);
		int novZacetni=trenutniKot;
		int novKoncni=kotcilja;
		int obrat=novKoncni-novZacetni;
		int obrat2=obrat;
		//if(trenutniKot<0){novZacetni=-trenutniKot;} else {novZacetni=trenutniKot;}
		//if(kotcilja<0){novKoncni=-kotcilja;} else {novKoncni=kotcilja;}
		//obrat=novKoncni-novZacetni;
		if(abs(obrat)>180){
			obrat2 = 360-abs(obrat);
			if(obrat>0){
				obrat2=-obrat2;
			}
		}
		ROS_INFO("Obrat za kot %d",obrat2);
		rotacijaRobota(obrat2);
	}
	
	if(c=='1') 
	{
		ROS_INFO("Detekcija QR kode");
		msg.data="qrkoda";
		ukazSkener.publish(msg);
		ros::spinOnce();
		sekundniPostanek01.sleep();
		ros::spinOnce();
		sekundniPostanek01.sleep();
		ros::spinOnce();
		sekundniPostanek01.sleep();
		ros::spinOnce();
		sekundniPostanek01.sleep();
		ros::spinOnce();
		sekundniPostanek01.sleep();
	}
	
	
	if(c=='2')
	{
		ROS_INFO("Detekcija obraza");
		msg.data="zaznavaobraza";
		ukazSkener.publish(msg);
		ros::spinOnce();
		sekundniPostanek01.sleep();
		ros::spinOnce();
		sekundniPostanek01.sleep();
		ros::spinOnce();
		sekundniPostanek01.sleep();
		ros::spinOnce();
		sekundniPostanek01.sleep();
		ros::spinOnce();
		sekundniPostanek01.sleep();
	}
	
	
	ros::spinOnce();
	stotinkasekundePostanek01.sleep();
	}
	ros::spinOnce();
	stotinkasekundePostanek01.sleep();
  }
  
  printf("CIAO!!! \n");
  return 0;
  
}
