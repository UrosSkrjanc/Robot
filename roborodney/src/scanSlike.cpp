#include <ros/ros.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <iostream>
#include <stdio.h>
#include "roborodney/pozicijaObraza.h"
#include "zbar.h"

using namespace std;
using namespace cv;

string face_cascade_name = "/root/opencv/data/haarcascades/haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;
std::string arg1;
std::string arg2;

zbar::ImageScanner scanner;

ros::Publisher publisher_pozicijaObraza;
ros::Publisher barcode_pub; 


// Function Headers
void detectAndDisplay(Mat frame, Point* p1, Point* p2);

void zaznavanjeObrazov() {
	ROS_INFO("Znotraj funkcije zaznave obrazov");
	
	ros::Rate sekundniPostanek(1);
	roborodney::pozicijaObraza pozicijaObr;
		
	std::istringstream video_sourceCmd("0");
	int video_source;
	if(!(video_sourceCmd >> video_source)) ROS_INFO("Nekaj s kamero ni OK 1");
	cv::VideoCapture cap(video_source);
	if(!cap.isOpened()) ROS_INFO("Nekaj s kamero ni OK 2");
	cv::Mat dImg;
	cap >> dImg;
	cv::flip(dImg,dImg,-1);
	Point pt1; 
	Point pt2; 
	detectAndDisplay(dImg,&pt1,&pt2);
	Point pt3((pt1.x+pt2.x)/2,(pt1.y+pt2.y)/2);
	if(pt2.x > 0)
	{
		pozicijaObr.cas = ros::Time::now();
		pozicijaObr.p1.x = pt1.x;
		pozicijaObr.p1.y = pt1.y;
		pozicijaObr.p2.x = pt2.x;
		pozicijaObr.p2.y = pt2.y;
		pozicijaObr.res.x = dImg.cols;
		pozicijaObr.res.y = dImg.rows;
	} else {
		pozicijaObr.cas = ros::Time::now();
		pozicijaObr.p1.x = 0;
		pozicijaObr.p1.y = 0;
		pozicijaObr.p2.x = 0;
		pozicijaObr.p2.y = 0;
		pozicijaObr.res.x = dImg.cols;
		pozicijaObr.res.y = dImg.rows;
	}
	publisher_pozicijaObraza.publish(pozicijaObr);
	ros::spinOnce();  
	sekundniPostanek.sleep();
	ros::spinOnce();
}

void zaznavanjeQR() {
	ROS_INFO("Znotraj funkcije zaznave qr");
	ros::Rate sekundniPostanek(1);

	std::istringstream video_sourceCmd("0");
	int video_source;
	if(!(video_sourceCmd >> video_source)) ROS_INFO("Nekaj s kamero ni OK 1");
	cv::VideoCapture cap(video_source);
	if(!cap.isOpened()) ROS_INFO("Nekaj s kamero ni OK 2");
	cv::Mat frame;
	sensor_msgs::ImagePtr msg;
	cap >> frame;
	if(!frame.empty())
	{
		cv::flip(frame,frame,-1);
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		cv_bridge::CvImageConstPtr cv_image;
		cv_image = cv_bridge::toCvShare(msg, "mono8");
		zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data,cv_image->image.cols * cv_image->image.rows);
		scanner.scan(zbar_image);
		int nasel=0;
		for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
			{
				std::string barcode = symbol->get_data();
				std_msgs::String barcode_string;
				barcode_string.data = barcode;
				barcode_pub.publish(barcode_string);
				nasel=1;
			}
			
		/*
		//ce ni nasel qr kode
		if(nasel==0)
		{
			std::string barcode = "NULL";
			std_msgs::String barcode_string;
			barcode_string.data = barcode;
			barcode_pub.publish(barcode_string);
		}
		*/
	}
	ros::spinOnce();  
	sekundniPostanek.sleep();
	ros::spinOnce();  
}


void strsken(const std_msgs::String& msg)
{
	if(msg.data=="zaznavaobraza"){
		ROS_INFO("Zaznaval bom obraze");
		zaznavanjeObrazov();
	}
	
	if(msg.data=="qrkoda"){
		ROS_INFO("Zaznaval bom qr kodo");
		zaznavanjeQR();
	}
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "skener");
    ros::NodeHandle nh;

	ros::Subscriber ukaz = nh.subscribe("ukazskener", 1000, strsken);
    publisher_pozicijaObraza = nh.advertise<roborodney::pozicijaObraza>("pozicija_obraza",1000);
	barcode_pub = nh.advertise<std_msgs::String>("barcode", 10);
    face_cascade.load(face_cascade_name);
    ros::spin();
}


//---------------------------------------
// Function detectAndDisplay
void detectAndDisplay(Mat frame, Point* p1, Point* p2)
{
    std::vector<Rect> faces;
    Mat frame_gray;
    Mat crop;
    Mat res;
    Mat gray;
	
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    // Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

    // Set Region of Interest
    cv::Rect roi_b;
    cv::Rect roi_c;

    size_t ic = 0; // ic is index of current element
    int ac = 0; // ac is area of current element

    size_t ib = 0; // ib is index of biggest element
    int ab = 0; // ab is area of biggest element    //waitKey(0);

    for (ic = 0; ic < faces.size(); ic++) // Iterate through all current elements (detected faces)
    {
        roi_c.x = faces[ic].x;
        roi_c.y = faces[ic].y;
        roi_c.width = (faces[ic].width);
        roi_c.height = (faces[ic].height);

        ac = roi_c.width * roi_c.height; // Get the area of current element (detected face)

        roi_b.x = faces[ib].x;
        roi_b.y = faces[ib].y;
        roi_b.width = (faces[ib].width);
        roi_b.height = (faces[ib].height);

        ab = roi_b.width * roi_b.height; // Get the area of biggest element, at beginning it is same as "current" element

        if (ac > ab)
        {
            ib = ic;
            roi_b.x = faces[ib].x;
            roi_b.y = faces[ib].y;
            roi_b.width = (faces[ib].width);
            roi_b.height = (faces[ib].height);
        }

        crop = frame(roi_b);
        resize(crop, res, Size(128, 128), 0, 0, INTER_LINEAR); // This will be needed later while saving images
        cvtColor(crop, gray, CV_BGR2GRAY); // Convert cropped image to Grayscale

        Point pt1(faces[ic].x, faces[ic].y); // Display detected faces on main window - live stream from camera
        Point pt2((faces[ic].x + faces[ic].height), (faces[ic].y + faces[ic].width));
		
        *p1 = pt1;
        *p2 = pt2;
    } 
}
