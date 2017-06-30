
#include <stdlib.h>
#include <bvt_sdk.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"

int pos = 0;
int intensity = 0;
double int2double = 0;

int thres = 0 ;
double maxval = 255 ;
int thres_type = CV_THRESH_BINARY ;

BVTSDK::ImageGenerator img;
BVTSDK::ColorMapper map;
cv::Mat color_img ;

std::string GetMatType(const cv::Mat& mat)
{
    const int mtype = mat.type();

    switch (mtype)
    {
    case CV_8UC1:  
		printf("8UC1\n");
		return "CV_8UC1";
    case CV_8UC2:  return "CV_8UC2";
    case CV_8UC3:  return "CV_8UC3";
    case CV_8UC4:  return "CV_8UC4";

    case CV_8SC1:  return "CV_8SC1";
    case CV_8SC2:  return "CV_8SC2";
    case CV_8SC3:  return "CV_8SC3";
    case CV_8SC4:  return "CV_8SC4";

    case CV_16UC1: return "CV_16UC1";
    case CV_16UC2: return "CV_16UC2";
    case CV_16UC3: return "CV_16UC3";
    case CV_16UC4: return "CV_16UC4";

    case CV_16SC1: return "CV_16SC1";
    case CV_16SC2: return "CV_16SC2";
    case CV_16SC3: return "CV_16SC3";
    case CV_16SC4: return "CV_16SC4";

    case CV_32SC1: return "CV_32SC1";
    case CV_32SC2: return "CV_32SC2";
    case CV_32SC3: return "CV_32SC3";
    case CV_32SC4: return "CV_32SC4";

    case CV_32FC1: return "CV_32FC1";
    case CV_32FC2: return "CV_32FC2";
    case CV_32FC3: return "CV_32FC3";
    case CV_32FC4: return "CV_32FC4";

    case CV_64FC1: return "CV_64FC1";
    case CV_64FC2: return "CV_64FC2";
    case CV_64FC3: return "CV_64FC3";
    case CV_64FC4: return "CV_64FC4";

    default:
        return "Invalid type of matrix!";
    }
}


void SpeedChange( int , void*){
	img.SetSoundSpeedOverride(pos);
}
void GammaChange( int , void*){
	int2double = intensity/100 ;
	map.SetGamma(int2double);
}
void ThresholdChange( int , void*){
	//cvtColor(color_img, color_img, cv::COLOR_RGB2GRAY);
	threshold(color_img, color_img,thres, maxval, cv::THRESH_BINARY );

}

int main(int argc, char** argv){

	std::string rootPath = "/home/paicaloid/bvtsdk/";
	std::string dataPath = rootPath + "data/";
	std::string mapperPath = rootPath + "colormaps/bone.cmap";
	std::string fileName = "Mar_01_2017_115453.son";
	std::string fullPath = dataPath + fileName;
	pos = 1500;
	intensity = 30;
	thres = 70 ;
	cv::namedWindow("SoundSpeedTesting" , cv::WINDOW_NORMAL);
	cv::createTrackbar("Sound Speed" , "SoundSpeedTesting" , &pos , 2000 , SpeedChange);
	//cv::createTrackbar("Gamma" , "SoundSpeedTesting" , &intensity , 100 , GammaChange);
	cv::createTrackbar("Threshold" , "SoundSpeedTesting" , &thres , 255 , ThresholdChange);
	
	BVTSDK::Sonar sonar;
	sonar.Open("FILE" , fullPath);

	BVTSDK::Head head = sonar.GetHead(0);
	BVTSDK::Ping ping = head.GetPing(0);
	
	int i = head.GetPingCount();

	img.SetHead(head);
	//BVTSDK::ColorMapper map;
	//map.Load(mapperPath);

	int p = 1;
	int k = 0;

	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("imaging_sonar", 1);
	

	ros::Rate loop_rate(5);

	while (nh.ok()) {
		map.Load(mapperPath);
		ping = head.GetPing(-1);
		BVTSDK::MagImage mag = img.GetImageXY(ping);
		BVTSDK::ColorImage cimg = map.MapImage(mag);
		
		int height = cimg.GetHeight();
		int width = cimg.GetWidth();

		cv::Mat color_img(height , width , CV_8UC4 , cimg.GetBits());
		// Pause by pressing 'P'
	 	while(!p){
	 		k = cv::waitKey(30);
	    	if (k == 112){
	    		p = !p;
	    	}
		}
		//double thres = 70 ;
		//double maxval = 255 ;
		//int thres_type = CV_THRESH_BINARY ;
		cvtColor(color_img, color_img, cv::COLOR_RGB2GRAY);
		threshold(color_img, color_img,thres, maxval, cv::THRESH_BINARY );
		
		
		
		cv::imshow("SoundSpeedTesting", color_img);
        k = cv::waitKey(30);
    	if( k == 27 )
    		break;
    	else if (k == 112){
    		p = !p;
    	}
    	i--;

		//sensor_msgs::CompressedImage msg_cpr ;
		//msg_cpr.format = "jpeg" ;
		//msg.data = color_img ;

		GetMatType(color_img) ;


		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", color_img).toImageMsg();
		pub.publish(msg);
		//pub.publish(msg_cpr) ;
		ros::spinOnce();
		loop_rate.sleep();

	}
}

