#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>
const double PI  = 3.141592653589793238463;
using namespace std;
ros::NodeHandle *n;
cv::VideoWriter outputVideo,outputVideo1;
bool cap1=false,cap0=false;
int cntr = 0;
sensor_msgs::NavSatFix gps_init;
double x,y;
FILE *f;


inline double calcDistance(sensor_msgs::NavSatFix p1,sensor_msgs::NavSatFix p2)
{
	/**
	 * calc_distance calculates the distance between two latitude/longitude points.
	 * The equations are taken from the following site:
	 * http://www.movable-type.co.uk/scripts/latlong.html
	 */
	double R = 6371.0; //[km]
	double lat1 = p1.latitude*PI/180.0;
	double lat2 = p2.latitude*PI/180.0;
	double dLat = lat2 - lat1;
	double dLon = (p2.longitude - p1.longitude)*PI/180.0;

	double a = sin(dLat/2) * sin(dLat/2) +
			sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	double d = R*c;

	return d*1000; //return distance in [m]
}
inline double calcBearing(sensor_msgs::NavSatFix p1,sensor_msgs::NavSatFix p2)
{
	/**
	 * calc the bearing from my initial position
	 */
	double lat1 = p1.latitude*PI/180;
	double lat2 = p2.latitude*PI/180;
	double dLat = lat2 - lat1;
	double dLon = (p2.longitude - p1.longitude)*PI/180;
	double y = sin(dLon) * cos(lat2);
	double x = cos(lat1) * sin(lat2) -
			sin(lat1) * cos(lat2) * cos(dLon);
	double brng = atan2(y, x);

	return brng;
}

void compressedImageCallback(const sensor_msgs::CompressedImage& msg)
{
 // std::cout << msg.format << std::endl;
    cv::Mat im = imdecode(cv::Mat(msg.data),1);
    
//     outputVideo << im;
//     cv::imshow("view", im);
//     cv::waitKey(30);
    char name[20]={0};
    sprintf(name,"%04dl.jpeg",cntr);
    cv::imwrite(name,im);
    if(cap0)
    {
      cntr++;
      cap1 = false;
      cap0 = false;
    }
    else
      cap1 = true;
}
void compressedImageCallback1(const sensor_msgs::CompressedImage& msg)
{
 // std::cout << msg.format << std::endl;
    cv::Mat im = imdecode(cv::Mat(msg.data),1);
        char name[20]={0};
    sprintf(name,"%04dr.jpeg",cntr);
    cv::imwrite(name,im);
    if(cap1)
    {
      cntr++;
      cap1 = false;
      cap0 = false;
    }
    else
      cap0 = true;
//     outputVideo1 << im;
//     cv::imshow("view2", im);
//     cv::waitKey(30);
    
}

void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat im = cv_bridge::toCvShare(msg, "bgr8")->image;
    char name[20]={0};
    sprintf(name,"%04dr.png",cntr);
    cv::imwrite(name,im);
    if(cap1)
    {
      cntr++;
      cap1 = false;
      cap0 = false;
    }
    else
      cap0 = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void ImageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat im = cv_bridge::toCvShare(msg, "bgr8")->image;
    char name[20]={0};
    sprintf(name,"%04dl.png",cntr);
    cv::imwrite(name,im);
    if(cap0)
    {
      cntr++;
      cap1 = false;
      cap0 = false;
    }
    else
      cap1 = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }


//     outputVideo1 << im;
//     cv::imshow("view2", im);
//     cv::waitKey(30);
//     f = fopen("xy_dat.txt","a");
//     fprintf(f,"%f,%f\n",x,y);
//     fclose(f);
    
}
void GPSCallback1(const sensor_msgs::NavSatFix mes)
{
  if (gps_init.altitude == 0) gps_init = mes;
  double d = calcDistance(mes, gps_init);
  double t = calcBearing(gps_init,mes);
  x = d * cos(t);
  y = d * sin(t);
  
  cout << x << ", " << y << endl;
  
}
int main(int argc, char **argv)
{
  remove("xy_dat.txt");
  outputVideo.open("Right.mpg",CV_FOURCC('P','I','M','1'),28,cv::Size(1288,964));
  outputVideo1.open("Left.mpg",CV_FOURCC('P','I','M','1'),28,cv::Size(1288,964));
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //cv::namedWindow("view");cv::namedWindow("view2");
  //cv::startWindowThread();
 ros::Subscriber sub3 = nh.subscribe("/SENSORS/FLEA3/0/compressed", 1, compressedImageCallback);
ros::Subscriber sub4 = nh.subscribe("/SENSORS/FLEA3/1/compressed", 1, compressedImageCallback1);

//  ros::Subscriber sub3 = nh.subscribe("/SENSORS/FLEA3/0", 1, ImageCallback);
// ros::Subscriber sub4 = nh.subscribe("/SENSORS/FLEA3/1", 1, ImageCallback1);
ros::Subscriber sub5 = nh.subscribe("/SENSORS/GPS", 1, GPSCallback1);
  ros::spin();
  cout << "here";
  cv::destroyWindow("view");
}