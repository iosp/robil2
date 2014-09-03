#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

ros::Publisher pub;

void my_sum(vector<float> hist,int height, int width ,float *mean, float *var)
{
  /**
   * Calculate the mean and the variance of the histogram
   */
  double v = 0;
  double e = 0;
  for(int i=0;i<256;i++)
  {
    double p = hist[i] / height / width;
    e += p * i;
    v += p * i * i;
  }
  (*mean) = e;
  (*var) = sqrt((v - e*e));
}

void cam_callback(const sensor_msgs::Image::ConstPtr& data)
{
  ///transform ROS image to CV image
  try
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
    ///Transform to gray scale
    Mat gray;
    cvtColor(cv_ptr->image, gray, CV_RGB2GRAY);
    ///Calc histogram
    int hbins = 255;
    int histSize[] = {hbins};
    
    float sranges[] = { 0, 256 };
    const float* ranges[] = { sranges };
    MatND hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0};
    
    calcHist( &gray, 1, channels, Mat(), // do not use mask
	      hist, 1, histSize, ranges,
	      true, // the histogram is uniform
	      false );
    
    ///Load message with data
    sensor_msgs::LaserScan msg;
    msg.header = data->header;
    if (data->header.frame_id == "cameraL_frame") msg.header.frame_id = "cameraL_monitor";
    else msg.header.frame_id = "cameraR_monitor";
    msg.ranges = hist;
    msg.angle_min = data->width;
    msg.angle_max = data->height;
    msg.angle_increment = 256;
    my_sum(msg.ranges,data->height,data->width,&msg.range_min,&msg.range_max);
    ///Make sure that number of counts in histogram equal number of pixels
    int cnt = 0;
    for (int i=0;i<256;i++) cnt+=msg.ranges[i];
    if (!(cnt == data->width * data->height)) printf("Warning number of counts in histogram does not equal number of pixels\n");
    ///Publish message
    pub.publish(msg);
    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_monitor");  
  ros::NodeHandle n;  
  ros::Subscriber l_image = n.subscribe("/SENSORS/CAM/L", 1, cam_callback);
  ros::Subscriber r_image = n.subscribe("/SENSORS/CAM/R", 1, cam_callback);
  pub = n.advertise<sensor_msgs::LaserScan>("/camera_monitor", 1000);
  
  
  
  cout << "===============================================" << endl;
  cout << "Starting to monitor camera\n" << endl << "Please subscribe to the /camera_monitor topic\nThe data in this topic is arranged as follows:\n" << endl;
  cout << "Data           |   Field" << endl;
  cout << "--------------------------------" << endl;
  cout << "Histogram      |  ranges"<< endl;
  cout << "Variance       |  range_max"<< endl;
  cout << "mean           |  range_min"<< endl;
  cout << "Camera(L or R) |  header->frame_id"<< endl;
  cout << "image width    |  angle_min"<< endl;
  cout << "image length   |  angle_max"<< endl;
  cout << "===============================================" << endl;
  cout << "Read the README file for more information" << endl;
  ros::spin();
  return 0;
}
