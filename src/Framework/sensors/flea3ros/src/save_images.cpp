#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sensor_msgs/CompressedImage.h>
#include <vector>

bool cap[4] = {false,false,false,false};
ros::NodeHandle *n;
std::string name = "cam";
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat im = cv_bridge::toCvShare(msg, "bgr8")->image;
    
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv::waitKey(30);
    cv::imwrite(name+"0.png",im);
    cap[0] = true;
    if(cap[0] && cap[1]&& cap[2] && cap[3]) exit(0);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat im = cv_bridge::toCvShare(msg, "bgr8")->image;
    
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv::waitKey(30);
    cv::imwrite(name+"1.png",im);
    cap[1] = true;
    if(cap[0] && cap[1]&& cap[2] && cap[3]) exit(0);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void compressedImageCallback(const sensor_msgs::CompressedImage& msg)
{
  try
  {
    cv::Mat im = imdecode(cv::Mat(msg.data),1);
    
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv::waitKey(30);
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(1);
    cv::imwrite(name+"0.jpeg",im,compression_params);
    cap[2] = true;
    if(cap[0] && cap[1]&& cap[2] && cap[3]) exit(0);
  }
  catch (cv_bridge::Exception& e)
  {
    //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg.encoding.c_str());
  }
}
void compressedImageCallback1(const sensor_msgs::CompressedImageConstPtr& msg)
{
  try
  {
    cv::Mat im = imdecode(cv::Mat(msg->data),1);
    
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv::waitKey(30);
    cv::imwrite(name+"1.jpeg",im);
    cap[3] = true;
    if(cap[0] && cap[1] && cap[2] && cap[3]) exit(0);
  }
  catch (cv_bridge::Exception& e)
  {
    //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
int main(int argc, char **argv)
{
  if (argc == 2) name = argv[1];
  
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
image_transport::Subscriber sub = it.subscribe("/SENSORS/FLEA3/0", 1, imageCallback);
image_transport::Subscriber sub1 = it.subscribe("/SENSORS/FLEA3/1", 1, imageCallback1);
 ros::Subscriber sub3 = nh.subscribe("/SENSORS/FLEA3/0/compressed", 1, compressedImageCallback);
 ros::Subscriber sub4 = nh.subscribe("/SENSORS/FLEA3/1/compressed", 1, compressedImageCallback1);
  ros::spin();
  cv::destroyWindow("view");
}