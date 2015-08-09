#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sensor_msgs/CompressedImage.h>
ros::NodeHandle *n;
cv::VideoWriter outputVideo,outputVideo1;
void compressedImageCallback(const sensor_msgs::CompressedImage& msg)
{
 // std::cout << msg.format << std::endl;
    cv::Mat im = imdecode(cv::Mat(msg.data),1);
    
    outputVideo << im;
    cv::imshow("view", im);
    cv::waitKey(30);
    
}
void compressedImageCallback1(const sensor_msgs::CompressedImage& msg)
{
 // std::cout << msg.format << std::endl;
    cv::Mat im = imdecode(cv::Mat(msg.data),1);
    
    outputVideo1 << im;
    cv::imshow("view2", im);
    cv::waitKey(30);
    
}
int main(int argc, char **argv)
{
  outputVideo.open("Right.mpg",CV_FOURCC('P','I','M','1'),30,cv::Size(1288,964));
  outputVideo1.open("Left.mpg",CV_FOURCC('P','I','M','1'),30,cv::Size(1288,964));
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");cv::namedWindow("view2");
  cv::startWindowThread();
 ros::Subscriber sub3 = nh.subscribe("/SENSORS/FLEA3/0/compressed", 1, compressedImageCallback);
ros::Subscriber sub4 = nh.subscribe("/SENSORS/FLEA3/1/compressed", 1, compressedImageCallback1);
  ros::spin();
  cv::destroyWindow("view");
}