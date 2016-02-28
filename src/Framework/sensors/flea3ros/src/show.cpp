#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>
using namespace std;using namespace cv;
ros::NodeHandle *n;

Mat full_im = Mat(480,1280,CV_8UC3);
Mat left_im(full_im,Rect(0,0,640,480));
Mat right_im(full_im,Rect(640,0,640,480));

void compressedImageCallback(const sensor_msgs::CompressedImage& msg)
{
    Mat im = imdecode(Mat(msg.data),1);resize(im,im,Size(640,480),0,0,INTER_LINEAR);
        im.copyTo(right_im);

//     imshow("right", im);
	imshow("flea",full_im);
    waitKey(1);
}
void compressedImageCallback1(const sensor_msgs::CompressedImage& msg)
{
    Mat im = imdecode(Mat(msg.data),1);resize(im,im,Size(640,480),0,0,INTER_LINEAR);
    im.copyTo(left_im);
//     imshow("left", im);
    imshow("flea",full_im);
    waitKey(1);

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

 ros::Subscriber sub3 = nh.subscribe("/SENSORS/FLEA3/0/compressed", 1, compressedImageCallback);
ros::Subscriber sub4 = nh.subscribe("/SENSORS/FLEA3/1/compressed", 1, compressedImageCallback1);
imshow("flea",full_im);
    waitKey(1);

  ros::spin();

  destroyWindow("view");
}