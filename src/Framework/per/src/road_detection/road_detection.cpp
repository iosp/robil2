#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/CompressedImage.h>
// #include <sensor_msgs/Image.h>
#include "std_msgs/Float64MultiArray.h"
#include "per/roadLanes.h"
#include "per/lane.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "displayImage/DisplayImage.h"
#include "displayImage/polyfit.h"
#define _USE_MATH_DEFINES
#include <cmath>

using namespace cv;
using namespace std;

/** *
 * 		displayImage:
 ** */
ros::NodeHandle *n;
ros::Publisher chatter_pub ;

int toDebug;

void displayImage(const sensor_msgs::CompressedImage& msg)
// void displayImage(const sensor_msgs::Image& msg)
{
  Mat m = imdecode(Mat(msg.data),1);
//   Mat m = Mat(msg.step/3, msg.height);
  
  //imshow("blat", m);
  //waitKey(0);
  std::vector<double> lanes;
  
  lanes = detectRoad(m, 50, 100, toDebug);
 
  per::roadLanes retMsg;
  
  retMsg.header.stamp = ros::Time::now();
  retMsg.header.frame_id = "Road Detection\n";
  
  int size = lanes.size()/5;
  retMsg.lanes.resize(size);
  
  for(int i=0; i<size; i++)
  {
    retMsg.lanes[i].highPix 	= lanes[i*5 + 0];
    retMsg.lanes[i].lowPix 	= lanes[i*5 + 1];
    retMsg.lanes[i].x2 	= lanes[i*5 + 2];
    retMsg.lanes[i].x1 	= lanes[i*5 + 3];
    retMsg.lanes[i].x0 	= lanes[i*5 + 4];
  }
  
  chatter_pub.publish(retMsg);
}

/** *
 * 		chatterCallback:
 ** */
  
int counter = 0;
int everyNthTime = 5;//should be : 2;

void chatterCallback(const sensor_msgs::CompressedImage& msg)
{
  if(counter%everyNthTime == 0)
  {
    displayImage(msg);
    counter = 1;
  }
  else
  {
    counter ++;
  }
    
}


/** *
 * 		main:
 ** */

int main(int argc, char **argv)
{
  if(argc > 1)
    if(strcmp(argv[1], "-d"))
      toDebug = 0;
    else
      toDebug =1; 
    
  ros::init(argc, argv, "listener");
  ros::NodeHandle n1;
  n = &n1;
  ros::Subscriber sub = n->subscribe("SENSORS/CAM/R/compressed", 1000, chatterCallback);
  chatter_pub = n->advertise<per::roadLanes>("RoadLanes",1000);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  
  return 0;
}
