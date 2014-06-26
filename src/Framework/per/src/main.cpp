#include <ros/ros.h>
#include "component/ComponentMain.h"
#include "component/ComponentStates.h"
#include <ros/spinner.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Char.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "component/rdbg.h"

using namespace cv;


ComponentMain* cptr;

void setVisual(const std_msgs::Char::ConstPtr& msg)
{
   cptr->setVisualize(msg->data);
}

int main(int argc,char** argv)
{
  ComponentMain comp(argc,argv);
  cptr = &comp;
  ros::NodeHandle n;
  ros::Subscriber vis = n.subscribe("/PER/VISUAL", 5, setVisual);
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  runComponent(argc,argv, comp);
  ros::waitForShutdown();
  return 0;
}
