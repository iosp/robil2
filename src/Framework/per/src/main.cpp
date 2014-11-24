#include <ros/ros.h>
#include "component/ComponentMain.h"
#include "component/ComponentStates.h"
#include <ros/spinner.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Char.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "component/rdbg.h"
#include <road_detection/roadLanes.h>
#include <road_detection/lane.h>

#include <vector>


using namespace cv;
using namespace road_detection;

ComponentMain* cptr;
vector<lane> emptyLanes;
bool flag = false;

void setVisual(const std_msgs::Char::ConstPtr& msg)
{
   cptr->setVisualize(msg->data);
}

void handleWalrusData(const road_detection::roadLanes& msg){
  
  if(flag /* some contition to chekc flag */)
  {
   cptr->setLanes(emptyLanes);
  }
  else{
    cptr->setLanes(msg.lanes);
  }

}

int main(int argc,char** argv)
{
  ComponentMain comp(argc,argv);
  cptr = &comp;
  ros::NodeHandle n;
  
  ros::Subscriber vis = n.subscribe("/PER/VISUAL", 5, setVisual);
  ros::Subscriber wlrs = n.subscribe("RoadLanes", 10, handleWalrusData);
  
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  runComponent(argc,argv, comp);
  ros::waitForShutdown();
  return 0;
}
