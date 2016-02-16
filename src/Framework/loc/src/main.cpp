#include <ros/ros.h>
#include "component/ComponentMain.h"
#include <ros/spinner.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Float64.h>
ComponentMain* cptr;

void throttle_callback(const std_msgs::Float64& msg){
  cptr->setThrottleInput(msg.data);
}

void steering_callback(const std_msgs::Float64& msg){
  cptr->setSteeringInput(msg.data);
}

int main(int argc,char** argv)
{
  ComponentMain comp(argc,argv);
  cptr = &comp;
  ros::NodeHandle n;
  
  ros::Subscriber vis = n.subscribe("/LLC/EFFORTS/Throttle", 5, throttle_callback);
  ros::Subscriber wlrs = n.subscribe("/LLC/EFFORTS/Steering", 10, steering_callback);
  dynamic_reconfigure::Server<loc::configConfig> server;
  dynamic_reconfigure::Server<loc::configConfig>::CallbackType cb;
  cb = boost::bind(&ComponentMain::configCallback, &comp,  _1, _2);
  server.setCallback(cb);
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
