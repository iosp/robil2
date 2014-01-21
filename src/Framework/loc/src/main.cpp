
#include <ros/ros.h>
#include "component/ComponentMain.h"
#include <ros/spinner.h>
#include <boost/thread/thread.hpp>
int main(int argc,char** argv)
{
  ComponentMain *comp=new ComponentMain(argc,argv);
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
