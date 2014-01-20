
#include <ros/ros.h>
#include "ComponentMain.h"
#include <ros/spinner.h>
#include <boost/thread/thread.hpp>
int main(int argc,char** argv)
{
  ComponentMain comp(argc,argv);
  ros::spin();

  return 0;
}
