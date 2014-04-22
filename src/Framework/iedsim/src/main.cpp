#include <ros/ros.h>
#include "component/ComponentMain.h"
#include "component/ComponentStates.h"
#include <ros/spinner.h>
#include <boost/thread/thread.hpp>
int main(int argc,char** argv)
{
    ComponentMain comp(argc,argv);
    ros::AsyncSpinner spinner(2); // Use 1 thread
    spinner.start();
    //ros::waitForShutdown();
    runComponent(argc,argv, comp);
    return 0;
}
