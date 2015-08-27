#include <ros/ros.h>
#include "component/ComponentMain.h"
#include "component/ComponentStates.h"
#include <ros/spinner.h>
#include <boost/thread/thread.hpp>

int main(int argc,char** argv)
{

    ComponentMain comp(argc,argv);

    //ros::NodeHandle n;

    ros::AsyncSpinner spinner(4); // Use 4 threads

    spinner.start();
    //ros::waitForShutdown();

    //cptr->activateThread();

    runComponent(argc,argv, comp);

    return 0;
}
