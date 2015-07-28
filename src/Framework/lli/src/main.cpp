#include <ros/ros.h>
#include "component/ComponentMain.h"
#include "component/ComponentStates.h"
#include <ros/spinner.h>
#include <boost/thread/thread.hpp>
int main(int argc,char** argv)
{
    printf ("before ComponentMain\n");
    ComponentMain comp(argc,argv);
    printf ("before spinner\n");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    printf ("before spinner.start\n");
    spinner.start();
    //ros::waitForShutdown();
    printf ("before runComponent\n");
    runComponent(argc,argv, comp);
    printf ("after runComponent\n");
    return 0;
}
