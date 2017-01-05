#include "localization.h"



int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization");
    ros::NodeHandle nh;
    localization loc(nh);
    /// Setting up the dynamic reconfigure server ///
    dynamic_reconfigure::Server<loc::configConfig> server;
    dynamic_reconfigure::Server<loc::configConfig>::CallbackType cb;
    cb = boost::bind(&localization::configCallback, &loc,  _1, _2);
    server.setCallback(cb);
    /// Setting up synchronized subscribers ///
    message_filters::Subscriber<Imu> imu_sub(nh, "/SENSORS/INS", 1);
    message_filters::Subscriber<NavSatFix> gps_sub(nh, "/SENSORS/GPS", 1);
    message_filters::Subscriber<NavSatFix> gps_speed_sub(nh, "/SENSORS/GPS/Speed", 1);
    TimeSynchronizer<Imu, NavSatFix, NavSatFix> sync(imu_sub, gps_sub, gps_speed_sub, 10);
    sync.registerCallback(boost::bind(&localization::callback, &loc, _1, _2, _3));
    ros::spin();

    return 0;
}
//#include <ros/ros.h>
//#include "component/ComponentMain.h"
//#include <ros/spinner.h>
//#include <boost/thread/thread.hpp>
//#include <std_msgs/Float64.h>
//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/NavSatFix.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

//using namespace sensor_msgs;
//int main(int argc,char** argv)
//{
//    ComponentMain comp(argc,argv);
//    message_filters::Subscriber<Imu> imu_sub(comp._nh, "/SENSORS/INS", 1);
//    message_filters::Subscriber<NavSatFix> gps_sub(comp._nh, "/SENSORS/GPS", 1);
//    message_filters::Subscriber<NavSatFix> gps_speed_sub(comp._nh, "/SENSORS/GPS/Speed", 1);
//    message_filters::TimeSynchronizer<Imu, NavSatFix, NavSatFix> sync(imu_sub, gps_sub, gps_speed_sub, 10);
//    sync.registerCallback(boost::bind(&ComponentMain::callback, &comp, _1, _2, _3));
//    dynamic_reconfigure::Server<loc::configConfig> server;
//    dynamic_reconfigure::Server<loc::configConfig>::CallbackType cb;
//    cb = boost::bind(&ComponentMain::configCallback, &comp,  _1, _2);
//    server.setCallback(cb);
//    ros::AsyncSpinner spinner(4); // Use 4 threads
//    spinner.start();
//    ros::waitForShutdown();
//    return 0;
//}
