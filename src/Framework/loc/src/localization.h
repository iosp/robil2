#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <geometry_msgs/TwistStamped.h>

#include "component/gps_calculator.h"
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include "loc/configConfig.h"
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace sensor_msgs;
using namespace message_filters;


class localization{
public:
    localization(ros::NodeHandle& n);

    ~localization();
    /*
     * This function actually publishes the message and
     * creates a tf tranform between WORLD and ODOM
     * inputs: IMU, GPS, and GPS speed
     * This function calls the publish LOC functions
     */
    void callback(const ImuConstPtr& imu,
                  const NavSatFixConstPtr& gps,
                  const NavSatFixConstPtr& speed_msg);
    /*
     * Publishes the LOC pose and speed messages.
     * Also broadcasts the TF
     **/
    void publishersBroadcasters();
    /*
     * Dynamic reconfiguration callback
     **/
    void configCallback(loc::configConfig &config, uint32_t level);
    /*
     * Set data of speed from messages
     **/
    void handleSpeed(const NavSatFixConstPtr& speed_msg, const ImuConstPtr& imu);
    /*
     * Set data of pose from messages
     **/
    void handlePose(const NavSatFixConstPtr& gps, const ImuConstPtr& imu);

private:
    /*
     * Transform quaternion message into RPY Vector3 message
     */
    geometry_msgs::Vector3 quaternion_to_rpy(geometry_msgs::Quaternion q);

    /*
     *
     * Varibales
     *
     */
    ros::Publisher loc_pub;
    ros::Publisher speed_pub;
    geometry_msgs::TwistStamped speed;
    geometry_msgs::PoseWithCovarianceStamped pose;
    sensor_msgs::NavSatFix initialGPS;
    loc::configConfig dyn_conf;
//    ros::Publisher odom_pub;
};

#endif // LOCALIZATION_H
