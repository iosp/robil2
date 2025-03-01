
/*
 * ComponentMain.cpp
 *
 *  Created on: Thursday, 27. February 2014 12:29PM
 *      Author: autogenerated
 */
#include "ComponentMain.h"
#include "boost/bind.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "userHeader.h"
#include "gps_calculator.h"


ComponentMain::ComponentMain(int argc,char** argv)	: _inited(init(argc, argv))
{
	_sub_PositionUpdate=ros::Subscriber(_nh.subscribe("/OCU/PositionUpdate", 10, &ComponentMain::handlePositionUpdate,this));
	_pub_Location=ros::Publisher(_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/LOC/Pose",10));
	_pub_PerVelocity=ros::Publisher(_nh.advertise<geometry_msgs::TwistStamped>("/LOC/Velocity",10));

    ros::param::param("/LOC/Ready", 1);
}
ComponentMain::~ComponentMain()
{
    ros::param::param("/LOC/Ready", 0);
}
void ComponentMain::callback(const ImuConstPtr& imu, const NavSatFixConstPtr& gps, const NavSatFixConstPtr& speed_msg)
{
    geometry_msgs::TwistStamped speed;
    geometry_msgs::PoseWithCovarianceStamped pose;
    speed.header.frame_id = "ODOM";
    speed.header.stamp = ros::Time::now();
    pose.header = speed.header;
    if (initialGPS.latitude == 0 || initialGPS.longitude == 0 || dyn_conf.init)
    {
        ROS_INFO("LOC: Setting a new init position\n");
        initialGPS = *gps;
    }
    /// Setting speed message. linear.z is the global speed
    speed.twist.linear.x = speed_msg->latitude;
    speed.twist.linear.y = speed_msg->longitude;
    speed.twist.linear.z = sqrt(speed.twist.linear.x * speed.twist.linear.x +
                                speed.twist.linear.y * speed.twist.linear.y);
    speed.twist.angular.x = imu->angular_velocity.x;
    speed.twist.angular.y = imu->angular_velocity.y;
    speed.twist.angular.z = imu->angular_velocity.z;
    speed.header.stamp = imu->header.stamp;
    /// Handle GPS message
    double d = calcDistance(*gps,initialGPS);
    double theta = -calcBearing(initialGPS,*gps);
    double x = d * cos(theta);
    double y = d * sin(theta);
    pose.pose.pose.position.x = x;
    if (dyn_conf.right_hand_axes)
        pose.pose.pose.position.y = -y;
    else
        pose.pose.pose.position.y = y;
    if (dyn_conf.height)
        pose.pose.pose.position.z = gps->altitude;
    pose.pose.pose.orientation = imu->orientation;
    this->publishPerVelocity(speed);
    this->publishLocation(pose);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.pose.pose.position.x,
                                    pose.pose.pose.position.y,
                                    pose.pose.pose.position.z));
    transform.setRotation(tf::Quaternion(pose.pose.pose.orientation.x,
                                         pose.pose.pose.orientation.y,
                                         pose.pose.pose.orientation.z,
                                         pose.pose.pose.orientation.w));
    this->publishTransform(transform, "WORLD", "ODOM");
    //        /// publish the data ///
}

bool ComponentMain::init(int argc,char** argv){
	ros::init(argc,argv,"LOC_node");
	return true;
}


void ComponentMain::handlePositionUpdate(const geometry_msgs::PoseStamped& msg)
{
}

void ComponentMain::publishLocation(geometry_msgs::PoseWithCovarianceStamped& msg)
{
	_pub_Location.publish(msg);
}
	

void ComponentMain::publishPerVelocity(geometry_msgs::TwistStamped& msg)
{
	_pub_PerVelocity.publish(msg);
}
	
void ComponentMain::publishTransform(const tf::Transform& _tf, std::string srcFrame, std::string distFrame){
	static tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(_tf, ros::Time::now(), srcFrame, distFrame));
}

tf::StampedTransform ComponentMain::getLastTransform(std::string srcFrame, std::string distFrame){
	tf::StampedTransform _tf;
	static tf::TransformListener listener;
	try {
	    listener.waitForTransform(distFrame, srcFrame, ros::Time(0), ros::Duration(10.0) );
	    listener.lookupTransform(distFrame, srcFrame, ros::Time(0), _tf);
	} catch (tf::TransformException& ex) {
	    ROS_ERROR("%s",ex.what());
	}
	return _tf;
}

void ComponentMain::configCallback(loc::configConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
    dyn_conf = config;
//    _estimator._gps_height = config.height;
//    _estimator._dyn = config;
//    _estimator.modify_Q(config.Q);
}
