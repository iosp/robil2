/*
 * MoveBase.cpp
 *
 *  Created on: Apr 30, 2014
 *      Author: dan
 */

#include "MoveBase.h"
#include "ComponentMain.h"

MoveBase::MoveBase(ComponentMain* comp)
:
	comp(comp), resend_thread(0)

{
	ros::NodeHandle node;
	sub_location = node.subscribe("/test/location", 10, &MoveBase::on_sub_loc, this);
	sub_location_cov = node.subscribe("/test/location_cov", 10, &MoveBase::on_sub_loc_cov, this);
	sub_speed = node.subscribe("/cmd_vel", 10, &MoveBase::on_sub_speed, this);
}

MoveBase::~MoveBase() {
	if(resend_thread){
		resend_thread->interrupt();
		resend_thread->join();
		resend_thread = 0;
	}
}

void MoveBase::on_sub_loc(const geometry_msgs::PoseStamped::ConstPtr& msg){
	geometry_msgs::PoseWithCovarianceStamped p;
	p.header.frame_id = "map";
	p.header.stamp = ros::Time::now();
	p.pose.pose = msg->pose;
	on_position_update(p);
}
void MoveBase::on_sub_loc_cov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	geometry_msgs::PoseWithCovarianceStamped p = *msg;
	p.header.frame_id = "map";
	on_position_update(p);
}
void MoveBase::on_sub_speed(const geometry_msgs::Twist::ConstPtr& msg){
	on_speed(*msg);
}
void MoveBase::on_speed(const geometry_msgs::Twist& msg){
	geometry_msgs::TwistStamped tw;
	tw.header.frame_id="map";
	tw.header.stamp = ros::Time::now();
	tw.twist = msg;
	comp->publishWPDVelocity(tw);
}
void MoveBase::on_position_update(const config::PP::sub::Location& msg){
	tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	tf::Vector3 l(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z);
	//tf::Quaternion q(0,0,0,1);
	//tf::Vector3 l(0,0,0);
	tf::Transform tf(q,l);
	last_location = msg;
	comp->publishTransform(tf,"map", "base_link");
	if(resend_thread==0){ resend_thread = new boost::thread(boost::bind(&MoveBase::resend,this)); }
}

#include <boost/thread.hpp>
void MoveBase::resend(){
	while(ros::ok and not boost::this_thread::interruption_requested()){
		//ROS_INFO("send tf");
		on_position_update(last_location);
		boost::this_thread::sleep(boost::posix_time::millisec(100));
	}
}

