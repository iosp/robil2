/*
 * MoveBase.cpp
 *
 *  Created on: Apr 30, 2014
 *      Author: dan
 */

#include "MoveBase.h"
#include "ComponentMain.h"

#include "sensor_msgs/LaserScan.h"


ros::Subscriber speed_reg_sub;
ros::Publisher speed_norm_pub;
ros::Subscriber pose_1_sub;
ros::Publisher pose_2_pub;

ros::Subscriber laser_1_sub;
ros::Publisher laser_2_pub;


void on_speed_reg_callback(const geometry_msgs::Twist::ConstPtr& m){
	config::WPD::pub::WPDVelocity o;
	o.twist = *m;
	speed_norm_pub.publish(o);
}

void on_laser_reg_callback(const sensor_msgs::LaserScan::ConstPtr& m){
	sensor_msgs::LaserScan o;
	o = *m;
	o.header.stamp = ros::Time::now();
//	o.header.stamp.nsec += 300e6;
	laser_2_pub.publish(o);
}

void on_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& m){
	geometry_msgs::PoseStamped o;
	o.pose = m->pose.pose;
	o.header.frame_id="/odom";
	pose_2_pub.publish(o);
}
void init_speed_patch(ros::NodeHandle& node){
	speed_reg_sub = node.subscribe("/WPD/speed_reg", 10, &on_speed_reg_callback);
	speed_norm_pub = node.advertise<config::WPD::pub::WPDVelocity>("/WPD/Speed", 10);

	pose_1_sub = node.subscribe("/LOC/Pose", 10, &on_pose_callback);
	pose_2_pub = node.advertise<geometry_msgs::PoseStamped>("/LOC/pose_reg", 10);

//	laser_1_sub = node.subscribe("/scan", 1, &on_laser_reg_callback);
//	laser_2_pub = node.advertise<sensor_msgs::LaserScan>("/scan_good", 1);
}

MoveBase::MoveBase(ComponentMain* comp)
:
	comp(comp), resend_thread(0)

{
	ros::NodeHandle node;
	sub_location = node.subscribe("/test/location", 10, &MoveBase::on_sub_loc, this);
	sub_location_cov = node.subscribe("/test/location_cov", 10, &MoveBase::on_sub_loc_cov, this);
	sub_speed = node.subscribe("/cmd_vel", 10, &MoveBase::on_sub_speed, this);

	init_speed_patch(node);
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
	p.header.frame_id = "/map";
	p.header.stamp = ros::Time::now();
	p.pose.pose = msg->pose;
	on_position_update(p);
}
void MoveBase::on_sub_loc_cov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	geometry_msgs::PoseWithCovarianceStamped p = *msg;
	p.header.frame_id = "/map";
	on_position_update(p);
}
void MoveBase::on_sub_speed(const geometry_msgs::Twist::ConstPtr& msg){
	on_speed(*msg);
}
void MoveBase::on_speed(const geometry_msgs::Twist& msg){
	geometry_msgs::TwistStamped tw;
	tw.header.frame_id="/map";
	tw.header.stamp = ros::Time::now();
	tw.twist = msg;
	comp->publishWPDVelocity(tw);
}

void send_static_ziro_tf_links(ComponentMain* comp){
	tf::Quaternion q(0,0,0,1);
	tf::Vector3 l(0,0,0);
	tf::Transform tf(q,l);
	comp->publishTransform(tf,"world", "map");
	comp->publishTransform(tf,"map", "odom");
	comp->publishTransform(tf,"map", "laser");
}

const double x_offset_for_bebug=0;
const double y_offset_for_bebug=0;

void MoveBase::on_position_update(const config::PP::sub::Location& msg){
	tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	tf::Vector3 l(msg.pose.pose.position.x+x_offset_for_bebug,msg.pose.pose.position.y+y_offset_for_bebug,msg.pose.pose.position.z);
	tf::Transform tf(q,l);
	last_location = msg;
	send_static_ziro_tf_links(comp);
	comp->publishTransform(tf,"odom", "base_link");
	if(resend_thread==0){
		resend_thread = new boost::thread(boost::bind(&MoveBase::resend,this));
	}
}

#include <boost/thread.hpp>
void MoveBase::resend(){
	ROS_INFO("Start TF structure sender(20Hz): world->map(->odom->base_link|->laser)");
	while(ros::ok and not boost::this_thread::interruption_requested()){
		//ROS_INFO("send tf");
		on_position_update(last_location);
		boost::this_thread::sleep(boost::posix_time::millisec(50));
	}
}

