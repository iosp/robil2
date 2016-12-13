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

geometry_msgs::Twist last_published_speed_message;
bool published_speed_message_before;
double angular_z_dampening_alpha;
double linear_x_dampening_alpha;

void on_speed_reg_callback(const geometry_msgs::Twist::ConstPtr& m){
	geometry_msgs::TwistStamped o;
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
	o.header.frame_id="/ODOM";
	pose_2_pub.publish(o);
}
void init_speed_patch(ros::NodeHandle& node){
	speed_reg_sub = node.subscribe("/WPD/speed_reg", 10, &on_speed_reg_callback);
	speed_norm_pub = node.advertise<geometry_msgs::TwistStamped>("/WPD/Speed", 10);

	pose_1_sub = node.subscribe("/LOC/Pose", 10, &on_pose_callback);
	pose_2_pub = node.advertise<geometry_msgs::PoseStamped>("/LOC/pose_reg", 10);

//	laser_1_sub = node.subscribe("/scan", 1, &on_laser_reg_callback);
//	laser_2_pub = node.advertise<sensor_msgs::LaserScan>("/scan_good", 1);
}

MoveBase::MoveBase(ComponentMain* comp)
: comp(comp)
, resend_thread(0)
, pub_frequancy (10)
, last_move_base_vel_ok (false)
, last_real_vel_ok (false)
{
	ros::NodeHandle node;

	boost::thread auto_vel_republisher (&MoveBase::running, this);

	sub_location = node.subscribe("/test/location", 10, &MoveBase::on_sub_loc, this);
	sub_location_cov = node.subscribe("/test/location_cov", 10, &MoveBase::on_sub_loc_cov, this);
	sub_speed = node.subscribe("/cmd_vel", 10, &MoveBase::on_sub_speed, this);

	published_speed_message_before = false;

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
	p.header.frame_id = "/WORLD";
	p.header.stamp = ros::Time::now();
	p.pose.pose = msg->pose;
	on_position_update(p);
}
void MoveBase::on_sub_loc_cov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	geometry_msgs::PoseWithCovarianceStamped p = *msg;
	p.header.frame_id = "/WORLD";
	on_position_update(p);
}
void MoveBase::on_sub_speed(const geometry_msgs::Twist::ConstPtr& msg){
	on_speed(*msg);
}
void MoveBase::on_speed(const geometry_msgs::Twist& msg){
	geometry_msgs::TwistStamped tw;
	tw.header.frame_id="/WORLD";
	tw.header.stamp = ros::Time::now();
	tw.twist = msg;

	set_last_move_base_vel(msg);

	//check validity of the message. Some values must be 0.
	if (tw.twist.angular.x or tw.twist.angular.y or tw.twist.linear.z or tw.twist.linear.y)
	{
		ROS_WARN_STREAM ("Invalid values in speed message : " << tw.twist);
		tw.twist.angular.x = tw.twist.angular.y = tw.twist.linear.z = tw.twist.linear.y = 0;
	}
	
	double angle_scale=5.0;
	ros::param::param("/wpd/angle_scale",angle_scale,5.0);
	tw.twist.angular.z*=angle_scale;
	
	double lin_scale=1.0;
	ros::param::param("/wpd/lin_scale",lin_scale,1.0);
	tw.twist.linear.x*=lin_scale;
	
	//perform dampening according to last twist message
	if(published_speed_message_before){
		ros::param::param("/wpd/angular_dampening_alpha",angular_z_dampening_alpha,0.8);
		ros::param::param("/wpd/linear_x_dampening_alpha",linear_x_dampening_alpha,0.8);

		tw.twist.angular.z = (last_published_speed_message.angular.z * angular_z_dampening_alpha) +
						(tw.twist.angular.z * (1-angular_z_dampening_alpha));
		tw.twist.linear.x = (last_published_speed_message.linear.x * linear_x_dampening_alpha) +
				(tw.twist.linear.x * (1-linear_x_dampening_alpha));
	}
	comp->publishWPDVelocity(tw);
	set_last_real_vel (tw);

	last_published_speed_message = tw.twist;
	published_speed_message_before = true;
}

void send_static_ziro_tf_links(ComponentMain* comp){
// 	tf::Quaternion q(0,0,0,1);
// 	tf::Vector3 l(0,0,0);
// 	tf::Transform tf(q,l);
// 	comp->publishTransform(tf,"world", "map");
// 	comp->publishTransform(tf,"map", "odom");
// 	comp->publishTransform(tf,"map", "laser");
}
void send_static_base_center_offset(ComponentMain* comp){
// 	tf::Quaternion q(0,0,0,1);
// 	tf::Vector3 l(0.75,0,0);
// 	tf::Transform tf(q,l);
// 	comp->publishTransform(tf,"base_link", "base_link_center");
}
const double x_offset_for_bebug=0;
const double y_offset_for_bebug=0;

void MoveBase::on_position_update(const geometry_msgs::PoseWithCovarianceStamped& msg){
// 	tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
// 	tf::Vector3 l(msg.pose.pose.position.x+x_offset_for_bebug,msg.pose.pose.position.y+y_offset_for_bebug,msg.pose.pose.position.z);
// 	tf::Transform tf(q,l);
// 	last_location = msg;
// 	send_static_ziro_tf_links(comp);
// 	//comp->publishTransform(tf,"odom", "base_link");
// 	comp->publishTransform(tf,"world", "odom");
// 	send_static_base_center_offset(comp);
// 	if(resend_thread==0){
// 		resend_thread = new boost::thread(boost::bind(&MoveBase::resend,this));
// 	}
}

#include <boost/thread.hpp>
void MoveBase::resend(){
	//ROS_INFO("Start TF structure sender(20Hz): world->map(->odom->base_link->base_link_center|->laser)");
	while(ros::ok and not boost::this_thread::interruption_requested()){
		//ROS_INFO("send tf");
		on_position_update(last_location);
		boost::this_thread::sleep(boost::posix_time::millisec(50));
	}
}





#define VAR_MUTEX boost::mutex var_mutex;
#define VAR_MUTEX_LOCK boost::mutex::scoped_lock l_for_var_mutex (var_mutex);
VAR_MUTEX

void MoveBase::set_last_move_base_vel (const geometry_msgs::Twist & new_move_base_vel){
	VAR_MUTEX_LOCK
	last_move_base_vel = new_move_base_vel;
	last_move_base_vel_ok = true;
}

void MoveBase::set_last_real_vel (const geometry_msgs::TwistStamped & new_real_vel){
	VAR_MUTEX_LOCK
	last_real_vel.linear.x = new_real_vel.twist.linear.x;
	last_real_vel.linear.y = new_real_vel.twist.linear.y;
	last_real_vel.linear.z = new_real_vel.twist.linear.z;
	last_real_vel.angular.x = new_real_vel.twist.angular.x;
	last_real_vel.angular.y = new_real_vel.twist.angular.y;
	last_real_vel.angular.z = new_real_vel.twist.angular.z;
	last_real_vel_ok = true;
}

geometry_msgs::Twist MoveBase::get_last_move_base_vel (){
	VAR_MUTEX_LOCK
	return last_move_base_vel;
}

geometry_msgs::Twist MoveBase::get_last_real_vel (){
	VAR_MUTEX_LOCK
	return last_real_vel;
}

void MoveBase::set_min_real_vel (const geometry_msgs::Twist & new_min_real_vel){
	VAR_MUTEX_LOCK
	min_real_vel = new_min_real_vel;
}

geometry_msgs::Twist MoveBase::get_min_real_vel (){
	VAR_MUTEX_LOCK
	return min_real_vel;
}


void MoveBase::set_pub_frequancy (const double new_pub_frequancy){
	VAR_MUTEX_LOCK
	pub_frequancy = new_pub_frequancy;
}

double MoveBase::get_pub_frequancy (){
	VAR_MUTEX_LOCK
	return pub_frequancy;
}



void MoveBase::load_param (){
	ros::NodeHandle node;
	double min_linear = 0.15;
	double min_angular = 0.1;
	node.getParamCached ("/wpd/min_linear", min_linear);
	node.getParamCached ("/wpd/min_angular", min_angular);
	geometry_msgs::Twist min_real_vel;
	min_real_vel.linear.x = min_linear;
	min_real_vel.angular.z = min_angular;
	set_min_real_vel(min_real_vel);

	double pub_frequancy = get_pub_frequancy();
	node.getParamCached ("/wpd/pub_frequancy", pub_frequancy);
	set_pub_frequancy (pub_frequancy);
}

void MoveBase::running (){
	while (ros::ok()){
//		std::cout << "started = " << started << std::endl;
//		std::cout << "last_vel_ok = " << last_vel_ok << std::endl;
//		std::cout << "min_vel_ok = " << min_vel_ok << std::endl;
		load_param();
		geometry_msgs::Twist local_last_real_vel = get_last_real_vel();
		geometry_msgs::Twist local_min_real_vel = get_min_real_vel();
//		std::cout << "min_real_vel = " << local_min_real_vel.linear.x << ", " << local_min_real_vel.angular.z << std::endl;
		geometry_msgs::Twist local_last_move_base_vel = get_last_move_base_vel();
//		std::cout << fabs(local_last_real_vel.linear.x) << ">=" << fabs(local_min_real_vel.linear.x) <<
//		" | " <<  fabs(local_last_real_vel.angular.z) << ">=" << fabs(local_min_real_vel.angular.z) << std::endl;
		if ((fabs(local_last_real_vel.linear.x) >= fabs(local_min_real_vel.linear.x) or fabs(local_last_real_vel.angular.z) >= fabs(local_min_real_vel.angular.z))
			 and last_move_base_vel_ok and last_real_vel_ok){

			on_speed (local_last_move_base_vel);

//			std::cout << "Redundant velocity is published" << std::endl;
		} else
			if (last_move_base_vel_ok and last_real_vel_ok){
				// pubslish stop velocity and stop republishing
				geometry_msgs::TwistStamped stop_vel;
				stop_vel.header.frame_id="/WORLD";
				stop_vel.header.stamp = ros::Time::now();
				stop_vel.twist.linear.x = 0;
				stop_vel.twist.linear.y = 0;
				stop_vel.twist.linear.z = 0;
				stop_vel.twist.angular.x = 0;
				stop_vel.twist.angular.y = 0;
				stop_vel.twist.angular.z = 0;
				comp->publishWPDVelocity(stop_vel);
				last_move_base_vel_ok = false;
				last_real_vel_ok = false;
//				std::cout << "Stop velocity is published" << std::endl;
			}
		double local_pub_frequancy = get_pub_frequancy();
		double pub_interval = round (1 / local_pub_frequancy * 1000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(pub_interval));
	}
}




