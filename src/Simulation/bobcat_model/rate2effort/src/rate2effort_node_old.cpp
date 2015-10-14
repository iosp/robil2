#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <iostream>
#include <time.h>
#include <bondcpp/bond.h>

  int movie = 1 ;
  int emergancy = 1 ;
  ros::Publisher front_left_pub_;
  ros::Publisher front_right_pub_;
  ros::Publisher back_left_pub_;
  ros::Publisher back_right_pub_;
  ros::Publisher platform_hb_pub_;

  ros::Publisher supporter_pub_;
  ros::Publisher loader_pub_;
  ros::Publisher brackets_pub_;
  geometry_msgs::Twist::Ptr wheels (new geometry_msgs::Twist()) ;

  ros::Time e_stop;

void wheelsCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	std_msgs::Float64 lin;
	std_msgs::Float64 ang;

	std_msgs::Float64 pub;
	lin.data = msg->linear.x ;
	ang.data = msg->angular.x ;

	if(msg->linear.x > 1)
		lin.data = 1 ;
	if(msg->angular.x > 1)
		ang.data = 1 ;
	if(msg->linear.x < -1)
		lin.data = -1 ;
	if(msg->angular.x < -1)
		ang.data = -1 ;
if(!movie){
    	pub.data = (0.5*lin.data + 0.3*ang.data)*30*emergancy;
    	front_left_pub_.publish(pub);
    	back_left_pub_.publish(pub);

    	pub.data = (0.5*lin.data - 0.3*ang.data)*30*emergancy;
    	front_right_pub_.publish(pub);
    	back_right_pub_.publish(pub);
}
else{
	pub.data = (0.5*lin.data + 0.3*ang.data)*50*emergancy;
    	front_left_pub_.publish(pub);
    	back_left_pub_.publish(pub);

    	pub.data = (0.5*lin.data - 0.3*ang.data)*50*emergancy;
    	front_right_pub_.publish(pub);
    	back_right_pub_.publish(pub);
}
}

void armCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	std_msgs::Float64 pub;
	e_stop = ros::Time::now();
	
    	pub.data = msg->x;
    	supporter_pub_.publish(pub);

    	pub.data = msg->y;
    	loader_pub_.publish(pub);

    	pub.data = msg->z;
    	brackets_pub_.publish(pub);
}

void ThCallback (const std_msgs::Float64ConstPtr &msg)
{
	e_stop = ros::Time::now();
	wheels->linear.x = msg->data ;
}

void StCallback	(const std_msgs::Float64ConstPtr &msg)
{
	e_stop = ros::Time::now();
	geometry_msgs::Twist::ConstPtr pub;
	wheels->angular.x = msg->data ;
	pub = wheels;
	wheelsCallback(pub);
}

void JoCallback	(const sensor_msgs::JointState &msg)
{
	std_msgs::Float64 pub;

	for(int i = 0 ; i < msg.position.size() ; i++)
	{
	pub.data = msg.position[i];

	if(msg.name[i] == "supporter_joint"){
		supporter_pub_.publish(pub);
	}
	else if (msg.name[i] == "loader_joint"){
		loader_pub_.publish(pub);
	}
	else if(msg.name[i] == "brackets_joint"){
		brackets_pub_.publish(pub);
	}
  }
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "rate2effort");

 //geometry_msgs::Twist::Ptr wheels2(new geometry_msgs::Twist());
 // wheels = wheels2 ;

  ros::NodeHandle n;

  front_left_pub_ = n.advertise<std_msgs::Float64>("/Sahar/front_left_wheel_velocity_controller/command", 100);
  front_right_pub_ = n.advertise<std_msgs::Float64>("/Sahar/front_right_wheel_velocity_controller/command", 100);
  back_left_pub_ = n.advertise<std_msgs::Float64>("/Sahar/back_left_wheel_velocity_controller/command", 100);
  back_right_pub_ = n.advertise<std_msgs::Float64>("/Sahar/back_right_wheel_velocity_controller/command", 100);

  supporter_pub_ = n.advertise<std_msgs::Float64>("/Sahar/supporter_position_controller/command", 100);
  loader_pub_ = n.advertise<std_msgs::Float64>("/Sahar/loader_position_controller/command", 100);
  brackets_pub_ = n.advertise<std_msgs::Float64>("/Sahar/brackets_position_controller/command", 100);
  platform_hb_pub_ = n.advertise<std_msgs::Bool>("/Sahar/link_with_platform" , 100);

  ros::Subscriber twist_sub_ = n.subscribe("/wheelsrate", 1000, wheelsCallback );
  ros::Subscriber arm_sub_ = n.subscribe("/armrate", 1000, armCallback );
  ros::Subscriber Throttle_rate_sub = n.subscribe("/LLC/EFFORTS/Throttle" , 1000, ThCallback);
  ros::Subscriber Steering_rate_sub = n.subscribe("/LLC/EFFORTS/Steering" , 1000, StCallback);
  ros::Subscriber Joint_rate_sub = n.subscribe("/LLC/EFFORTS/Joints" , 1000, JoCallback );

  ros::Rate loop_rate(10);
  std_msgs::Bool connection;

   while (ros::ok())
     {
   /* implementing E-stop, if LLC-node is offline, or doesn't publish any new messages for 1 second */
/*	  t_out = (ros::Time::now().operator -(e_stop));
	   	 if((t_out.sec)>=1)
	   		emergancy = 0 ;
	   	 else
	   		 emergancy = 1 ;*/
	   connection.data = true;
	   platform_hb_pub_.publish(connection);
       ros::spinOnce();
       loop_rate.sleep();
     }
   connection.data = false;
   platform_hb_pub_.publish(connection);
  return 0;
}






