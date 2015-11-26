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
  ros::Publisher effort_right;
  ros::Publisher effort_left;
  
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
        std_msgs::Float64 right;
	std_msgs::Float64 left;
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

/*
if(!movie){
    	pub.data = (0.5*lin.data + 0.3*ang.data)*30*emergancy;
    	front_left_pub_.publish(pub);
    	back_left_pub_.publish(pub);

    	pub.data = (0.5*lin.data - 0.3*ang.data)*30*emergancy;
    	front_right_pub_.publish(pub);
    	back_right_pub_.publish(pub);
}
else{
	pub.data = (0.5*lin.data + 0.5*ang.data)*50*emergancy;
    	front_left_pub_.publish(pub);
    	back_left_pub_.publish(pub);

    	pub.data = (0.5*lin.data - 0.5*ang.data)*50*emergancy;
    	front_right_pub_.publish(pub);
    	back_right_pub_.publish(pub);
}
*/


////
/*
effort.linear.x=(1.4341/10000)*(lin.data*lin.data)+(3.5161/100000)*lin.data*ang.data+(3.4545/10000)*lin.data+(2.1479/10000)*ang.data*ang.data-0.0123*ang.data+0.0199;
effort.angular.z=(-2.94631/10000)*(lin.data*lin.data)+(0.0064)*lin.data*ang.data+(0.0168)*ang.data*ang.data-0.9576*ang.data+1.7818;
effort_pub.publish(effort);


	pub.data = (0.5*lin.data + 0.5*ang.data)*emergancy;
    	//front_left_pub_.publish(pub);
	//pub.data = (0.5*lin.data + 0.5*ang.data)*emergancy;
    	back_left_pub_.publish(pub);

    	pub.data = (0.5*lin.data - 0.5*ang.data)*emergancy;
    	//front_right_pub_.publish(pub);
	//pub.data = (0.5*lin.data - 0.5*ang.data)*emergancy;
    	back_right_pub_.publish(pub);
/////
*/

right.data=(lin.data+ang.data)*2000;
left.data=(lin.data-ang.data)*2000;
effort_right.publish(right);
effort_left.publish(left);
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
  
  ros::init(argc, argv, "rate2effort_for_tracks");

 //geometry_msgs::Twist::Ptr wheels2(new geometry_msgs::Twist());
 // wheels = wheels2 ;

  ros::NodeHandle n;

  effort_right = n.advertise<std_msgs::Float64>("/Sahar/right_wheel_velocity_controller/command", 100);
  effort_left = n.advertise<std_msgs::Float64>("/Sahar/left_wheel_velocity_controller/command", 100);


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






