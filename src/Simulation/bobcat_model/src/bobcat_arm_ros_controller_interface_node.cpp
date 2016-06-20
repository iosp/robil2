#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>
#include <iostream>
#include <time.h>
#include <bondcpp/bond.h>


  
  ros::Publisher supporter_pub_;
  ros::Publisher loader_pub_;
  ros::Publisher brackets_pub_;



void armCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	std_msgs::Float64 pub;
	
    	pub.data = msg->x;
    	supporter_pub_.publish(pub);

    	pub.data = msg->y;
    	loader_pub_.publish(pub);

    	pub.data = msg->z;
    	brackets_pub_.publish(pub);
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
  
  ros::init(argc, argv, "bobcat_arm_ros_controller_interface_node");

 
  ros::NodeHandle n;


  supporter_pub_ = n.advertise<std_msgs::Float64>("/Sahar/supporter_position_controller/command", 100);
  loader_pub_ = n.advertise<std_msgs::Float64>("/Sahar/loader_position_controller/command", 100);
  brackets_pub_ = n.advertise<std_msgs::Float64>("/Sahar/brackets_position_controller/command", 100);

  ros::Subscriber arm_sub_ = n.subscribe("/armrate", 1000, armCallback );
  ros::Subscriber Joint_rate_sub = n.subscribe("/LLC/EFFORTS/Joints" , 1000, JoCallback );

  ros::Rate loop_rate(10);
   while (ros::ok())
     {
       ros::spinOnce();
       loop_rate.sleep();
     }

  return 0;
}






