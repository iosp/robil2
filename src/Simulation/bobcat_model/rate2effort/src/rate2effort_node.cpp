#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
//#include "rate2effort/WheelsRate_msg.h"
#include <sstream>


  ros::Publisher front_left_pub_;
  ros::Publisher front_right_pub_;
  ros::Publisher back_left_pub_;
  ros::Publisher back_right_pub_;

  ros::Publisher supporter_pub_;
  ros::Publisher loader_pub_;


void wheelsCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	std_msgs::Float64 pub;    
	
    	pub.data = (msg->linear.x + msg->angular.x )*100;
    	front_left_pub_.publish(pub);
	back_left_pub_.publish(pub);

    	pub.data = (msg->linear.x - msg->angular.x )*100;
	front_right_pub_.publish(pub);
	back_right_pub_.publish(pub);
}

void armCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	std_msgs::Float64 pub;    
	
    	pub.data = msg->x;
    	supporter_pub_.publish(pub);

    	pub.data = msg->y;
	loader_pub_.publish(pub);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "rate2effort");

  ros::NodeHandle n;

  front_left_pub_ = n.advertise<std_msgs::Float64>("/bobcat/front_left_wheel_velocity_controller/command", 100);
  front_right_pub_ = n.advertise<std_msgs::Float64>("/bobcat/front_right_wheel_velocity_controller/command", 100);
  back_left_pub_ = n.advertise<std_msgs::Float64>("/bobcat/back_left_wheel_velocity_controller/command", 100);
  back_right_pub_ = n.advertise<std_msgs::Float64>("/bobcat/back_right_wheel_velocity_controller/command", 100);

  supporter_pub_ = n.advertise<std_msgs::Float64>("/bobcat/supporter_position_controller/command", 100);
  loader_pub_ = n.advertise<std_msgs::Float64>("/bobcat/loader_position_controller/command", 100);

  ros::Subscriber twist_sub_ = n.subscribe("/wheelsrate", 1000, wheelsCallback );
  ros::Subscriber arm_sub_ = n.subscribe("/armrate", 1000, armCallback );

  ros::Rate loop_rate(10);	
  ros::spin();

  return 0;
}
