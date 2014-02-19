#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sensor_msgs/Joy.h>


sensor_msgs::Joy lastmsg;

void joystickCallback(const sensor_msgs::Joy::ConstPtr & msg){
	lastmsg.header.frame_id="ok";
	lastmsg.axes=msg->axes;
	lastmsg.buttons=msg->buttons;
}

int indexX=-1;
int indexY=-1;

int main(int argc, char **argv)
{
  //Initialize the node and connect to master
  ros::init(argc, argv, "bobcat_joystick_node");

  //generate a node handler to handle all messages
  ros::NodeHandle n;

  n.param("axis_linear",indexX,indexX);
  n.param("axis_angular",indexY,indexY);

  ros::Subscriber joystick_sub=n.subscribe("/joy", 1, &joystickCallback);
  ros::Publisher back_left_wheel_pub = n.advertise<std_msgs::Float64>("/bobcat/back_left_wheel_velocity_controller/command", 1000);
  ros::Publisher back_right_wheel_pub = n.advertise<std_msgs::Float64>("/bobcat/back_right_wheel_velocity_controller/command", 1000);
  ros::Publisher loader_pose_pub = n.advertise<std_msgs::Float64>("/bobcat/loader_position_controller/command", 1000);
  ros::Publisher shaft_pose_pub = n.advertise<std_msgs::Float64>("/bobcat/shaft_position_controller/command", 1000);
  ros::Publisher front_left_wheel_pub = n.advertise<std_msgs::Float64>("/bobcat/front_left_wheel_velocity_controller/command", 1000);
  ros::Publisher front_right_wheel_pub = n.advertise<std_msgs::Float64>("/bobcat/front_right_wheel_velocity_controller/command", 1000);
  ros::Publisher front_arm_pose_pub = n.advertise<std_msgs::Float64>("/bobcat/front_arm_position_controller/command", 1000);
  ros::Publisher back_arm_pose_pub = n.advertise<std_msgs::Float64>("/bobcat/back_arm_position_controller/command", 1000);

  ros::Rate rate(5);


  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */




	std_msgs::Float64 msg1;
	std_msgs::Float64 msg2;


    if (lastmsg.header.frame_id.compare("ok")==0) {
	lastmsg.header.frame_id="";


    	msg1.data=10*lastmsg.axes[indexX]-10*lastmsg.axes[indexY];
	msg2.data=10*lastmsg.axes[indexX]+10*lastmsg.axes[indexY];

	if(msg1.data>10)
		msg1.data=10;
	if(msg2.data>10)
		msg2.data=10;
	if(msg1.data<-10)
		msg1.data=-10;
	if(msg2.data<-10)
		msg2.data=-10;
	front_left_wheel_pub.publish(msg1);
    	front_right_wheel_pub.publish(msg2);
    	back_left_wheel_pub.publish(msg1);
    	back_right_wheel_pub.publish(msg2);
	
        ros::spinOnce();   
    }
    ros::spinOnce();

  }
 
  return 0;
}
