#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main(int argc, char **argv)
{
  //Initialize the node and connect to master
  ros::init(argc, argv, "bobcat_keyboad_node");

  //generate a node handler to handle all messages
  ros::NodeHandle n;

  ros::Publisher back_left_wheel_pub = n.advertise<std_msgs::Float64>("/bobcat/back_left_wheel_velocity_controller/command", 1000);
  ros::Publisher back_right_wheel_pub = n.advertise<std_msgs::Float64>("/bobcat/back_right_wheel_velocity_controller/command", 1000);
  ros::Publisher loader_pose_pub = n.advertise<std_msgs::Float64>("/bobcat/loader_position_controller/command", 1000);
  ros::Publisher shaft_pose_pub = n.advertise<std_msgs::Float64>("/bobcat/shaft_position_controller/command", 1000);
  ros::Publisher front_left_wheel_pub = n.advertise<std_msgs::Float64>("/bobcat/front_left_wheel_velocity_controller/command", 1000);
  ros::Publisher front_right_wheel_pub = n.advertise<std_msgs::Float64>("/bobcat/front_right_wheel_velocity_controller/command", 1000);
  ros::Publisher front_arm_pose_pub = n.advertise<std_msgs::Float64>("/bobcat/front_arm_position_controller/command", 1000);
  ros::Publisher back_arm_pose_pub = n.advertise<std_msgs::Float64>("/bobcat/back_arm_position_controller/command", 1000);

  ros::Rate rate(5);
  //structs to hold the shell buffer
  struct termios stdio;
  struct termios old_stdio;

  unsigned char c='D';
  tcgetattr(STDOUT_FILENO,&old_stdio);

  memset(&stdio,0,sizeof(stdio));
  stdio.c_iflag=0;
  stdio.c_oflag=0;
  stdio.c_cflag=0;
  stdio.c_lflag=0;
  stdio.c_cc[VMIN]=1;
  stdio.c_cc[VTIME]=0;
  tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
  tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking

  while (ros::ok() && c!='q')
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
	std_msgs::Float64 msg;
	std_msgs::Float64 msg2;
	while(read(STDIN_FILENO,&c,1)>0){}


    if (c!='q' && c!=' ') {
    	msg.data=10;
    	msg2.data=-10;
    	switch(c){
    		case 'i':
    			front_left_wheel_pub.publish(msg);
    			front_right_wheel_pub.publish(msg);
    			back_left_wheel_pub.publish(msg);
    			back_right_wheel_pub.publish(msg);
    			break;
    		case 'j':

    			back_right_wheel_pub.publish(msg);
    			front_right_wheel_pub.publish(msg);
       			back_left_wheel_pub.publish(msg2);
    			front_left_wheel_pub.publish(msg2);
    			break;
    		case 'l':
    			back_left_wheel_pub.publish(msg);
    			front_left_wheel_pub.publish(msg);
    			back_right_wheel_pub.publish(msg2);
    			front_right_wheel_pub.publish(msg2);
    			break;
    		case 'k':
    			front_left_wheel_pub.publish(msg2);
    			front_right_wheel_pub.publish(msg2);
    			back_left_wheel_pub.publish(msg2);
    			back_right_wheel_pub.publish(msg2);
    			break;
    		case 't':
    			loader_pose_pub.publish(msg);
    			break;
    		case 'g':
    			loader_pose_pub.publish(msg2);
    			break;
    		case 'r':
    			shaft_pose_pub.publish(msg);
    			break;
    		case 'f':
    			shaft_pose_pub.publish(msg2);
    			break;
    		case 'e':
    			front_arm_pose_pub.publish(msg);
    			break;
    		case 'd':
    			front_arm_pose_pub.publish(msg2);
    			break;
    		case 'w':
    			back_arm_pose_pub.publish(msg);
    			break;
    		case 's':
    			back_arm_pose_pub.publish(msg2);
    			break;

    	}
        ros::spinOnce();
        rate.sleep();
        c=' ';
    }else{
    	  //stop all activity
        msg.data=0;
		back_left_wheel_pub.publish(msg);
		back_right_wheel_pub.publish(msg);
		front_left_wheel_pub.publish(msg);
		front_right_wheel_pub.publish(msg);
	    ros::spinOnce();
        rate.sleep();
    }

  }
  tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);

  return 0;
}
