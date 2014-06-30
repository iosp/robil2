#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>

ros::Publisher *descisionMaking_pub;
ros::Rate * rate;

void siginthandler(int param)
{
  printf("User pressed Ctrl+C\n");
  std_msgs::String dm_msg;
  dm_msg.data="/llc/Resume";
  descisionMaking_pub->publish(dm_msg);
  ros::spinOnce();
  rate->sleep();
  exit(1);
}



sensor_msgs::Joy lastmsg;

void joystickCallback(const sensor_msgs::Joy::ConstPtr & msg){
	lastmsg.header.frame_id="ok";
	lastmsg.axes=msg->axes;
	lastmsg.buttons=msg->buttons;
}

int indexX=-1;
int indexY=-1;
int indexLift=-1;
int indexTilt=-1;
int indexHold=0;
int main(int argc, char **argv)
{
  signal(SIGINT, siginthandler);
  signal(SIGTERM, siginthandler);
  signal(SIGKILL, siginthandler);
  //Initialize the node and connect to master
  ros::init(argc, argv, "bobcat_joystick_node");

  //generate a node handler to handle all messages
  ros::NodeHandle n;

  n.param("axis_linear",indexX,indexX);
  n.param("axis_angular",indexY,indexY);
  n.param("axis_lift",indexLift,indexLift);
  n.param("axis_tilt",indexTilt,indexTilt);
  n.param("button_hold",indexHold,indexHold);

 descisionMaking_pub = new ros::Publisher(n.advertise<std_msgs::String>("/decision_making/events", 1000));

  ros::Subscriber joystick_sub=n.subscribe("/joy", 1, &joystickCallback);


  ros::Publisher wheelsrate_pub = n.advertise<geometry_msgs::Twist>("/wheelsrate", 1000);
  ros::Publisher armrate_pub = n.advertise<geometry_msgs::Vector3>("/armrate", 1000);
  ros::Rate * rate=new ros::Rate(5);
  ros::spinOnce();
  (*rate).sleep();

  double supporter_max=1.6;
  double supporter_min=0;

  double loader_max=1.0;
  double loader_min=-1.0;

  double bracket_max=1.0;
  double bracket_min=-1.0;

  double supporter_val=0;
  double loader_val=0;
  double bracket_val=0;

  std_msgs::String dm_msg;
  dm_msg.data="/Teleoperation";
  descisionMaking_pub->publish(dm_msg);
  ros::spinOnce();
  (*rate).sleep();
  geometry_msgs::Vector3 vectorMsg;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */




	geometry_msgs::Twist twistMsg;


    if (lastmsg.header.frame_id.compare("ok")==0) {
	lastmsg.header.frame_id="";

		twistMsg.linear.x=0.5*lastmsg.axes[indexX];
		twistMsg.angular.x=-0.5*lastmsg.axes[indexY];

		if(lastmsg.buttons[indexHold]==0)
		{
			supporter_val=supporter_val+0.05*lastmsg.axes[indexLift];
			supporter_val=supporter_val>supporter_max?supporter_max:supporter_val;
			supporter_val=supporter_val<supporter_min?supporter_min:supporter_val;
			loader_val=loader_val+0.05*lastmsg.axes[indexTilt];
			loader_val=loader_val>loader_max?loader_max:loader_val;
			loader_val=loader_val<loader_min?loader_min:loader_val;
		}
		else
		{
			bracket_val=bracket_val+0.1*lastmsg.axes[indexTilt];
			bracket_val=bracket_val>bracket_max?bracket_max:bracket_val;
			bracket_val=bracket_val<bracket_min?bracket_min:bracket_val;
		}

		vectorMsg.x=supporter_val;
		vectorMsg.y=loader_val;
		vectorMsg.z=bracket_val;

		armrate_pub.publish(vectorMsg);
		wheelsrate_pub.publish(twistMsg);

        ros::spinOnce();   
    }
    ros::spinOnce();

  }
 
  dm_msg.data="/Autonomy";
  descisionMaking_pub->publish(dm_msg);
  ros::spinOnce();
  (*rate).sleep();
  return 0;
}
