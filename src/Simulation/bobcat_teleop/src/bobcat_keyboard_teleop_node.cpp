#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <signal.h>

ros::Publisher *descisionMaking_pub;
ros::Rate * rate;

void siginthandler(int param)
{
  printf("User pressed Ctrl+C\n");
  std_msgs::String dm_msg;
  dm_msg.data="/Autonomy";
  descisionMaking_pub->publish(dm_msg);
  ros::spinOnce();
  rate->sleep();
  exit(1);
}


int main(int argc, char **argv)
{
  signal(SIGINT, siginthandler);
  //Initialize the node and connect to master
  ros::init(argc, argv, "bobcat_keyboad_node");

  //generate a node handler to handle all messages
  ros::NodeHandle n;
  descisionMaking_pub = new ros::Publisher(n.advertise<std_msgs::String>("/decision_making/events", 1000));
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


  //structs to hold the shell buffer
  struct termios stdio;
  struct termios old_stdio;

  unsigned char c='D';
  tcgetattr(STDOUT_FILENO,&old_stdio);

  memset(&stdio,0,sizeof(stdio));
  stdio.c_iflag=0;
  stdio.c_oflag=0;
  stdio.c_cflag=0;
  stdio.c_lflag=ISIG;
  stdio.c_cc[VMIN]=1;
  stdio.c_cc[VTIME]=0;
  //stdio.c_cc[VINTR]=1;
  tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
  tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking



  geometry_msgs::Vector3 vectorMsg;
  bool stop=false;

  std_msgs::String dm_msg;
  dm_msg.data="/Teleoperation";
  descisionMaking_pub->publish(dm_msg);
  ros::spinOnce();
  (*rate).sleep();
  while (ros::ok() && c!='q' && !stop)
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

	geometry_msgs::Twist twistMsg;
	while(read(STDIN_FILENO,&c,1)>0){}


    if (c!='q' && c!=' ') {
	int i=c;
	if(i==3)
	  stop=true;
	
    	switch(c){
    		case 'i':
    			twistMsg.linear.x=0.5;
    			wheelsrate_pub.publish(twistMsg);
    			break;
    		case 'j':
    			twistMsg.angular.x=-0.5;
    			twistMsg.linear.x=0.5;
    			wheelsrate_pub.publish(twistMsg);
    			break;
    		case 'l':
    			twistMsg.angular.x=0.5;
    			wheelsrate_pub.publish(twistMsg);
    			break;
    		case 'k':
    			twistMsg.linear.x=-0.5;
    			wheelsrate_pub.publish(twistMsg);
    			break;
    		case 't':
    			supporter_val=supporter_val+0.05>supporter_max?supporter_val:supporter_val+0.05;
    			vectorMsg.x=supporter_val;
    			armrate_pub.publish(vectorMsg);
    			break;
    		case 'g':
    			supporter_val=supporter_val-0.05<supporter_min?supporter_val:supporter_val-0.05;
    			vectorMsg.x=supporter_val;
    			armrate_pub.publish(vectorMsg);
    			break;
    		case 'r':
    			loader_val=loader_val+0.05>loader_max?loader_val:loader_val+0.05;
    			vectorMsg.y=loader_val;
    			armrate_pub.publish(vectorMsg);
    			break;
    		case 'f':
    			loader_val=loader_val-0.05<loader_min?loader_val:loader_val-0.05;
    			vectorMsg.y=loader_val;
    			armrate_pub.publish(vectorMsg);
    		case 'e':
    			bracket_val=bracket_val+0.05>bracket_max?bracket_val:bracket_val+0.05;
    			vectorMsg.z=bracket_val;
    			armrate_pub.publish(vectorMsg);
    			break;
    		case 'd':
    			bracket_val=bracket_val-0.05<bracket_min?bracket_val:bracket_val-0.05;
    			vectorMsg.z=bracket_val;
    			armrate_pub.publish(vectorMsg);
    			break;
    		default:
    			break;
    	}
        ros::spinOnce();
        (*rate).sleep();
        c=' ';
    }else{
    	  //stop all activity
    	twistMsg.linear.x=0;
    	twistMsg.angular.x=0;
		wheelsrate_pub.publish(twistMsg);
	    ros::spinOnce();
    	(*rate).sleep();
    }
  }

  dm_msg.data="/Autonomy";
  descisionMaking_pub->publish(dm_msg);
  ros::spinOnce();
  (*rate).sleep();
  tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);

  return 0;
}
