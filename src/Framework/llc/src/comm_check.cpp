#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
double v_global,omega_global;
int fine=0;
double timer=0;
void wpd_callback(const geometry_msgs::TwistStamped &msg)
{
v_global=msg.twist.linear.x;
omega_global=msg.twist.angular.z;
fine=1;
timer=0;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "llc_checkup");
  ros::NodeHandle n;

		ros::Subscriber wpd_sub = n.subscribe("/WPD/Speed", 100,wpd_callback);
	        ros::Publisher  status_pub = n.advertise<std_msgs::Float64>("/llc_status", 100);
		std_msgs::Float64 status;
		ros::Duration pause(0.001);
		ROS_INFO("LLC_checkup init");
while(ros::ok()) {
pause.sleep();
timer++;
status.data=fine;
if(timer>1000) status.data=0;
status_pub.publish(status);
ros::spinOnce();

}

return 0;
}
