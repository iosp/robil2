#include "ros/ros.h"

#include <iostream>
#include <string>     // std::string, std::stof

#include "std_msgs/String.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"

#include <sstream>


 const int max_wp_num = 10;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "srvss_wp_keyboard_commander_node");
  
  ros::NodeHandle n;
  
  ros::Publisher WP_command_pub = n.advertise<geometry_msgs::Pose>("/WP_command", 100);
  
  geometry_msgs::Pose WP_command; 

  float wp_x, wp_y, wp_V;

  ros::Rate loop_rate(1);
  while (ros::ok())
       {  
	  std::cout << " wp_X =  ";
 	  std::cin >> wp_x ; 
	  std::cout << " wp_Y =  ";
 	  std::cin >> wp_y ;
	  std::cout << " wp_Speed =  ";
 	  std::cin >> wp_V ;
         	         
 	  int i = 0;
          WP_command.position.x = wp_x;
          WP_command.position.y = wp_y;  
          WP_command.position.z = wp_V;

          ROS_INFO("Received :  wp_X = %f  , wp_Y = %f, wp_Speed = %f " , wp_x , wp_y, wp_V );

          WP_command_pub.publish(WP_command);
     
  ros::spinOnce();

  loop_rate.sleep();  


      }
  return 0;
}
