#include "ros/ros.h"

#include <iostream>
#include <string>     // std::string, std::stof


//#include "geometry_msgs/PoseWithCovariance.h"
#include "sensor_msgs/Imu.h"


#include <sstream>

#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"


#define PI 3.14159265359





void INS_callback(const sensor_msgs::Imu IMU_msg)
{   
    float azi = -1000;
    tf::Quaternion q(IMU_msg.orientation.x,IMU_msg.orientation.y,IMU_msg.orientation.z,IMU_msg.orientation.w);
    
    azi = q.getAngle()*180/PI;
    
    std::cout << " azi = " << azi << "\n";
}






int main(int argc, char **argv)
  {
  ros::init(argc, argv, "a2q_node");
  
  ros::NodeHandle n;
  
  //ros::Subscriber WP_command = n.subscribe("/LOC/Pose", 100, Pose_callback); 
    ros::Subscriber IMU_data = n.subscribe("/SENSORS/INS", 100, INS_callback); 

  
  ros::spin();
  
  return(0);
  }


