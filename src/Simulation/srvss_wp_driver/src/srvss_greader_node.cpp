#include "ros/ros.h"

#include <sstream>
#include <iostream>
#include <cmath>      // std::abs()

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

#include "gazebo_msgs/ModelStates.h"

#include "geometry_msgs/Point.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"


#define PI 3.14159265359
#define MIN_ALLOWED_OBS_DIST 2
#define MAX_ALLOWED_ROLL_ANG (15*PI/180)
#define MAX_ALLOWED_PITCH_ANG (30*PI/180)

   ros::Publisher greades_pub_;
   ros::Publisher reset_pub_;

   std_msgs::Float32MultiArray greads_array;
   std_msgs::Bool reset_flag;


float scenario_obj_min_dist = 100;
float scenario_roll_max_ang = 0;
float scenario_pitch_max_ang = 0;

void model_statesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{

	int obj_num = msg->name.size();

	geometry_msgs::Point bobcat_pos = msg->pose[1].position;
    float obj_dis = 0;
    float obj_min_dist = 100;

    // object distance
	for (int i=2 ; i<obj_num ; i++)
	{
		geometry_msgs::Point obj_pos = msg->pose[i].position;
        obj_dis = sqrt( (obj_pos.x - bobcat_pos.x)*(obj_pos.x - bobcat_pos.x) + (obj_pos.y - bobcat_pos.y)*(obj_pos.y - bobcat_pos.y) );
        obj_min_dist = std::min(obj_min_dist,obj_dis);
	}
		scenario_obj_min_dist = std::min(scenario_obj_min_dist , obj_min_dist);


    // platform stability (rollover)
		tf::Quaternion bobcat_orintation_quat(msg->pose[1].orientation.x , msg->pose[1].orientation.y , msg->pose[1].orientation.z , msg->pose[1].orientation.w);
		tf::Matrix3x3 mat(bobcat_orintation_quat);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);

		scenario_roll_max_ang = std::max( scenario_roll_max_ang , std::abs((float)roll) );
		scenario_pitch_max_ang = std::max( scenario_pitch_max_ang , std::abs((float)pitch) );


     ROS_INFO(" vehicle stability grade : roll = %f   , pitch = %f " , scenario_roll_max_ang, scenario_pitch_max_ang);
     ROS_INFO(" obstacle proximity grade : dis = %f " , scenario_obj_min_dist);

  	greads_array.data.clear();
    greads_array.data.push_back(scenario_obj_min_dist);
  	greads_array.data.push_back(scenario_roll_max_ang);
  	greads_array.data.push_back(scenario_pitch_max_ang);
    greades_pub_.publish(greads_array);



  	if (  (scenario_obj_min_dist <= MIN_ALLOWED_OBS_DIST)  || ( scenario_roll_max_ang >= MAX_ALLOWED_ROLL_ANG) || (scenario_pitch_max_ang >= MAX_ALLOWED_PITCH_ANG)  )
  	{
  		reset_flag.data = true;
  		ROS_INFO(" vreset_flag = %d " , reset_flag.data);
  	}
   reset_pub_.publish(reset_flag);
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "srvss_greader_node");

   ros::NodeHandle n;

   reset_flag.data = false;
   ros::Subscriber model_states_sub_ = n.subscribe("/gazebo/model_states", 100, model_statesCallback );

   greades_pub_ = n.advertise<std_msgs::Float32MultiArray>("/srvss/greade", 100);
   reset_pub_ = n.advertise<std_msgs::Bool>("/srvss/scenario_reset", 100);

   ros::spin();

   return 0;
}
