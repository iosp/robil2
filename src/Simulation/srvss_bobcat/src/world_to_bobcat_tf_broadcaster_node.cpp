#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "gazebo_msgs/ModelStates.h"



void poseCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
  //geometry_msgs::Point bobcat_pos = msg->pose[1].position;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  //transform.setOrigin( tf::Vector3(msg->pose[1].position.x, msg->pose[1].position.y, msg->pose[1].position.z) );
  tf::Vector3 v(msg->pose[1].position.x,msg->pose[1].position.y,msg->pose[1].position.z);
  tf::Quaternion q(msg->pose[1].orientation.x,msg->pose[1].orientation.y,msg->pose[1].orientation.z,msg->pose[1].orientation.w);
  transform.setOrigin(v);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "world_to_bobcat_tf_broadcaster_node");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/gazebo/model_states", 10, &poseCallback);

  ros::spin();
  return 0;
};
