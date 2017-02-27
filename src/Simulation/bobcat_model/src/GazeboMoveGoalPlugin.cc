// Written By : Yossi Cohen

// If the plugin is not defined then define it
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <random>
// Gazebo Libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/PoseStamped.h>
// ROS Communication
#include "ros/ros.h"

namespace gazebo
{
class GazeboMoveGoalPlugin : public ModelPlugin
{
  ///  Constructor
public:
  GazeboMoveGoalPlugin() {}

  /// The load function is called by Gazebo when the plugin is inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is attached to.
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) // we are not using the pointer to the sdf file so its commanted as an option
  {
    this->model = _model;
    this->Ros_nh = new ros::NodeHandle("GazeboMoveGoal_node");
    pub = this->Ros_nh->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    goal.header.seq = 0;
    goal.header.frame_id = "WORLD";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = model->GetWorldPose().pos.x;
    goal.pose.position.y = model->GetWorldPose().pos.y;
    goal.pose.position.z = 0;
    goal.pose.orientation.x=0;
    goal.pose.orientation.y=0;
    goal.pose.orientation.z=0;
    goal.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal");
    pub.publish(goal);
  }
    // Called by the world update start event, This function is the event that will be called every update
  public:
    void OnUpdate(const common::UpdateInfo &simInfo) // we are not using the pointer to the info so its commanted as an option
    {
      if(count>0){ 
        if (count%100==0){
      ROS_INFO("Sending goal");
      std::cout << "Sending goal"<< std::endl;
      pub.publish(goal);
      }
      std::cout <<count<< std::endl;
      count--;
      }
      // transform.setOrigin( tf::Vector3(model->GetWorldPose().pos.x, model->GetWorldPose().pos.y, model->GetWorldPose().pos.z) );
      // transform.setRotation(tf::Quaternion(model->GetWorldPose().rot.x,model->GetWorldPose().rot.y,model->GetWorldPose().rot.z,model->GetWorldPose().rot.w));
    }
    // Defining private Pointer to model
    physics::ModelPtr model;
    ros::Publisher pub;
    // Defining private Ros Node Handle
    ros::NodeHandle *Ros_nh;
    geometry_msgs::PoseStamped goal;
    int count=2000;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GazeboMoveGoalPlugin)
}

