// Written By : Sagi Vald, modified for bobtank : or tslil

// If the plugin is not defined then define it
#ifndef _BOBTANK_DRIVE_PLUGIN_HH_
#define _BOBTANK_DRIVE_PLUGIN_HH_

// Including Used Libraries

// Boost Bind
#include <boost/bind.hpp>

// Boost Thread Mutex
#include <boost/thread/mutex.hpp>

// Standard Messages - Float Type
#include "std_msgs/Float64.h"

// Gazebo Libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <stdio.h>

// ROS Communication
#include "ros/ros.h"

// Maximum time delays
#define velocity_message_max_time_delay 0.03
#define steering_message_max_time_delay 0.03

// PID - Gain Values
#define Kp 50
#define Kd 0.1
#define Ke 20
#define wide 1.5
namespace gazebo
{
  
  class bobtankDrivePlugin : public ModelPlugin
  {
    ///  Constructor
    public: bobtankDrivePlugin() {}

    /// The load function is called by Gazebo when the plugin is inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
  public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) // we are not using the pointer to the sdf file so its commanted as an option
    {
      // Store the pointer to the model
      this->model = _model;
	
      // Store the pointers to the joints
      this->back_left_joint = this->model->GetJoint("back_left_boggie_joint");
      this->back_right_joint = this->model->GetJoint("back_right_boggie_joint");
      this->front_left_joint = this->model->GetJoint("front_left_boggie_joint");
      this->front_right_joint = this->model->GetJoint("front_right_boggie_joint");

      // Starting Timers
      steering_timer.Start();
      velocity_timer.Start();

      // Subscribe to the topic, and register a callback
      Steering_rate_sub = n.subscribe("/LLC/EFFORTS/Steering" , 1000, &bobtankDrivePlugin::On_Angular_Msg, this);
      Velocity_rate_sub = n.subscribe("/LLC/EFFORTS/Throttle" , 1000, &bobtankDrivePlugin::On_Velocity_Msg, this);
       
      // Listen to the update event. This event is broadcast every simulation iteration. 
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&bobtankDrivePlugin::OnUpdate, this, _1));


    }

    // Called by the world update start event, This function is the event that will be called every update
  public: void OnUpdate(const common::UpdateInfo & /*_info*/)  // we are not using the pointer to the info so its commanted as an option
    {
	// Applying effort to the wheels , brakes if no message income
	if (velocity_timer.GetElapsed().Float()>velocity_message_max_time_delay)
	{
		// Brakes
	  this->back_left_joint->SetForce(0,Set_Velocity_Back_Left_Wheel_Effort(1));
	  this->back_right_joint->SetForce(0,Set_Velocity_Back_Right_Wheel_Effort(1));
	  this->front_left_joint->SetForce(0,Set_Velocity_Front_Left_Wheel_Effort(1));
	  this->front_right_joint->SetForce(0,Set_Velocity_Front_Right_Wheel_Effort(1));
	}
	else
	{
	       // Accelerates
	  this->back_left_joint->SetForce(0,Set_Velocity_Back_Left_Wheel_Effort(0));
	  this->back_right_joint->SetForce(0,Set_Velocity_Back_Right_Wheel_Effort(0));
	  this->front_left_joint->SetForce(0,Set_Velocity_Front_Left_Wheel_Effort(0));
	  this->front_right_joint->SetForce(0,Set_Velocity_Front_Right_Wheel_Effort(0));
	}

	

    }

     // Defining private Pointer to model
    private: physics::ModelPtr model;

     // Defining private Pointer to joints
    private: physics::JointPtr steering_joint;
    private: physics::JointPtr back_left_joint;
    private: physics::JointPtr back_right_joint;
    private: physics::JointPtr front_left_joint;
    private: physics::JointPtr front_right_joint;
     // Defining private Pointer to link
    private: physics::LinkPtr link;

     // Defining private Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Defining private Ros Node Handle
    private: ros::NodeHandle n;
    
    // Defining private Ros Subscribers
    private: ros::Subscriber Steering_rate_sub;
    private: ros::Subscriber Velocity_rate_sub;
    
    // Defining private Timers
    private: common::Timer steering_timer;
    private: common::Timer velocity_timer;

    // Defining private Reference Holders
    private: float Angular_velocity_ref;
    private: float Linear_velocity_ref;
    private: float Angular_velocity_function;
    private: float Linear_velocity_function;
    // Defining private Mutex
    private: boost::mutex Angular_velocity_ref_mutex;
    private: boost::mutex Linear_velocity_ref_mutex;
    private: boost::mutex Linear_velocity_function_mutex;
    private: boost::mutex Angular_velocity_function_mutex;



	private: float left_velocity_function(float linear_rate,float angular_rate)
	{
		float left_rate_output,linear_rate_private,angular_rate_private;
		linear_rate_private=linear_rate;
		angular_rate_private=angular_rate;
		
		
		//TODO: map from the matlab function to left_rate_output 
		
		left_rate_output=linear_rate_private-angular_rate_private*wide;
		return left_rate_output;
	}
	private: float right_velocity_function(float linear_rate,float angular_rate)
	{
		float right_rate_output,linear_rate_private,angular_rate_private;
		linear_rate_private=linear_rate;
		angular_rate_private=angular_rate;
		
		
		//TODO: map from the matlab function to right_rate_output 
		
		right_rate_output=linear_rate_private+angular_rate_private*wide;
		return right_rate_output;
	}
	
	
	// The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
	private: void On_Angular_Msg(const std_msgs::Float64ConstPtr &msg)
	{
	  Angular_velocity_ref_mutex.lock();
		  // Recieving referance steering angle
		  
		  Angular_velocity_ref=msg->data;
		  if(msg->data>1) Angular_velocity_ref=1;
		  if(msg->data<-1) Angular_velocity_ref=-1;
		
		  // Reseting timer every time LLC publishes message
		  steering_timer.Start();
	  Angular_velocity_ref_mutex.unlock();
	}

	// The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
	private: void On_Velocity_Msg(const std_msgs::Float64ConstPtr &msg)
	{
	  Linear_velocity_ref_mutex.lock();
		  // Recieving referance hammer velocity
		  Linear_velocity_ref=msg->data;
		  if(msg->data>1) Linear_velocity_ref=1;
		  if(msg->data<-1) Linear_velocity_ref=-1;
		  		  
		  // Reseting timer every time LLC publishes message
		  velocity_timer.Start();
	  Linear_velocity_ref_mutex.unlock();

	}



	// this function sets the efforts given to the hammer wheels according to error getting to refarance velocity, effort inserted via wheel_joints	
	// if brake command is recieved refarance change to 0
	private: float Set_Velocity_Back_Left_Wheel_Effort(int brake)
	{
		Linear_velocity_function_mutex.lock();
			float error,effort=0;
			if (brake)
				Linear_velocity_ref=0;
			error = (left_velocity_function(Linear_velocity_ref*Ke,Angular_velocity_ref*Ke)-(this->back_left_joint->GetVelocity(0)));
			effort = Kp*error;
		Linear_velocity_function_mutex.unlock();
		return effort;
	}

	private: float Set_Velocity_Back_Right_Wheel_Effort(int brake)
	{
		Linear_velocity_function_mutex.lock();
			float error,effort=0;
			if (brake)
				Linear_velocity_ref=0;
			error = (right_velocity_function(Linear_velocity_ref*Ke,Angular_velocity_ref*Ke)-(this->back_right_joint->GetVelocity(0)));
			effort = Kp*error;
		Linear_velocity_function_mutex.unlock();
		return effort;
	}

	private: float Set_Velocity_Front_Left_Wheel_Effort(int brake)
	{
		Linear_velocity_function_mutex.lock();
			float error,effort=0;
			if (brake)
				Linear_velocity_ref=0;
			error = (left_velocity_function(Linear_velocity_ref*Ke,Angular_velocity_ref*Ke)-(this->front_left_joint->GetVelocity(0)));
			effort = Kp*error;
		Linear_velocity_function_mutex.unlock();
		return effort;
	}

	private: float Set_Velocity_Front_Right_Wheel_Effort(int brake)
	{
		Linear_velocity_function_mutex.lock();
			float error,effort=0;
			if (brake)
				Linear_velocity_ref=0;
			error = (right_velocity_function(Linear_velocity_ref*Ke,Angular_velocity_ref*Ke)-(this->front_right_joint->GetVelocity(0)));
			effort = Kp*error;
		Linear_velocity_function_mutex.unlock();
		return effort;
	}

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(bobtankDrivePlugin)
}
#endif
