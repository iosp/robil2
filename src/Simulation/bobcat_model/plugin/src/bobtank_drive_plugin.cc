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
#include <gazebo/math/gzmath.hh>
#include <stdio.h>
#include <math.h> 

// ROS Communication
#include "ros/ros.h"

// Maximum time delays
#define velocity_message_max_time_delay 0.03
#define steering_message_max_time_delay 0.03

// PID - Gain Values
#define Kp 1000
#define Klin 1
#define Kang 1
#define wide 1.5

// coefficients for linear surface
#define l0 -0.09307
#define l1 5.927*pow(10,-19)
#define l2 0.005458
#define l3 8.708*pow(10,-5)
#define l4 -5.349*pow(10,-21)
#define l5 1.676*pow(10,-6)
#define l6 6.486*pow(10,-9)
#define l7 -1.468*pow(10,-22)
#define l8 9.182*pow(10,-7)

// coefficients for angular surface
#define a0 -2.258*pow(10,-17)
#define a1 -0.00206
#define a2 7.758*pow(10,-19)
#define a3 6.377*pow(10,-21)
#define a4 -5.738*pow(10,-21)
#define a5 2.299*pow(10,-21)
#define a6 1.462*pow(10,-6)
#define a7 -4.933*pow(10,-22)
#define a8 4.91*pow(10,-7)
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
      this->body_link = this->model->GetLink("body");
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
    private: physics::LinkPtr body_link;
     // Defining private Pointer to link
    private: physics::LinkPtr link;
    private: math::Vector3 lin_v;
    private: math::Vector3 ang_v;

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


		double x=angular_rate;
		double y=linear_rate;

		//TODO: map from the matlab function to left_rate_output
		double vel= l0 + l1*x + l2*y + l3*pow(x,2) + l4*x*y + l5*pow(y,2) + l6*pow(x,2)*y + l7*x*pow(y,2) + l8*pow(y,3);
		double ang= a0 + a1*x + a2*y + a3*pow(x,2) + a4*x*y + a5*pow(y,2) + a6*pow(x,3) + a7*pow(x,2)*y + a8*x*pow(y,2);

		linear_rate_private=vel;
		angular_rate_private=ang;
		//ROS_INFO("vel=%lf     ang=%lf",vel,ang);
		lin_v=this->body_link-> GetRelativeLinearVel(); // get velocity in gazebo frame
	    ang_v=this->body_link-> GetRelativeAngularVel(); // get velocity in gazebo frame

		ROS_INFO("body_link=%lf         body_ang=%lf",lin_v.x,ang_v.z);
		   
		
		left_rate_output=linear_rate_private-angular_rate_private*wide;
		return left_rate_output;
	}
	private: float right_velocity_function(float linear_rate,float angular_rate)
	{
		float right_rate_output,linear_rate_private,angular_rate_private;


		double x=angular_rate;
		double y=linear_rate;

		//TODO: map from the matlab function to right_rate_output 
		double vel= l0 + l1*x + l2*y + l3*pow(x,2) + l4*x*y + l5*pow(y,2) + l6*pow(x,2)*y + l7*x*pow(y,2) + l8*pow(y,3);
		double ang= a0 + a1*x + a2*y + a3*pow(x,2) + a4*x*y + a5*pow(y,2) + a6*pow(x,3) + a7*pow(x,2)*y + a8*x*pow(y,2);

		linear_rate_private=vel;
		angular_rate_private=ang;
		
		
		right_rate_output=linear_rate_private+angular_rate_private*wide;
		return right_rate_output;
	}
	
	
	// The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
	private: void On_Angular_Msg(const std_msgs::Float64ConstPtr &msg)
	{
	  Angular_velocity_ref_mutex.lock();
		  // Recieving referance steering angle
		  
		  Angular_velocity_ref=100*msg->data;
		  if(msg->data>100) Angular_velocity_ref=100;
		  if(msg->data<-100) Angular_velocity_ref=-100;
		
		  // Reseting timer every time LLC publishes message
		  steering_timer.Start();
	  Angular_velocity_ref_mutex.unlock();
	}

	// The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
	private: void On_Velocity_Msg(const std_msgs::Float64ConstPtr &msg)
	{
	  Linear_velocity_ref_mutex.lock();
		  // Recieving referance hammer velocity
		  Linear_velocity_ref=100*msg->data;
		  if(msg->data>100) Linear_velocity_ref=100;
		  if(msg->data<-100) Linear_velocity_ref=-100;
		  		  
		  // Reseting timer every time LLC publishes message
		  velocity_timer.Start();
	  Linear_velocity_ref_mutex.unlock();

	}



	// this function sets the efforts given to the hammer wheels according to error getting to refarance velocity, effort inserted via wheel_joints	
	// if brake command is recieved refarance change to 0
	private: float Set_Velocity_Back_Left_Wheel_Effort(int brake)
	{
		lin_v=this->body_link-> GetRelativeLinearVel(); // get velocity in gazebo frame
	    ang_v=this->body_link-> GetRelativeAngularVel(); // get velocity in gazebo frame
		double left_vel;
		left_vel= lin_v.x  -  ang_v.z*wide/2;
		Linear_velocity_function_mutex.lock();
			float error,effort=0;
			if (brake)
				Linear_velocity_ref=0;
			error = (left_velocity_function(Linear_velocity_ref*Klin,Angular_velocity_ref*Kang)-  left_vel     );
			effort = Kp*error;
		Linear_velocity_function_mutex.unlock();
		return effort;
	}

	private: float Set_Velocity_Back_Right_Wheel_Effort(int brake)
	{
		lin_v=this->body_link-> GetRelativeLinearVel(); // get velocity in gazebo frame
	    ang_v=this->body_link-> GetRelativeAngularVel(); // get velocity in gazebo frame
		double right_vel;
		right_vel= lin_v.x  +  ang_v.z*wide/2;
		Linear_velocity_function_mutex.lock();
			float error,effort=0;
			if (brake)
				Linear_velocity_ref=0;
			error = (right_velocity_function(Linear_velocity_ref*Klin,Angular_velocity_ref*Kang)-    right_vel    );
			effort = Kp*error;
		Linear_velocity_function_mutex.unlock();
		return effort;
	}

	private: float Set_Velocity_Front_Left_Wheel_Effort(int brake)
	{
		lin_v=this->body_link-> GetRelativeLinearVel(); // get velocity in gazebo frame
	    ang_v=this->body_link-> GetRelativeAngularVel(); // get velocity in gazebo frame
		double left_vel;
		left_vel= lin_v.x  -  ang_v.z*wide/2;
		Linear_velocity_function_mutex.lock();
			float error,effort=0;
			if (brake)
				Linear_velocity_ref=0;
			error = (left_velocity_function(Linear_velocity_ref*Klin,Angular_velocity_ref*Kang)-  left_vel     );
			effort = Kp*error;
		Linear_velocity_function_mutex.unlock();
		return effort;
	}

	private: float Set_Velocity_Front_Right_Wheel_Effort(int brake)
	{
		lin_v=this->body_link-> GetRelativeLinearVel(); // get velocity in gazebo frame
	    ang_v=this->body_link-> GetRelativeAngularVel(); // get velocity in gazebo frame
		double right_vel;
		right_vel= lin_v.x  +  ang_v.z*wide/2;
		Linear_velocity_function_mutex.lock();
			float error,effort=0;
			if (brake)
				Linear_velocity_ref=0;
			error = (right_velocity_function(Linear_velocity_ref*Klin,Angular_velocity_ref*Kang)-    right_vel    );
			effort = Kp*error;
		Linear_velocity_function_mutex.unlock();
		return effort;
	}

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(bobtankDrivePlugin)
}
#endif
