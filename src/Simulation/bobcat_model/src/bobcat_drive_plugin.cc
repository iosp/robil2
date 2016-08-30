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

// Dynamic Configuration
#include <dynamic_reconfigure/server.h>
#include <bobcat_model/bobcat_modelConfig.h>


#include "std_msgs/Bool.h"

// Maximum time delays
#define command_MAX_DELAY 0.03
//#define steering_message_max_time_delay 0.03

// PID - Gain Values
#define Kp 1200
#define Klin 1
#define Kang 1

#define WHEEL_EFFORT_LIMIT 1200

#define PLAT_WIDE 1.5
#define WHEEL_DIAMETER 0.77
#define PI 3.14159265359

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
      this->back_left_joint   = this->model->GetJoint("back_left_wheel_joint");
      this->back_right_joint  = this->model->GetJoint("back_right_wheel_joint");
      this->front_left_joint  = this->model->GetJoint("front_left_wheel_joint");
      this->front_right_joint = this->model->GetJoint("front_right_wheel_joint");
      
      // Starting Timers
      command_timer.Start();

      this->Ros_nh = new ros::NodeHandle("bobtankDrivePlugin_node");

      // Subscribe to the topic, and register a callback
      Steering_rate_sub = this->Ros_nh->subscribe("/LLC/EFFORTS/Steering" , 1000, &bobtankDrivePlugin::On_Angular_command, this);
      Velocity_rate_sub = this->Ros_nh->subscribe("/LLC/EFFORTS/Throttle" , 1000, &bobtankDrivePlugin::On_Linear_command, this);
      
      platform_hb_pub_ = this->Ros_nh->advertise<std_msgs::Bool>("/Sahar/link_with_platform" , 100);

      // Listen to the update event. This event is broadcast every simulation iteration. 
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&bobtankDrivePlugin::OnUpdate, this, _1));

      this->model_reconfiguration_server = new dynamic_reconfigure::Server<bobcat_model::bobcat_modelConfig> (*(this->Ros_nh));
      this->model_reconfiguration_server->setCallback(boost::bind(&bobtankDrivePlugin::dynamic_Reconfiguration_callback, this, _1, _2));
    }


   public: void dynamic_Reconfiguration_callback(bobcat_model::bobcat_modelConfig &config, uint32_t level)
      {
          cP = config.Wheel_conntrol_P;
          cI = config.Wheel_conntrol_I;
          cD = config.Wheel_conntrol_D;
      }




    // Called by the world update start event, This function is the event that will be called every update
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)  // we are not using the pointer to the info so its commanted as an option
    {
            // Applying effort to the wheels , brakes if no message income
            if (command_timer.GetElapsed().Float()> command_MAX_DELAY)
            {
                // Brakes
                   Linear_ref_vel = 0;
                   Angular_ref_vel = 0;
            }

              update_ref_vels();

              apply_efforts();


              std_msgs::Bool connection;
              connection.data = true;
              platform_hb_pub_.publish(connection);
    }

    

    private: void update_ref_vels() // float linear_command, float angular_command)
    {
        Angular_command_mutex.lock();
           float x = Angular_command;
        Angular_command_mutex.unlock();

        Linear_command_mutex.lock();
           float y = Linear_command;
        Linear_command_mutex.unlock();

        Linear_ref_vel = y; //( l0 + l1*x + l2*y + l3*pow(x,2) + l4*x*y + l5*pow(y,2) + l6*pow(x,2)*y + l7*x*pow(y,2) + l8*pow(y,3)+0.1 )*1.15;
        Angular_ref_vel = x; //a0 + a1*x + a2*y + a3*pow(x,2) + a4*x*y + a5*pow(y,2) + a6*pow(x,3) + a7*pow(x,2)*y + a8*x*pow(y,2);
    }


   private: void wheel_controller(physics::JointPtr wheel_joint, double ref_omega)
   {
        double wheel_omega = wheel_joint->GetVelocity(0);

        double error = ref_omega - wheel_omega;

        double effort_command = cP * error;

        if(effort_command > WHEEL_EFFORT_LIMIT) effort_command = WHEEL_EFFORT_LIMIT;
        if(effort_command < -WHEEL_EFFORT_LIMIT) effort_command = -WHEEL_EFFORT_LIMIT;


        std::cout << " wheel_joint->GetName() = " << wheel_joint->GetName() << std::endl;
        std::cout << "           ref_omega = " << ref_omega << " wheel_omega = " << wheel_omega  << " error = " << error << " effort_command = " << effort_command <<  std::endl;
        //std::cout << "           Pl = " << Pl << std::endl;

        wheel_joint->SetForce(0,effort_command);
    }


  private: void apply_efforts()
    {

        float right_side_vel = (Linear_ref_vel + Angular_ref_vel * PLAT_WIDE/2);
        float left_side_vel = (Linear_ref_vel - Angular_ref_vel * PLAT_WIDE/2);

        std::cout << " right_side_vel = " << right_side_vel <<  " left_side_vel = " << left_side_vel << std::endl;

        float rigth_wheels_omega_ref = right_side_vel / (0.5 * WHEEL_DIAMETER);
        float left_wheels_omega_ref = left_side_vel / (0.5 * WHEEL_DIAMETER);

        std::cout << " rigth_wheels_omega_ref = " << rigth_wheels_omega_ref <<  " left_wheels_omega_ref = " << left_wheels_omega_ref << std::endl;



        wheel_controller(this->front_right_joint, rigth_wheels_omega_ref);
        wheel_controller(this->back_right_joint , rigth_wheels_omega_ref);
        wheel_controller(this->front_left_joint , left_wheels_omega_ref);
        wheel_controller(this->back_left_joint  , left_wheels_omega_ref);
    }


    // The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
    private: void On_Angular_command(const std_msgs::Float64ConstPtr &msg)
    {
      Angular_command_mutex.lock();
          // Recieving referance steering angle

          Angular_command=100*msg->data;
          if(msg->data>1) Angular_command=100;
          if(msg->data<-1) Angular_command=-100;

          // Reseting timer every time LLC publishes message
          command_timer.Start();
      Angular_command_mutex.unlock();
    }


    // The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
    private: void On_Linear_command(const std_msgs::Float64ConstPtr &msg)
    {
      Linear_command_mutex.lock();
          // Recieving referance hammer velocity
          Linear_command=100*msg->data;
          if(msg->data>1) Linear_command=100;
          if(msg->data<-1) Linear_command=-100;

          // Reseting timer every time LLC publishes message
          command_timer.Start();
      Linear_command_mutex.unlock();

    }


     // Defining private Pointer to model
     private: physics::ModelPtr model;

      // Defining private Pointer to joints
     private: physics::JointPtr steering_joint;
     private: physics::JointPtr back_left_joint;
     private: physics::JointPtr back_right_joint;
     private: physics::JointPtr front_left_joint;
     private: physics::JointPtr front_right_joint;



      // Defining private Pointer to the update event connection
     private: event::ConnectionPtr updateConnection;

     // Defining private Ros Node Handle
     private: ros::NodeHandle  *Ros_nh;

     // Defining private Ros Subscribers
     private: ros::Subscriber Steering_rate_sub;
     private: ros::Subscriber Velocity_rate_sub;

     // Defining private Ros Publishers
     ros::Publisher platform_hb_pub_;


     // Defining private Timers
     private: common::Timer command_timer;



     // Defining private Mutex
     private: boost::mutex Angular_command_mutex;
     private: boost::mutex Linear_command_mutex;


     private: float Linear_command;
     private: float Angular_command;
     private: double Linear_ref_vel;
     private: double Angular_ref_vel;

     private: dynamic_reconfigure::Server<bobcat_model::bobcat_modelConfig> *model_reconfiguration_server;
     private: double cP, cI ,cD;		/* PID constants */


  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(bobtankDrivePlugin)
}
#endif
