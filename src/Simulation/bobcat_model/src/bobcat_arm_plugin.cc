// Written By : Daniel Meltz

// If the plugin is not defined then define it
#ifndef _bobcat_DRIVE_PLUGIN_HH_
#define _bobcat_DRIVE_PLUGIN_HH_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// Gazebo Libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include "ros/ros.h"
#include "std_msgs/Float64.h"
// Boost Thread Mutex
#include <boost/thread/mutex.hpp>

// Dynamic Configuration
#include <dynamic_reconfigure/server.h>
#include <bobcat_model/bobcat_armConfig.h>
#include <boost/bind.hpp> // Boost Bind

#define PI 3.14159265359
#define command_MAX_DELAY 0.1
namespace gazebo
{

class bobcatArmPlugin : public ModelPlugin
{
    double deltaSimTime = 0.001;
    ///  Constructor
  public:
    bobcatArmPlugin() {}

    /// The load function is called by Gazebo when the plugin is inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) // we are not using the pointer to the sdf file so its commanted as an option
    {
        std::cout << "MY_GAZEBO_VER = [" << GAZEBO_MAJOR_VERSION << "]" << std::endl;
            Hydraulics_command_timer.Start();
    Loader_command_timer.Start();
    Brackets_command_timer.Start();
        // Store the pointer to the model
        this->model = _model;

        // Store the pointers to the joints

        this->Loader = this->model->GetJoint("loader_joint");
        this->Hydraulics = this->model->GetJoint("Hydraulics_joint");
        this->Brackets = this->model->GetJoint("brackets_joint");

        this->Ros_nh = new ros::NodeHandle("bobcatArmPlugin_node");
        Hydraulics_sub = this->Ros_nh->subscribe("/bobcat/arm/hydraulics", 60, &bobcatArmPlugin::On_Hydraulics_command, this);
        Loader_sub = this->Ros_nh->subscribe("/bobcat/arm/loader", 60, &bobcatArmPlugin::On_Loader_command, this);
        Brackets_sub = this->Ros_nh->subscribe("/bobcat/arm/brackets", 60, &bobcatArmPlugin::On_Brackets_command, this);

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&bobcatArmPlugin::OnUpdate, this, _1));

        this->model_reconfiguration_server = new dynamic_reconfigure::Server<bobcat_model::bobcat_armConfig>(*(this->Ros_nh));
        this->model_reconfiguration_server->setCallback(boost::bind(&bobcatArmPlugin::dynamic_Reconfiguration_callback, this, _1, _2));
    }
double clampValue(double ValueToNormalize,double lim) //A normalizing function that clips the value to -1,1 range
{
  double value=0;
  if (ValueToNormalize > lim)
    value = lim;
  else if (ValueToNormalize < -lim)
    value = -lim;
  else
    value = ValueToNormalize;
	return value;
}
  public:
    void dynamic_Reconfiguration_callback(bobcat_model::bobcat_armConfig &config, uint32_t level)
    {
        J_P = config.joint_control_P;
        J_I = config.joint_control_I;
        J_D = config.joint_control_D;
        H_P = config.hydraulics_control_P;
        H_I = config.hydraulics_control_I;
        H_D = config.hydraulics_control_D;
        HydraulicSpeed = config.HydraulicSpeed;
        JointSpeed = config.JointSpeed;
    }

    // Called by the world update start event, This function is the event that will be called every update
  public:
    void OnUpdate(const common::UpdateInfo &simInfo) // we are not using the pointer to the info so its commanted as an option
    {
        deltaSimTime = simInfo.simTime.Double() - simTime.Double();
        simTime = simInfo.simTime;
        // std::cout << "update function started"<<std::endl;
        // std::cout << "command_timer = " << command_timer.GetElapsed().Float() << std::endl;
        // Resets if no message income
        if (Hydraulics_command_timer.GetElapsed().Float() > command_MAX_DELAY)
        {
            // Applies 0 throtle
            Hydraulics_command = 0;
        }
        if (Loader_command_timer.GetElapsed().Float() > command_MAX_DELAY)
        {
            // Restores straight direction.
            Loader_command = 0;
        }
        if (Brackets_command_timer.GetElapsed().Float() > command_MAX_DELAY)
        {
            // Restores straight direction.
            Brackets_command = 0;
        }
        //
        CalculateTargets();
        apply_efforts();
    }

  private:
    void CalculateTargets()
    {
// Adding input to the spring-damper control target:
        Hydraulics_target += Hydraulics_command * HydraulicSpeed * deltaSimTime;
        Loader_target += Loader_command * JointSpeed * deltaSimTime;
        Brackets_target += Brackets_command * JointSpeed * deltaSimTime;
// Limits:
        if(Hydraulics_target>0.39)Hydraulics_target=0.39;
        if(Hydraulics_target<-0.1)Hydraulics_target=-0.1;
        if(Loader_target>0.5)Loader_target=0.5;
        if(Loader_target<-0.5)Loader_target=-0.5;
        if(Brackets_target>1)Brackets_target=1;
        if(Brackets_target<-0.5)Brackets_target=-0.5;
    }
    void apply_efforts()
    {
        joint_controller(this->Loader, Loader_target);
        joint_controller(this->Brackets, Brackets_target);
        hydraulics_controller(this->Hydraulics, Hydraulics_target);
    }


    void joint_controller(physics::JointPtr joint, double target)
    {
        // std::cout << " getting angle"<< std::endl;

        double current = joint->GetAngle(0).Radian();
        double currentVel = joint->GetVelocity(0);
        double err=target - current;
         err=clampValue(err,0.05);
         //error is clamped to avoid huge lash-back when the loader is obsructed from moving but the control target is still moving away due to input.
        double jointforce = J_P * (err) - J_D * (currentVel);
        if(joint==this->Brackets) jointforce*=0.1;
        joint->SetForce(0, jointforce);
        //  std::cout << current<< std::endl;
    }
    void hydraulics_controller(physics::JointPtr joint, double target)
    {
        // std::cout << " getting angle"<< std::endl;
        double current = joint->GetAngle(0).Radian();
        // std::cout << current<< std::endl;
        double currentVel = joint->GetVelocity(0);
        double err=target - current;
        //error is clamped to avoid huge lash-back when the loader is obsructed from moving but the control target is still moving away due to input.
        err=clampValue(err,0.02);
        IerH += err;
        double jointforce = H_P * err + H_I * IerH - H_D * (currentVel);
        joint->SetForce(0, jointforce);
    }

    void On_Hydraulics_command(const std_msgs::Float64ConstPtr &msg)
    {
        Hydraulics_command_mutex.lock();
        // Recieving referance velocity
        Hydraulics_command = clampValue(msg->data,1);

// Reseting timer every time LLC publishes message
#if GAZEBO_MAJOR_VERSION >= 5
        Hydraulics_command_timer.Reset();
#endif
        Hydraulics_command_timer.Start();

        Hydraulics_command_mutex.unlock();
    }

    void On_Loader_command(const std_msgs::Float64ConstPtr &msg)
    {
        Loader_command_mutex.lock();
        // Recieving referance velocity
        Loader_command = clampValue(msg->data,1);

// Reseting timer every time LLC publishes message
#if GAZEBO_MAJOR_VERSION >= 5
        Loader_command_timer.Reset();
#endif
        Loader_command_timer.Start();

        Loader_command_mutex.unlock();
}

    void On_Brackets_command(const std_msgs::Float64ConstPtr &msg)
    {
        Brackets_command_mutex.lock();
        // Recieving referance velocity
        Brackets_command = clampValue(msg->data,1);

// Reseting timer every time LLC publishes message
#if GAZEBO_MAJOR_VERSION >= 5
        Brackets_command_timer.Reset();
#endif
        Brackets_command_timer.Start();

        Brackets_command_mutex.unlock();
    }
    // Defining private Pointer to model
    physics::ModelPtr model;

    // Defining private Pointer to joints
    physics::JointPtr Loader;
    physics::JointPtr Hydraulics;
    physics::JointPtr Brackets;

    // Defining private Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Defining Ros
    ros::NodeHandle *Ros_nh;
    ros::Subscriber Hydraulics_sub;
    ros::Subscriber Loader_sub;
    ros::Subscriber Brackets_sub;
    // Defining private Timers
    common::Timer Hydraulics_command_timer;
    common::Timer Loader_command_timer;
    common::Timer Brackets_command_timer;
    common::Time simTime;

    // Defining private Mutex
    boost::mutex Hydraulics_command_mutex;
    boost::mutex Loader_command_mutex;
    boost::mutex Brackets_command_mutex;

    float Hydraulics_command = 0;
    float Loader_command = 0;
    float Brackets_command = 0;
    float Hydraulics_target = 0.075, HydraulicSpeed = 0.1,JointSpeed=0.25, IerH = 0;
    float Loader_target = 0;
    float Brackets_target = 0.5;

    dynamic_reconfigure::Server<bobcat_model::bobcat_armConfig> *model_reconfiguration_server;

    double H_P, H_I, H_D; // PID constants
    double J_P, J_I, J_D;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(bobcatArmPlugin)
}
#endif
