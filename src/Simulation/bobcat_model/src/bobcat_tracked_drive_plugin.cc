// Written By : Daniel Meltz, Adapted for the tracked model by: Yossi Cohen.

// If the plugin is not defined then define it
#ifndef _BOBTANK_DRIVE_PLUGIN_HH_
#define _BOBTANK_DRIVE_PLUGIN_HH_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <random>

// Gazebo Libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/gazebo_config.h>

// ROS Communication
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

// Boost Thread Mutex
#include <boost/thread/mutex.hpp>

// Dynamic Configuration
#include <dynamic_reconfigure/server.h>
#include <bobcat_model/bobcat_modelConfig.h>
#include <boost/bind.hpp> // Boost Bind

// Interpolation
#include <ctime>
#include "linterp.h"

#include "sim_qinetiq_client.cc"

// Maximum time delays
#define command_MAX_DELAY 0.3

#define WHEEL_EFFORT_LIMIT 1000

#define WHEELS_BASE 0.95
#define STEERING_FRICTION_COMPENSATION 2; // compensate for the faliure of reaching the angular velocity

#define WHEEL_DIAMETER 0.4
#define PI 3.14159265359

#define LINEAR_COMMAND_FILTER_ARRY_SIZE 750
#define ANGULAR_COMMAND_FILTER_ARRY_SIZE 500

namespace gazebo
{

class bobtankDrivePlugin : public ModelPlugin
{
    ///  Constructor
  public:
    bobtankDrivePlugin() {}

    /// The load function is called by Gazebo when the plugin is inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) // we are not using the pointer to the sdf file so its commanted as an option
    {
        std::cout << "MY_GAZEBO_VER = [" << GAZEBO_MAJOR_VERSION << "]" << std::endl;

        // Store the pointer to the model
        this->model = _model;

        // Store the pointers to the joints
        this->back_left_joint = this->model->GetJoint("back_left_wheel_joint");
        this->back_right_joint = this->model->GetJoint("back_right_wheel_joint");
        this->front_left_joint = this->model->GetJoint("front_left_wheel_joint");
        this->front_right_joint = this->model->GetJoint("front_right_wheel_joint");
        this->roller_back_right = this->model->GetJoint("roller_back_right_joint");
        this->roller_mid_right = this->model->GetJoint("roller_mid_right_joint");
        this->roller_front_right = this->model->GetJoint("roller_front_right_joint");
        this->roller_back_left = this->model->GetJoint("roller_back_left_joint");
        this->roller_mid_left = this->model->GetJoint("roller_mid_left_joint");
        this->roller_front_left = this->model->GetJoint("roller_front_left_joint");
        this->cogwheel_right = this->model->GetJoint("cogwheel_right_joint");
        this->cogwheel_left = this->model->GetJoint("cogwheel_left_joint");

        this->Ros_nh = new ros::NodeHandle("bobtankDrivePlugin_node");

        platform_hb_pub_ = this->Ros_nh->advertise<std_msgs::Bool>("/Sahar/link_with_platform", 100);

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&bobtankDrivePlugin::OnUpdate, this, _1));

        this->model_reconfiguration_server = new dynamic_reconfigure::Server<bobcat_model::bobcat_modelConfig>(*(this->Ros_nh));
        this->model_reconfiguration_server->setCallback(boost::bind(&bobtankDrivePlugin::dynamic_Reconfiguration_callback, this, _1, _2));

        /* initialize random seed: */
        srand(time(NULL));
        this->Linear_Noise_dist = new std::normal_distribution<double>(0, 1);
        this->Angular_Noise_dist = new std::normal_distribution<double>(0, 1);

        calibration_data_setup();

        Linear_command_sum = 0;
        Angular_command_sum = 0;
        Linear_command_index = 0;
        Angular_command_index = 0;
        for (int i = 0; i < LINEAR_COMMAND_FILTER_ARRY_SIZE; i++)
        {
            Linear_command_array[i] = 0;
        }

        for (int i = 0; i < ANGULAR_COMMAND_FILTER_ARRY_SIZE; i++)
        {
            Angular_command_array[i] = 0;
        }

          sqc.Init("127.0.0.1", 4660, 5355);
    }

    void calibration_data_setup()
    {
        // construct the grid in each dimension.
        // note that we will pass in a sequence of iterators pointing to the beginning of each grid
        double Throttle_commands_array[] = {-1.00, -0.70, -0.40, 0.00, 0.40, 0.70, 1.00};

        double Sttering_commands_array[] = {-1.00, -0.70, -0.40, 0.00, 0.40, 0.70, 1.00};
        //                                s=-1.00 s=-0.70 s=-0.40 s=0 s=0.40  s=0.70  s=1.00
        double Linear_vel_values_array[] = {-1.20, -1.30, -1.40, -1.50, -1.40, -1.30, -1.20, //t=-1.00
                                            -0.65, -0.70, -0.75, -0.80, -0.75, -0.70, -0.65, //t=-0.70
                                            -0.14, -0.16, -0.18, -0.20, -0.18, -0.16, -0.14, //t=-0.40
                                            0.40, 0.17, 0.00, 0.00, 0.00, 0.17, 0.40,        //t=0.00
                                            0.14, 0.16, 0.18, 0.20, 0.18, 0.16, 0.14,        //t=0.40
                                            0.65, 0.70, 0.75, 0.80, 0.75, 0.70, 0.65,        //t=0.70
                                            1.20, 1.30, 1.40, 1.50, 1.40, 1.30, 1.20};       //t=1.00

        //                                s=-1.00 s=-0.70 s=-0.40 s=0  s=0.40 s=0.70  s=1.00
        double Angular_vel_values_array[] = {-1.40, -0.42, -0.18, 0.00, 0.18, 0.42, 1.40,  //t=-1.00
                                             -1.36, -0.38, -0.14, 0.00, 0.14, 0.38, 1.36,  //t=-0.70
                                             -1.32, -0.34, -0.10, 0.00, 0.10, 0.34, 1.32,  //t=-0.40
                                             -1.22, -0.24, -0.00, 0.00, 0.00, 0.24, 1.22,  //t=0.00
                                             -1.32, -0.34, -0.10, 0.00, 0.10, 0.34, 1.32,  //t=0.40
                                             -1.36, -0.38, -0.14, 0.00, 0.14, 0.38, 1.36,  //t=0.70
                                             -1.40, -0.42, -0.18, 0.00, 0.18, 0.42, 1.40}; //t=1.00

        std::vector<double> Throttle_commands(Throttle_commands_array, Throttle_commands_array + sizeof(Throttle_commands_array) / sizeof(double));
        std::vector<double> Sttering_commands(Sttering_commands_array, Sttering_commands_array + sizeof(Sttering_commands_array) / sizeof(double));
        std::vector<double> Linear_vel_values(Linear_vel_values_array, Linear_vel_values_array + sizeof(Linear_vel_values_array) / sizeof(double));
        std::vector<double> Angular_vel_values(Angular_vel_values_array, Angular_vel_values_array + sizeof(Angular_vel_values_array) / sizeof(double));

        std::vector<std::vector<double>::iterator> grid_iter_list;
        grid_iter_list.push_back(Throttle_commands.begin());
        grid_iter_list.push_back(Sttering_commands.begin());

        // the size of the grid in each dimension
        array<int, 2> grid_sizes;
        grid_sizes[0] = Throttle_commands.size();
        grid_sizes[1] = Sttering_commands.size();

        // total number of elements
        int num_elements = grid_sizes[0] * grid_sizes[1];

        // construct the interpolator. the last two arguments are pointers to the underlying data
        Linear_vel_interp = new InterpMultilinear<2, double>(grid_iter_list.begin(), grid_sizes.begin(), Linear_vel_values.data(), Linear_vel_values.data() + num_elements);
        Angular_vel_interp = new InterpMultilinear<2, double>(grid_iter_list.begin(), grid_sizes.begin(), Angular_vel_values.data(), Angular_vel_values.data() + num_elements);
    }

  public:
    void dynamic_Reconfiguration_callback(bobcat_model::bobcat_modelConfig &config, uint32_t level)
    {
        Power = config.Power;
        MinAngMult = config.MinAngMult;           //Rotation speed multiplier on minimal angular velocity.
        MaxAngMult = config.MaxAngMult;           //Rotation speed multiplier on maximal angular velocity.
        MinAngPowerMult = config.MinAngPowerMult; //Control power multiplier on minimal angular velocity.
        MaxAngPowerMult = config.MaxAngPowerMult; //Control power multiplier on maximal angular velocity.
        command_lN = config.Command_Linear_Noise;
        command_aN = config.Command_Angular_Noise;
    }

    // Called by the world update start event, This function is the event that will be called every update
  public:
    void OnUpdate(const common::UpdateInfo & /*_info*/) // we are not using the pointer to the info so its commanted as an option
    {
        update_ref_vels();
        apply_efforts();

        std_msgs::Bool connection;
        connection.data = true;
        platform_hb_pub_.publish(connection);
    }

    double command_fillter(double prev_commands_array[], int array_size, double &commmand_sum, int &command_index, double command)
    {
        commmand_sum -= prev_commands_array[command_index];
        commmand_sum += command;

        prev_commands_array[command_index] = command;

        if (++command_index == array_size)
            command_index = 0;

        double filtered_command = commmand_sum / array_size;

        return (filtered_command);
    }

  private:
    void update_ref_vels() // float linear_command, float angular_command)
    {
        Linear_command_mutex.lock();
        Angular_command_mutex.lock();
        array<double, 2> args = {sqc.getThrottel(), sqc.getSteering()};
        Linear_command_mutex.unlock();
        Angular_command_mutex.unlock();

        double Linear_nominal_vel = Linear_vel_interp->interp(args.begin());
        double Angular_nominal_vel = Angular_vel_interp->interp(args.begin());

        double Linear_vel = command_fillter(Linear_command_array, LINEAR_COMMAND_FILTER_ARRY_SIZE, Linear_command_sum, Linear_command_index, Linear_nominal_vel);
        double Angular_vel = command_fillter(Angular_command_array, ANGULAR_COMMAND_FILTER_ARRY_SIZE, Angular_command_sum, Angular_command_index, Angular_nominal_vel);

        double LinearNoise = command_lN * (*Linear_Noise_dist)(generator);
        double AngularNoise = command_aN * (*Angular_Noise_dist)(generator);

        Linear_ref_vel = (1 + LinearNoise) * Linear_vel;
        Angular_ref_vel = (1 + AngularNoise) * Angular_vel;
    }

  private:
    void wheel_controller(physics::JointPtr wheel_joint, double ref_omega)
    {
        double wheel_omega = wheel_joint->GetVelocity(0);

        double error = ref_omega - wheel_omega;
        //  Compensating for friction using a value that is between MinAngPowerMult(higher) and MaxAngPowerMult(lower)
        //  so a more strict control is achieved on low rotation speeds where achieving the rotation value is harder.
        double FrictionCompensationPowerMultiplier = (1.4 * MinAngPowerMult - (MinAngPowerMult - MaxAngPowerMult) * fabs(Angular_ref_vel)) / 1.4;

        effort_command = Power * FrictionCompensationPowerMultiplier * error;
        if (ref_omega == 0)
            effort_command = effort_command - 900 * wheel_omega;
        if (effort_command > WHEEL_EFFORT_LIMIT)
            effort_command = WHEEL_EFFORT_LIMIT;
        if (effort_command < -WHEEL_EFFORT_LIMIT)
            effort_command = -WHEEL_EFFORT_LIMIT;
        if (wheel_joint == this->cogwheel_right || wheel_joint == this->cogwheel_left)
            effort_command = effort_command * 0.001;
        //        std::cout << " wheel_joint->GetName() = " << wheel_joint->GetName() << std::endl;
               std::cout << "ref_omega = " << ref_omega << " wheel_omega = " << wheel_omega  << " error = " << error << " effort_command = " << effort_command <<  std::endl;
        wheel_joint->SetForce(0, effort_command);
    }  

    private:
    void apply_efforts()
    {

        //std::cout << " Linear_ref_vel = " << Linear_ref_vel << " Angular_ref_vel = " << Angular_ref_vel << std::endl;
        // float right_side_vel = ( Linear_ref_vel ) + (Angular_ref_vel* WHEELS_BASE/2) ;
        // float left_side_vel  = ( Linear_ref_vel ) - (Angular_ref_vel * WHEELS_BASE/2) ;
        //Compensating for Real target Rotation speeds MinAngMult(higher) and MaxAngMult(lower)
        float RealAngularSpeedCompensation = (1.22 * MinAngMult - (MinAngMult - MaxAngMult) * fabs(Angular_ref_vel)) / 1.22;
        float right_side_vel = Linear_ref_vel + Angular_ref_vel * RealAngularSpeedCompensation * WHEELS_BASE / 2;
        float left_side_vel = Linear_ref_vel - Angular_ref_vel * RealAngularSpeedCompensation * WHEELS_BASE / 2;
        //std::cout << " right_side_vel = " << right_side_vel <<  " left_side_vel = " << left_side_vel << std::endl;

        float right_wheels_omega_ref = right_side_vel / (0.5 * WHEEL_DIAMETER);
        float left_wheels_omega_ref = left_side_vel / (0.5 * WHEEL_DIAMETER);

        // std::cout << " right_wheels_omega_ref = " << right_wheels_omega_ref <<  " left_wheels_omega_ref = " << left_wheels_omega_ref << std::endl;
        
        wheel_controller(this->front_right_joint, right_wheels_omega_ref);
        wheel_controller(this->back_right_joint, right_wheels_omega_ref);
        wheel_controller(this->front_left_joint, left_wheels_omega_ref);
        wheel_controller(this->back_left_joint, left_wheels_omega_ref);
        wheel_controller(this->roller_back_right,  right_wheels_omega_ref);
        wheel_controller(this->roller_mid_right,   right_wheels_omega_ref);
        wheel_controller(this->roller_front_right, right_wheels_omega_ref);
        wheel_controller(this->roller_back_left,   left_wheels_omega_ref);
        wheel_controller(this->roller_mid_left,    left_wheels_omega_ref);
        wheel_controller(this->roller_front_left,  left_wheels_omega_ref);
        // wheel_joint->SetVelocity(0,ref_omega);
        wheel_controller(this->cogwheel_right, right_wheels_omega_ref);
        wheel_controller(this->cogwheel_left, left_wheels_omega_ref);
    }


    // Defining private Pointer to model
    physics::ModelPtr model;

    // Defining private Pointer to joints
    physics::JointPtr steering_joint;
    physics::JointPtr back_left_joint;
    physics::JointPtr back_right_joint;
    physics::JointPtr front_left_joint;
    physics::JointPtr front_right_joint;
    physics::JointPtr roller_back_right;
    physics::JointPtr roller_mid_right;
    physics::JointPtr roller_front_right;
    physics::JointPtr roller_back_left;
    physics::JointPtr roller_mid_left;
    physics::JointPtr roller_front_left;
    physics::JointPtr cogwheel_left;
    physics::JointPtr cogwheel_right;

    // Defining private Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Defining private Ros Node Handle
    ros::NodeHandle *Ros_nh;

    // Defining private Ros Publishers
    ros::Publisher platform_hb_pub_;

    // Defining private Mutex
    boost::mutex Angular_command_mutex;
    boost::mutex Linear_command_mutex;

    float Linear_command=0;
    float Angular_command=0;
    double Linear_ref_vel=0;
    double Angular_ref_vel=0;
    double effort_command=0;
    double Linear_command_array[LINEAR_COMMAND_FILTER_ARRY_SIZE];
    double Angular_command_array[ANGULAR_COMMAND_FILTER_ARRY_SIZE];
    double Linear_command_sum;
    double Angular_command_sum;
    int Linear_command_index;
    int Angular_command_index;

    dynamic_reconfigure::Server<bobcat_model::bobcat_modelConfig> *model_reconfiguration_server;

    double Power, MinAngMult, MaxAngMult, MinAngPowerMult, MaxAngPowerMult; // PID constants and dynamic configuration constants
    double command_lN, command_aN; // command noise factors

    std::default_random_engine generator;
    std::normal_distribution<double> *Linear_Noise_dist;
    std::normal_distribution<double> *Angular_Noise_dist;

    InterpMultilinear<2, double> *Linear_vel_interp;
    InterpMultilinear<2, double> *Angular_vel_interp;
    
    simQinetiqClient sqc;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(bobtankDrivePlugin)
}
#endif
