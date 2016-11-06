// Written By : Daniel Meltz
#define MY_GAZEBO_VER 2
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


// Maximum time delays
#define command_MAX_DELAY 0.3

#define WHEEL_EFFORT_LIMIT 1000

#define WHEELS_BASE 0.95
#define STEERING_FRICTION_COMPENSATION 3; // compensate for the faliure of reaching the angular velocity

#define WHEEL_DIAMETER 0.4
#define ROLLER_DIAMETER 0.2
#define PI 3.14159265359

#define LINEAR_COMMAND_FILTER_ARRY_SIZE 750
#define ANGULAR_COMMAND_FILTER_ARRY_SIZE 500


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
      
      // Starting Timers
      Linear_command_timer.Start();
      Angular_command_timer.Start();

      this->Ros_nh = new ros::NodeHandle("bobtankDrivePlugin_node");

      // Subscribe to the topic, and register a callback
      Steering_rate_sub = this->Ros_nh->subscribe("/LLC/EFFORTS/Steering" , 1000, &bobtankDrivePlugin::On_Angular_command, this);
      Velocity_rate_sub = this->Ros_nh->subscribe("/LLC/EFFORTS/Throttle" , 1000, &bobtankDrivePlugin::On_Linear_command, this);
      
      platform_hb_pub_ = this->Ros_nh->advertise<std_msgs::Bool>("/Sahar/link_with_platform" , 100);

      // Listen to the update event. This event is broadcast every simulation iteration. 
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&bobtankDrivePlugin::OnUpdate, this, _1));

      this->model_reconfiguration_server = new dynamic_reconfigure::Server<bobcat_model::bobcat_modelConfig> (*(this->Ros_nh));
      this->model_reconfiguration_server->setCallback(boost::bind(&bobtankDrivePlugin::dynamic_Reconfiguration_callback, this, _1, _2));

      /* initialize random seed: */
      srand (time(NULL));
      this->Linear_Noise_dist = new std::normal_distribution<double>(0,1);
      this->Angular_Noise_dist = new std::normal_distribution<double>(0,1);

      calibration_data_setup();

      Linear_command_sum = 0;
      Angular_command_sum = 0;
      Linear_command_index = 0;
      Angular_command_index = 0;
      for (int i=0 ; i<LINEAR_COMMAND_FILTER_ARRY_SIZE ; i++)
          {
          Linear_command_array[i] = 0;
          }

      for (int i=0 ; i<ANGULAR_COMMAND_FILTER_ARRY_SIZE ; i++)
          {
          Angular_command_array[i] = 0;
          }

      }

   void calibration_data_setup()
    {
      // construct the grid in each dimension.
      // note that we will pass in a sequence of iterators pointing to the beginning of each grid
      double Throttle_commands_array[] =  { -1.00,    -0.70,    -0.40,     0.00,    0.40,     0.70,      1.00};

      double Sttering_commands_array[] =  { -1.00,    -0.70,    -0.40,     0.00,    0.40,     0.70,      1.00};

                                         //r=-1.00    r=-0.70   r=-0.40   r=0.00   r=0.40     r=0.70    r=1.00
      double Linear_vell_values_array[] = { -1.20,    -1.30,    -1.40,    -1.50,    -1.40,    -1.30,    -1.20,    //t=-1.00
                                            -0.65,    -0.70,    -0.75,    -0.80,    -0.75,    -0.70,    -0.65,    //t=-0.70
                                            -0.14,    -0.16,    -0.23,    -0.20,    -0.18,    -0.16,    -0.14,    //t=-0.40
                                             0.40,     0.17,     0.00,     0.00,     0.00,     0.17,     0.40,    //t=0.00
                                             0.14,     0.16,     0.23,     0.20,     0.23,     0.16,     0.14,    //t=0.40
                                             0.65,     0.70,     0.75,     0.80,     0.75,     0.70,     0.65,    //t=0.70
                                             1.20,     1.30,     1.40,     1.50,     1.40,     1.30,     1.20};   //t=1.00

                                         //r=-1.00    r=-0.70   r=-0.40   r=0.00    r=0.40    r=0.70    r=1.00
      double Angular_vell_values_array[] = { 1.40,     0.42,     0.18,     0.00,    -0.18,    -0.42,    -1.40,    //t=-1.00
                                             1.36,     0.38,     0.14,     0.00,    -0.14,    -0.38,    -1.36,    //t=-0.70
                                             1.32,     0.34,     0.10,     0.00,    -0.10,    -0.34,    -1.32,    //t=-0.40
                                            -1.22,    -0.24,     0.00,     0.00,     0.00,     0.24,     1.22,    //t=0.00
                                            -1.32,    -0.34,    -0.10,     0.00,     0.10,     0.34,     1.32,    //t=0.40
                                            -1.36,    -0.38,    -0.14,     0.00,     0.14,     0.38,     1.36,    //t=0.70
                                            -1.40,    -0.42,    -0.18,     0.00,     0.18,     0.42,     1.40};   //t=1.00


      std::vector<double> Throttle_commands(Throttle_commands_array, Throttle_commands_array+ sizeof(Throttle_commands_array)/sizeof(double));
      std::vector<double> Sttering_commands(Sttering_commands_array, Sttering_commands_array+ sizeof(Sttering_commands_array)/sizeof(double));
      std::vector<double> Linear_vell_values(Linear_vell_values_array, Linear_vell_values_array+ sizeof(Linear_vell_values_array)/sizeof(double));
      std::vector<double> Angular_vell_values(Angular_vell_values_array, Angular_vell_values_array+ sizeof(Angular_vell_values_array)/sizeof(double));


      std::vector< std::vector<double>::iterator > grid_iter_list;
      grid_iter_list.push_back(Throttle_commands.begin());
      grid_iter_list.push_back(Sttering_commands.begin());

      // the size of the grid in each dimension
      array<int,2> grid_sizes;
      grid_sizes[0] = Throttle_commands.size();
      grid_sizes[1] = Sttering_commands.size();

      // total number of elements
      int num_elements = grid_sizes[0] * grid_sizes[1];

      // construct the interpolator. the last two arguments are pointers to the underlying data
      Linear_vel_interp  =  new InterpMultilinear<2, double>(grid_iter_list.begin(), grid_sizes.begin(), Linear_vell_values.data(), Linear_vell_values.data() + num_elements);
      Angular_vel_interp =  new InterpMultilinear<2, double>(grid_iter_list.begin(), grid_sizes.begin(), Angular_vell_values.data(), Angular_vell_values.data() + num_elements);
      }

    public: void dynamic_Reconfiguration_callback(bobcat_model::bobcat_modelConfig &config, uint32_t level)
      {
          control_P = config.Wheel_conntrol_P;
          controll_I = config.Wheel_conntrol_I;
          controll_D = config.Wheel_conntrol_D;

          command_lN = config.Command_Linear_Noise;
          command_aN = config.Command_Angular_Noise;
      }

    // Called by the world update start event, This function is the event that will be called every update
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)  // we are not using the pointer to the info so its commanted as an option
    {

           // std::cout << "command_timer = " << command_timer.GetElapsed().Float() << std::endl;

            // Applying effort to the wheels , brakes if no message income
            if (Linear_command_timer.GetElapsed().Float()> command_MAX_DELAY)
            {
                // Brakes
                   Linear_command = 0;
            }

            if (Angular_command_timer.GetElapsed().Float()> command_MAX_DELAY)
            {
                // Brakes
                   Angular_command = 0;
            }

              update_ref_vels();
              apply_efforts();

              std_msgs::Bool connection;
              connection.data = true;
              platform_hb_pub_.publish(connection);
    }


double command_fillter(double prev_commands_array[], int array_size, double& commmand_sum, int &command_index , double command)
   {
        commmand_sum -= prev_commands_array[command_index];
        commmand_sum += command;

        prev_commands_array[command_index] = command;

        if( ++command_index == array_size)
            command_index = 0;

        double filtered_command = commmand_sum/array_size;

        return(filtered_command);
   }


    private: void update_ref_vels() // float linear_command, float angular_command)
    {
        Linear_command_mutex.lock();
        Angular_command_mutex.lock();
            array<double,2> args = {Linear_command, Angular_command};
        Linear_command_mutex.unlock();
        Angular_command_mutex.unlock();

        //printf("Linear_command = %f,  Angular_command = %f --->  Linear_vel_interp  = %f  \n", args[0], args[1],  Linear_vel_interp->interp(args.begin()) );
        //printf("Linear_command = %f,  Angular_command = %f --->  Angular_vel_interp = %f \n", args[0], args[1],  Angular_vel_interp->interp(args.begin()) );

        double Linear_nominal_vell = Linear_vel_interp->interp(args.begin());
        double Angular_nominal_vell = Angular_vel_interp->interp(args.begin());

        double Linear_vell = command_fillter(Linear_command_array, LINEAR_COMMAND_FILTER_ARRY_SIZE, Linear_command_sum, Linear_command_index , Linear_nominal_vell);
        double Angular_vell = command_fillter(Angular_command_array, ANGULAR_COMMAND_FILTER_ARRY_SIZE, Angular_command_sum, Angular_command_index , Angular_nominal_vell);

        double LinearNoise  = command_lN * (*Linear_Noise_dist)(generator);  //((std::rand() % 100)-50)/50;
        double AngularNoise = command_aN * (*Angular_Noise_dist)(generator); //((std::rand() % 100)-50)/50;

        Linear_ref_vel  =  (1 + LinearNoise)  * Linear_vell;
        Angular_ref_vel =  (1 + AngularNoise) * Angular_vell;
    }

    private: void wheel_controller(physics::JointPtr wheel_joint, double ref_omega)
    {
        double wheel_omega = wheel_joint->GetVelocity(0);

        double error = ref_omega - wheel_omega;

        double effort_command = (control_P * error);

        if(effort_command > WHEEL_EFFORT_LIMIT) effort_command = WHEEL_EFFORT_LIMIT;
        if(effort_command < -WHEEL_EFFORT_LIMIT) effort_command = -WHEEL_EFFORT_LIMIT;
        if(wheel_joint==this->cogwheel_right||wheel_joint==this->cogwheel_left) effort_command=effort_command*0.01;


//        std::cout << " wheel_joint->GetName() = " << wheel_joint->GetName() << std::endl;
//        std::cout << "           ref_omega = " << ref_omega << " wheel_omega = " << wheel_omega  << " error = " << error << " effort_command = " << effort_command <<  std::endl;


        // #if MY_GAZEBO_VER >= 6
                // wheel_joint->SetVelocity(0,ref_omega);
        // #else
                wheel_joint->SetForce(0,effort_command);
        // #endif

    }


  private: void apply_efforts()
    {

        //std::cout << " Linear_ref_vel = " << Linear_ref_vel << " Angular_ref_vel = " << Angular_ref_vel << std::endl;

        float right_side_vel = ( Linear_ref_vel ) + (Angular_ref_vel * WHEELS_BASE/2)*STEERING_FRICTION_COMPENSATION;
        float left_side_vel  = ( Linear_ref_vel ) - (Angular_ref_vel * WHEELS_BASE/2)*STEERING_FRICTION_COMPENSATION;

        //std::cout << " right_side_vel = " << right_side_vel <<  " left_side_vel = " << left_side_vel << std::endl;

        float right_wheels_omega_ref = right_side_vel / (0.5 * WHEEL_DIAMETER);
        float left_wheels_omega_ref = left_side_vel / (0.5 * WHEEL_DIAMETER);
        float roller_right_omega_ref=right_side_vel / (0.5 * ROLLER_DIAMETER);
        float roller_left_omega_ref=left_side_vel / (0.5 * ROLLER_DIAMETER);

        //std::cout << " right_wheels_omega_ref = " << right_wheels_omega_ref <<  " left_wheels_omega_ref = " << left_wheels_omega_ref << std::endl;

        wheel_controller(this->front_right_joint, right_wheels_omega_ref);
        wheel_controller(this->back_right_joint , right_wheels_omega_ref);
        wheel_controller(this->front_left_joint , left_wheels_omega_ref);
        wheel_controller(this->back_left_joint  , left_wheels_omega_ref);
        wheel_controller(this->roller_back_right, roller_right_omega_ref);
        wheel_controller(this->roller_mid_right, roller_right_omega_ref);
        wheel_controller(this->roller_front_right, roller_right_omega_ref);
        wheel_controller(this->roller_back_left, roller_left_omega_ref);
        wheel_controller(this->roller_mid_left, roller_left_omega_ref);
        wheel_controller(this->roller_front_left, roller_left_omega_ref);
        // wheel_joint->SetVelocity(0,ref_omega);
        wheel_controller(this->cogwheel_right  , right_wheels_omega_ref);
        wheel_controller(this->cogwheel_left  , left_wheels_omega_ref);
        
    }


    // The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
    private: void On_Angular_command(const std_msgs::Float64ConstPtr &msg)
    {
      Angular_command_mutex.lock();
          // Recieving referance steering angle  
          if(msg->data > 1)       { Angular_command =  1;          }
          else if(msg->data < -1) { Angular_command = -1;          }
          else                    { Angular_command = msg->data;   }

          // Reseting timer every time LLC publishes message
        #if  MY_GAZEBO_VER >= 5 
           Angular_command_timer.Reset();
        #endif
           Angular_command_timer.Start();


      Angular_command_mutex.unlock();
    }


    // The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
    private: void On_Linear_command(const std_msgs::Float64ConstPtr &msg)
    {
      Linear_command_mutex.lock();
          // Recieving referance velocity
          if(msg->data > 1)       { Linear_command =  1;          }
          else if(msg->data < -1) { Linear_command = -1;          }
          else                    { Linear_command = msg->data;   }

          // Reseting timer every time LLC publishes message
	#if  MY_GAZEBO_VER >= 5 
           Linear_command_timer.Reset();
        #endif
           Linear_command_timer.Start();

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
     private: physics::JointPtr roller_back_right;
     private: physics::JointPtr roller_mid_right;
     private: physics::JointPtr roller_front_right;
     private: physics::JointPtr roller_back_left;
     private: physics::JointPtr roller_mid_left;
     private: physics::JointPtr roller_front_left;
     private: physics::JointPtr cogwheel_left;
     private: physics::JointPtr cogwheel_right;




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
     private: common::Timer Linear_command_timer;
     private: common::Timer Angular_command_timer;



     // Defining private Mutex
     private: boost::mutex Angular_command_mutex;
     private: boost::mutex Linear_command_mutex;

     private: float Linear_command;
     private: float Angular_command;
     private: double Linear_ref_vel;
     private: double Angular_ref_vel;

     private: double Linear_command_array[LINEAR_COMMAND_FILTER_ARRY_SIZE];
     private: double Angular_command_array[ANGULAR_COMMAND_FILTER_ARRY_SIZE];
     private: double Linear_command_sum;
     private: double Angular_command_sum;
     private: int Linear_command_index;
     private: int Angular_command_index;


     private: dynamic_reconfigure::Server<bobcat_model::bobcat_modelConfig> *model_reconfiguration_server;
     private: double control_P, controll_I ,controll_D;		// PID constants
     private: double command_lN, command_aN;   // command noise factors

     std::default_random_engine generator;
     std::normal_distribution<double> * Linear_Noise_dist;
     std::normal_distribution<double> * Angular_Noise_dist;


     InterpMultilinear<2, double> * Linear_vel_interp;
     InterpMultilinear<2, double> * Angular_vel_interp;

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(bobtankDrivePlugin)
}
#endif
