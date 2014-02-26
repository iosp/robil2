#ifndef GAZEBO_ROS_TEMPLATE_HH
#define GAZEBO_ROS_TEMPLATE_HH

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{

   class GazeboChainPlugin : public ModelPlugin
   {
      /// \brief Constructor
      public: GazeboChainPlugin();

      /// \brief Destructor
      public: virtual ~GazeboChainPlugin();

      /// \brief Load the controller
      public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      /// \brief Update the controller
      protected: virtual void UpdateChild();

      private: physics::JointPtr fixed_joint;
      private: physics::LinkPtr link1;
      private: physics::LinkPtr link2;
      private: physics::WorldPtr world_;
      private: physics::ModelPtr model_;
   };

}

#endif

