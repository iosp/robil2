/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Dave Coleman
 *  \desc   Example ROS plugin for Gazebo
 */

#include "chain_plugin.h"
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <cstring>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboChainPlugin);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboChainPlugin::GazeboChainPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboChainPlugin::~GazeboChainPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboChainPlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;
  this->world_->EnablePhysicsEngine(true);
#if GAZEBO_MAJOR_VERSION >= 7
  this->link1 = this->model_->GetLink(_sdf->GetElement("link1")->GetName());
  this->link2 = this->model_->GetLink(_sdf->GetElement("link2")->GetName());
#else
  this->link1 = this->model_->GetLink(_sdf->GetElement("link1")->GetValueString());
  this->link2 = this->model_->GetLink(_sdf->GetElement("link2")->GetValueString());
#endif

  // options are prismatic/screw/revolute/revolute2/ball/universal
  this->fixed_joint =this->world_->GetPhysicsEngine()->CreateJoint(std::string("revolute2"),this->model_);
  this->fixed_joint->SetAxis(0,math::Vector3(0,0,1));
#if GAZEBO_MAJOR_VERSION >= 7
  this->fixed_joint->SetName(_sdf->GetElement("joint")->GetName());
#else
  this->fixed_joint->SetName(_sdf->GetElement("joint")->GetValueString());
#endif
  this->fixed_joint->Attach(link1,link2);
  this->fixed_joint->Load(link1, link2, math::Pose(math::Vector3(0,0,0), math::Quaternion()));

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboChainPlugin::UpdateChild()
{
	std::cout<<"no update"<<std::endl;
}

}
