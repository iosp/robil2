
/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Contact Plugin
 * Author: Nate Koenig mod by John Hsu
 */

#ifndef _GAZEBO_RAY_PLUGIN_HH_
#define _GAZEBO_RAY_PLUGIN_HH_

#include <sensor_msgs/LaserScan.h>
#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

#define SENSOR_NAME "flea3cam"
 
using namespace gazebo;
using namespace std;


/**
 * This model plugin is only for sending TF data for the camera
 * which is running on another plugin called razebo_ros_camera
 * which oded stole from somewhere.
 * NOTE: This will lag like shit so do not run if you don't require TF data.
 */

namespace gazebo
{   
  class Camera : public ModelPlugin
  {

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->_model = _parent;
      // simulation iteration.
      this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Camera::OnUpdate, this, _1));
      
      gazebo::physics::PhysicsEnginePtr engine =  _model->GetWorld()->GetPhysicsEngine();
      
      sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor(SENSOR_NAME); 
      
      if(!sensor)
      {
	string error = "Please name your sensor element \"flea3cam\" in the .sdf file to make TF plugin work properly.\n";
	gzthrow(error);
	return;
      }
#if GAZEBO_MAJOR_VERSION >= 7      
      _sensor = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
#else
      _sensor = boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
#endif
      
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & _info)
    {
     // BroadcastTF();
    }
    
    void BroadcastTF()
    {
      static tf::TransformBroadcaster br; 
      tf::Transform transform; 
#if GAZEBO_MAJOR_VERSION >= 7
     ignition::math::Pose3d pose= _sensor->Pose(); 
      transform.setOrigin(tf::Vector3((pose.Pos()).X(), (pose.Pos()).Y(), (pose.Pos()).Z() ) ); 
      transform.setRotation( tf::Quaternion((pose.Rot()).X(), (pose.Rot()).Y(), (pose.Rot()).Z(), (pose.Rot()).W() ) ); 
#else
      math::Pose pose= _sensor->GetPose(); 
      transform.setOrigin( tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z) ); 
      transform.setRotation( tf::Quaternion(pose.rot.x,pose.rot.y,pose.rot.z,pose.rot.w) ); 
#endif
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_frame")); 
    }
    
   

  protected:



  private:
    physics::ModelPtr 			_model; // Pointer to the model
    event::ConnectionPtr 		_updateConnection; // Pointer to the update event connection
    sensors::CameraSensorPtr 		_sensor;
  };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Camera)
}

#endif
