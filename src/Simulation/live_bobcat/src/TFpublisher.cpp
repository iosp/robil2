
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
#include <sstream>
#include <algorithm>
#include <iterator>
#include <string.h>
#include <vector>
#include <stdlib.h>
using namespace gazebo;
using namespace std;


class sElement
{
public:
  string _child;
  string _parent;
  gazebo::math::Pose _pose;
};


class dElement
{
public:
  string _child;
  string _parent;
  physics::LinkPtr _link;
};

static tf::TransformBroadcaster br;
namespace gazebo
{
  class TFpublisher : public ModelPlugin
  {

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->_model = _parent;

      //populate elements
      int i=1;
      while(true)
      {
        stringstream ss;
        ss << "tf";
        ss << i;
        if (_sdf->HasElement(ss.str()+"frame"))
        {
		sElement e;
                e._child =_sdf->GetElement(ss.str()+"frame")->Get<std::string>();
		e._parent = _sdf->GetElement(ss.str()+"parent")->Get<std::string>();
#if GAZEBO_MAJOR_VERSION >= 7
		ignition::math::Pose3d sPose=_sdf->Get<ignition::math::Pose3d>("pose");
		e._pose = gazebo::math::Pose(sPose);
#else
		sdf::Pose sPose=_sdf->GetElement(ss.str()+"pose")->GetValuePose();
		e._pose.Set(sPose.pos.x,sPose.pos.y,sPose.pos.z,sPose.rot.GetAsEuler().x,sPose.rot.GetAsEuler().y,sPose.rot.GetAsEuler().z);
#endif

		i++;
		_staticElements.push_back(e);
        }else{
        	break;
        }

     }
//populate dynamic elements
      i=1;
      while(true)
      {
        stringstream ss;
        ss << "livetf";
        ss << i;
        if (_sdf->HasElement(ss.str()+"frame"))
        {
		dElement e;
                e._child =_sdf->GetElement(ss.str()+"frame")->Get<std::string>();
		e._parent = "world";
		e._link = _parent->GetLink(e._child);
		
		i++;
		_dynamicElements.push_back(e);
        }else{
        	break;
        }

     }


	double param_update_rate;

	if (!_sdf->HasElement("update_rate"))
	{
 	   // if parameter tag does NOT exist
 	   std::cout << "Missing parameter <update_rate> in PluginName, default to 0" << std::endl;
  	  param_update_rate = 0;
	}
    
	else param_update_rate = _sdf->Get<double>("update_rate");
        //locate all the parameters
	this->updateRate = 1.0/ param_update_rate;
// initialize the prevUpdateTime
	this->prevUpdateTime = common::Time::GetWallTime();


      // simulation iteration.
      this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TFpublisher::OnUpdate, this, _1));
      
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & _info)
    {  
	if (common::Time::GetWallTime() - this->prevUpdateTime < this->updateRate)
    		return;

        ros::Time time=ros::Time::now();
     	 for(int i = 0 ; i < _staticElements.size() ; i++)
	{
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(_staticElements[i]._pose.pos.x, _staticElements[i]._pose.pos.y, _staticElements[i]._pose.pos.z) );
		transform.setRotation( tf::Quaternion(_staticElements[i]._pose.rot.x,_staticElements[i]._pose.rot.y,_staticElements[i]._pose.rot.z,_staticElements[i]._pose.rot.w) );
		br.sendTransform(tf::StampedTransform(transform, time, _staticElements[i]._parent, _staticElements[i]._child));
	}

     	 for(int i = 0 ; i < _dynamicElements.size() ; i++)
	{
		gazebo::math::Pose pose=_dynamicElements[i]._link->GetWorldPose();
		tf::Transform transform;
			  transform.setOrigin( tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z) );
	  transform.setRotation( tf::Quaternion(pose.rot.x,pose.rot.y,pose.rot.z,pose.rot.w) );
		br.sendTransform(tf::StampedTransform(transform, time, _dynamicElements[i]._parent, _dynamicElements[i]._child));
	}

 	 this->prevUpdateTime = common::Time::GetWallTime();
    }
    
    
  protected:



  private:
    common::Time updateRate,prevUpdateTime;
    physics::ModelPtr 			_model; // Pointer to the model
    event::ConnectionPtr 		_updateConnection; // Pointer to the update event connection
    sensors::CameraSensorPtr 		_sensor;
    std::vector<sElement> _staticElements;
    std::vector<dElement> _dynamicElements;
  };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TFpublisher)
}

#endif
