#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <stdio.h>
//#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include "std_msgs/Float64.h"
#include <sstream>
#include <string>


#define TOPIC_NAME 		"/SENSORS/WIRE"
using namespace std;


using std::string;
using std::stringstream;

namespace gazebo
{   
  class WIRE_SENSOR : public ModelPlugin
  {

  public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
	  // Store the pointer to the model
		

		this->_frequency = 10;
		if(_sdf->HasElement("sensor_name"))
		{
			sdf::ElementPtr elem = _sdf->GetElement("sensor_name");
			string val;
			elem->GetValue()->Get(val);
			_sensor_name = val;
		} else _sensor_name = "wire_sensor";

		if(_sdf->HasElement("mount_name"))
		{
			sdf::ElementPtr elem = _sdf->GetElement("mount_name");
			string val;
			elem->GetValue()->Get(val);
			_mount_name = val;
		} else _mount_name = "wire_sensor_mount";


		if(_sdf->HasElement("noise"))
		{
			sdf::ElementPtr elem = _sdf->GetElement("noise");
			double val;
			elem->GetValue()->Get(val);
			_noise = val;
		} else _noise = 0;

		_link1=_parent->GetLink(_sensor_name);
		if(!_link1)
		{
		gzthrow(string("Wire Sensor Load Error: Cannot find the model \"")+_sensor_name+"\" in the sdf. Please make sure it is loaded.");
		return;
		}

		_link2=_parent->GetLink(_mount_name);
		if(!_link2)
		{
		gzthrow(string("Wire Sensor Load Error: Cannot find the model \"")+_mount_name+"\" in the sdf. Please make sure it is loaded.");
		return;
		}
#if GAZEBO_MAJOR_VERSION >= 7
		this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&WIRE_SENSOR::OnUpdate, this, _1));
#else
		this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&WIRE_SENSOR::OnUpdate, this, _1));
#endif
		_publisher = _nodeHandle.advertise<std_msgs::Float64>(TOPIC_NAME, 10);

	}

	// Called by the world update start event
	void OnUpdate(const common::UpdateInfo & _info)
	{
		//manage frequency
		common::Time simTime = _info.simTime;
		if(simTime.Double()-_lastTime.Double() < 1.0/_frequency) return;
		_lastTime = simTime;
	  	
 		math::Pose mainPose = this->_link1->GetWorldPose();
		math::Pose mountPose = this->_link2->GetWorldPose();
		
		math::Vector3 d_pos = mainPose.pos - mountPose.pos;
		double dist = d_pos.Distance(0,0,0);
		double noise = _noise*(2*(double(rand())/double(RAND_MAX))-1);
		dist += noise;
		std_msgs::Float64 msg;
		msg.data = dist;
		_publisher.publish(msg);
      
	}

  protected:



  private:
    physics::LinkPtr 		_link1; // Pointer to the model
    physics::LinkPtr 		_link2; // Pointer to the attachment model (the mount part)
    event::ConnectionPtr 	_updateConnection; // Pointer to the update event connection

    ros::NodeHandle		_nodeHandle;
    ros::Publisher 		_publisher;
  
    int  			_frequency;
    double 			_noise;
    string 			_mount_name;
    string 			_sensor_name;
    common::Time		_lastTime;
  };

 // Register this plugin with the simulator
 GZ_REGISTER_MODEL_PLUGIN(WIRE_SENSOR)
}
