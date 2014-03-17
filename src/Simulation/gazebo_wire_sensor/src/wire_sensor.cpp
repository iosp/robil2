#include <boost/bind.hpp>

#include <stdio.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/gazebo.hh"
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
		this->_model = _parent;

		this->_frequency = 10;

		if(_sdf->HasElement("noise"))
		{
			sdf::ElementPtr elem = _sdf->GetElement("noise");
			double val;
			elem->GetValue()->Get(val);
			_noise = val;
		} else _noise = 0;

		this->_model2 = _model->GetWorld()->GetModel("wire_sensor_mount");
		if(this->_model2 == NULL) 
		{
		gzthrow("Wire Sensor Load Error: Cannot find the model \"wire_sensor_mount\" in the world. Please make sure it is loaded.");
		return;
		}

		this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&WIRE_SENSOR::OnUpdate, this, _1));
		_publisher = _nodeHandle.advertise<std_msgs::Float64>(TOPIC_NAME, 10);

	}

	// Called by the world update start event
	void OnUpdate(const common::UpdateInfo & _info)
	{
		//manage frequency
		common::Time simTime = _info.simTime;
		if(simTime.Double()-_lastTime.Double() < 1.0/_frequency) return;
		_lastTime = simTime;
	  	
 		math::Pose mainPose = this->_model->GetWorldPose();
		math::Pose mountPose = this->_model2->GetWorldPose();
		
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
    physics::ModelPtr 		_model; // Pointer to the model
    physics::ModelPtr 		_model2; // Pointer to the attachment model (the mount part)
    event::ConnectionPtr 	_updateConnection; // Pointer to the update event connection

    ros::NodeHandle		_nodeHandle;
    ros::Publisher 		_publisher;
  
    int  				_frequency;
	double 				_noise;
    common::Time		_lastTime;
  };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(WIRE_SENSOR)
}
