#include <boost/bind.hpp>

#include <stdio.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/sensors/SensorTypes.hh"
//#include "gazebo/sensors/GpsSensor.hh"
//#include "GpsSensor.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include "std_msgs/String.h"
#include "gazebo_gps_ins/GPS_INS.h"
#include <sstream>
#include <string>

#define CENTER_X_NS		31.2622		// GPS coordinates
#define CENTER_Y_EW		34.803611	// of lab 320
#define DEGREE_TO_M		111000 			//1 degree has appprox. 111km
#define ABS(x) (x > 0 ? x : -x)

#define TOPIC_NAME 		"GPS_INS"
#define SENSOR_GPS_NAME		"gps_component"
#define SENSOR_IMU_NAME		"imu_component"

using namespace std;

//        E (y)
//         ^ 
//         |
//  S ------------> N (x)
//         |
//         |
//         W


using std::string;
using std::stringstream;

namespace gazebo
{   
  class GPS : public ModelPlugin
  {

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->_model = _parent;
      if(_sdf->HasElement("start_latitude"))
      {
	sdf::ElementPtr elem = _sdf->GetElement("start_latitude");
	double val;
	elem->GetValue()->Get(val);
	_start_latitude = val;
	//cout << "Value: " << str << endl;
      }
      if(_sdf->HasElement("start_longitude"))
      {
	sdf::ElementPtr elem = _sdf->GetElement("start_longitude");
	double val;
	elem->GetValue()->Get(val);
	_start_longitude = val;
	//cout << "Value: " << str << endl;
      }
      physics::PhysicsEnginePtr engine =  _model->GetWorld()->GetPhysicsEngine();
      
      sensors::SensorPtr sensorGPS;
      sensors::SensorPtr sensorIMU;
      
      sensorGPS = sensors::SensorManager::Instance()->GetSensor(SENSOR_GPS_NAME); 
      sensorIMU = sensors::SensorManager::Instance()->GetSensor(SENSOR_IMU_NAME); 
      
      if(!sensorGPS || !sensorIMU)
      {
	string error = "GPS/INS Sensor Model \"" + _parent->GetName() + "\" failed to locate some of his sub-sensors.\n(Do the names in the .sdf match the names in the .cpp?)\n";
	gzthrow(error);
	return;
      }
      
      _gps = boost::dynamic_pointer_cast<sensors::GpsSensor>(sensorGPS);
      _imu = boost::dynamic_pointer_cast<sensors::ImuSensor>(sensorIMU);
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GPS::OnUpdate, this, _1));
      _publisher = _nodeHandle.advertise<gazebo_gps_ins::GPS_INS>(TOPIC_NAME, 1000);

    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      gazebo_gps_ins::GPS_INS msg;
      msg.altitude = _gps->GetAltitude();
      msg.longitude = _gps->GetLongitude().Degree()+_start_longitude;
      msg.latitude = _gps->GetLatitude().Degree()+_start_latitude;
      
      
      msg.linearAcceleration.resize(3);
      msg.angularVelocity.resize(3);
      msg.orientation.resize(3);
      
      msg.linearAcceleration[0] = _imu->GetLinearAcceleration().x;
      msg.linearAcceleration[1] = _imu->GetLinearAcceleration().y;
      msg.linearAcceleration[2] = _imu->GetLinearAcceleration().z;
      
      msg.angularVelocity[0] = _imu->GetAngularVelocity().x;
      msg.angularVelocity[1] = _imu->GetAngularVelocity().y;
      msg.angularVelocity[2] = _imu->GetAngularVelocity().z;
      
      msg.orientation[0] = _imu->GetOrientation().GetRoll();
      msg.orientation[1] = _imu->GetOrientation().GetPitch();
      msg.orientation[2] = _imu->GetOrientation().GetYaw();
      
      _publisher.publish(msg);
    }


    static string WorldPosToGPLL(gazebo::math::Vector3& pos)
    {
      double NS = CENTER_X_NS + pos.x/DEGREE_TO_M;
      double EW = CENTER_Y_EW + pos.y/DEGREE_TO_M;
      char NS_sign = NS < 0 ? 'S' : 'N';
      char EW_sign = EW < 0 ? 'W' : 'E';
      stringstream ss;
      ss << "$GPLL," << ABS(NS) << " " << NS_sign << ", " << ABS(EW) << " " << EW_sign << "*";

      //sprintf(buf, "$GPLL,%g %c, %g %c*", ABS(NS), NS_sign, ABS(EW), EW_sign);
      string ans = ss.str();
      return ans;
    }


  protected:



  private:
    physics::ModelPtr 		_model; // Pointer to the model
    event::ConnectionPtr 	_updateConnection; // Pointer to the update event connection

    ros::NodeHandle		_nodeHandle;
    ros::Publisher 		_publisher;
    
    sensors::GpsSensorPtr 	_gps;
    sensors::ImuSensorPtr 	_imu;
    
    double _start_latitude, _start_longitude;
  };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GPS)
}
