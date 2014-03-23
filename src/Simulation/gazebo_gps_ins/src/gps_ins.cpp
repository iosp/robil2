#include <boost/bind.hpp>

#include <stdio.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"

#include <sstream>
#include <string>

#define CENTER_X_NS		31.2622		// GPS coordinates
#define CENTER_Y_EW		34.803611	// of lab 320
#define DEGREE_TO_M		111000 			//1 degree has appprox. 111km
#define ABS(x) (x > 0 ? x : -x)


#define TOPIC_NAME_GPS 		"/SENSORS/GPS"
#define TOPIC_NAME_IMU 		"/SENSORS/IMU"
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
  class GPS_INS : public ModelPlugin
  {

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
		_seq = 0;
      // Store the pointer to the model
      this->_model = _parent;
      //load config from sdf
      if(_sdf->HasElement("start_latitude"))
      {
	sdf::ElementPtr elem = _sdf->GetElement("start_latitude");
	double val;
	elem->GetValue()->Get(val);
	_start_latitude = val;
	//cout << "Value: " << str << endl;
      }
      else _start_latitude = 0;
      if(_sdf->HasElement("start_longitude"))
      {
	sdf::ElementPtr elem = _sdf->GetElement("start_longitude");
	double val;
	elem->GetValue()->Get(val);
	_start_longitude = val;
	//cout << "Value: " << str << endl;
      }
      else _start_longitude = 0;
      if(_sdf->HasElement("frequency"))
      {
	sdf::ElementPtr elem = _sdf->GetElement("frequency");
	int val;
	elem->GetValue()->Get(val);
	_frequency = val;
	//cout << "Value: " << str << endl;
      }
      else _frequency = 1;
      _lastTime = 0;
      physics::PhysicsEnginePtr engine =  _model->GetWorld()->GetPhysicsEngine();
      
      //sensors::SensorPtr sensorGPS;
      sensors::SensorPtr sensorIMU;
      
      //sensorGPS = sensors::SensorManager::Instance()->GetSensor(SENSOR_GPS_NAME); 
      sensorIMU = sensors::SensorManager::Instance()->GetSensor(SENSOR_IMU_NAME); 
      
      if(!sensorIMU)
      {
	string error = "GPS/INS Sensor Model \"" + _parent->GetName() + "\" failed to locate some of his sub-sensors.\n(Do the names in the .sdf match the names in the .cpp?)\n";
	gzthrow(error);
	return;
      }
      
      //_gps = boost::dynamic_pointer_cast<sensors::GpsSensor>(sensorGPS);
      _imu = boost::dynamic_pointer_cast<sensors::ImuSensor>(sensorIMU);
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GPS_INS::OnUpdate, this, _1));
      _publisherGPS = _nodeHandle.advertise<sensor_msgs::NavSatFix>(TOPIC_NAME_GPS, 10);
	  _publisherIMU = _nodeHandle.advertise<sensor_msgs::Imu>(TOPIC_NAME_IMU, 10);

    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & _info)
    {
      //manage frequency
      common::Time simTime = _info.simTime;
      if(simTime.Double()-_lastTime.Double() < 1.0/_frequency) return;
      _lastTime = simTime;
      
	  	sensor_msgs::NavSatFix msg_gps;
		sensor_msgs::Imu msg_imu;

		math::Pose pose=_model->GetWorldPose();
		gazebo::math::Vector3 pos = pose.pos;		
		msg_gps.altitude = pos.z;
		msg_gps.longitude = _start_longitude + pos.y/DEGREE_TO_M;
		msg_gps.latitude = _start_latitude + pos.x/DEGREE_TO_M;
		msg_gps.header.seq = _seq++;
		msg_gps.header.frame_id = 1;
		msg_gps.header.stamp.sec = (int)simTime.Double();
		msg_gps.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
		msg_gps.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

		msg_imu.header.seq = _seq;
		msg_imu.header.frame_id = 1;
		msg_imu.header.stamp.sec = (int)simTime.Double();
		msg_imu.orientation.x = pose.rot.x;
		msg_imu.orientation.y = pose.rot.y;
		msg_imu.orientation.z = pose.rot.z;
		msg_imu.orientation.w = pose.rot.w;
		
		msg_imu.angular_velocity.x = _imu->GetAngularVelocity().x;
		msg_imu.angular_velocity.y = _imu->GetAngularVelocity().y;
		msg_imu.angular_velocity.z = _imu->GetAngularVelocity().z;
		
		msg_imu.linear_acceleration.x = _imu->GetLinearAcceleration().x;
		msg_imu.linear_acceleration.y = _imu->GetLinearAcceleration().y;
		msg_imu.linear_acceleration.z = _imu->GetLinearAcceleration().z;

		_publisherGPS.publish(msg_gps);
		_publisherIMU.publish(msg_imu);
		
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
    ros::Publisher 		_publisherGPS, _publisherIMU;
    
    //sensors::GpsSensorPtr 	_gps;
    sensors::ImuSensorPtr 	_imu;
    
    double _start_latitude, _start_longitude;
    int  _frequency;
    common::Time		_lastTime;
    int 			_seq;
  };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GPS_INS)
}
