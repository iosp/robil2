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
#include <ctime>
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

const double PI  =3.141592653589793238463;

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
      srand(time(NULL));
      
		_seq = 0;
      // Store the pointer to the model
      this->_model = _parent;
      _init_pos = _parent->GetWorldPose().pos;
      
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
      _gps_noise=_rp_noise=_yaw_noise=_gy_noise=_acc_noise=_acc_bias=_gy_bias=0;
      if(_sdf->HasElement("noise"))
      {
	sdf::ElementPtr elem = _sdf->GetElement("noise");
	if (elem->HasElement("gps"))
	  elem->GetElement("gps")->GetValue()->Get(_gps_noise);
	if (elem->HasElement("rollpitch"))
	  elem->GetElement("rollpitch")->GetValue()->Get(_rp_noise);
	if (elem->HasElement("yaw"))
	  elem->GetElement("yaw")->GetValue()->Get(_yaw_noise);
	if (elem->HasElement("acc_bias"))
	  elem->GetElement("acc_bias")->GetValue()->Get(_acc_bias);
	if (elem->HasElement("_gyro_bias"))
	  elem->GetElement("_gyro_bias")->GetValue()->Get(_gy_bias);
	if (elem->HasElement("acc_noise"))
	  elem->GetElement("acc_noise")->GetValue()->Get(_acc_noise);
	if (elem->HasElement("gyro_noise"))
	  elem->GetElement("gyro_noise")->GetValue()->Get(_gy_noise);
      }

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
    double sampleNormal() 
    {
      double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
      double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
      double r = u * u + v * v;
      if (r == 0 || r > 1) return sampleNormal();
      double c = sqrt(-2 * log(r) / r);
      return u * c;
    }
    double add_gps_noise()
    {
      return sampleNormal()*_gps_noise;
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
		
		pos.x += add_gps_noise();
		pos.y += add_gps_noise();
		double other_dist = (pos - _init_pos).GetLength();
		double dist = sqrt((pos.x-_init_pos.x)*(pos.x-_init_pos.x)+(pos.y-_init_pos.y)*(pos.y-_init_pos.y));
		//std::cout << "x= " << pos.x << "  y= " << pos.y <<"   init.x= "<<_init_pos.x<<"   init.y= "<<_init_pos.y<<"   dist= "<<dist <<"   my_dist= " << my_dist<<std::endl;
		double brng;
		if(!(pos.GetLength()*_init_pos.GetLength())) brng = atan2(pos.y,pos.x);
		else brng = atan2(pos.y-_init_pos.y,pos.x-_init_pos.x);//acos(pos.Dot(_init_pos)/(pos.GetLength()*_init_pos.GetLength()));
		//std::cout << "bearing: " << brng << std::endl;
		//lat2 = asin(sin(lat1)*cos(d/R) + cos(lat1)*sin(d/R)*cos(θ))
		//lon2 = lon1 + atan2(sin(θ)*sin(d/R)*cos(lat1), cos(d/R)−sin(lat1)*sin(lat2))
		
		double R = 6378.1*1000;
		msg_gps.altitude = pos.z;
		msg_gps.latitude = 180/PI*asin(sin(_start_latitude*PI/180)*cos(dist/R)+cos(_start_latitude*PI/180)*sin(dist/R)*cos(brng));
		
		msg_gps.longitude = _start_longitude + 180/PI*atan2(sin(brng)*sin(dist/R)*cos(_start_latitude*PI/180),cos(dist/R)-sin(_start_latitude*PI/180)*sin(msg_gps.latitude*PI/180));
		
		msg_gps.header.seq = _seq++;
		msg_gps.header.frame_id = 1;
		msg_gps.header.stamp = ros::Time::now();
		msg_gps.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
		msg_gps.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

		msg_imu.header.seq = _seq;
		msg_imu.header.frame_id = 1;
		msg_imu.header.stamp = ros::Time::now();
		
		msg_imu.orientation.x = pose.rot.x;
		msg_imu.orientation.y = pose.rot.y;
		msg_imu.orientation.z = pose.rot.z;
		msg_imu.orientation.w = pose.rot.w;
		
		msg_imu.angular_velocity.x = _imu->GetAngularVelocity().x;+_gy_bias+_gy_noise*sampleNormal();
		msg_imu.angular_velocity.y = _imu->GetAngularVelocity().y;+_gy_bias+_gy_noise*sampleNormal();
		msg_imu.angular_velocity.z = _imu->GetAngularVelocity().z;+_gy_bias+_gy_noise*sampleNormal();
		
		msg_imu.linear_acceleration.x = _imu->GetLinearAcceleration().x+_acc_bias+_acc_noise*sampleNormal();
		msg_imu.linear_acceleration.y = _imu->GetLinearAcceleration().y+_acc_bias+_acc_noise*sampleNormal();
		msg_imu.linear_acceleration.z = _imu->GetLinearAcceleration().z+_acc_bias+_acc_noise*sampleNormal();

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
    math::Vector3 _init_pos;
    double _start_latitude, _start_longitude;
    double _gps_noise,_rp_noise, _yaw_noise, _gy_noise, _acc_noise, _acc_bias, _gy_bias;
    int  _frequency;
    common::Time		_lastTime;
    int 			_seq;
  };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GPS_INS)
}
