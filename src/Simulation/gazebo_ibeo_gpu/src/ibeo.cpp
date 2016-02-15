

/*
 * <INSERT FAKE LICENSE HERE>
 */

/*
 * Desc: LD-MRS Plugin for the Robil Project
 * Author: Roman Kudinov
 */

#ifndef GAZEBO_ROBIL_IBEO_PLUGIN
#define GAZEBO_ROBIL_IBEO_PLUGIN

#include "gazebo_ibeo_gpu/MultiLaserScan.h"
#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <boost/thread.hpp>
#define PI 			3.14159
#define TOPIC_NAME 		"SENSORS/IBEO/1"

#define SENSOR_T1_NAME		"ibeo-t1"
#define SENSOR_T2_NAME		"ibeo-t2"
#define SENSOR_B1_NAME		"ibeo-b1"
#define SENSOR_B2_NAME		"ibeo-b2"



using namespace gazebo;
using namespace std;


namespace gazebo
{   
class IBEO : public ModelPlugin
{

public:

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->_model = _parent;
	
        physics::PhysicsEnginePtr engine =  _model->GetWorld()->GetPhysicsEngine();

        sensors::SensorPtr sensorB1, sensorB2, sensorT1, sensorT2;

        sensorT1 = sensors::SensorManager::Instance()->GetSensor(SENSOR_T1_NAME);
        sensorT2 = sensors::SensorManager::Instance()->GetSensor(SENSOR_T2_NAME);
        sensorB1 = sensors::SensorManager::Instance()->GetSensor(SENSOR_B1_NAME);
        sensorB2 = sensors::SensorManager::Instance()->GetSensor(SENSOR_B2_NAME);

        if(!sensorB1 || !sensorB2 || !sensorT1 || !sensorT2)
        {
            string error = "IBEO Sensor Model \"" + _parent->GetName() + "\" failed to locate some of his 4 sub-sensors.\n(Do the names in the .sdf match the names in the .cpp?)\n";
            gzthrow(error);
            return;
        }

        _sensorB1 = boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensorB1);
        _sensorB2 = boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensorB2);
        _sensorT1 = boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensorT1);
        _sensorT2 = boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensorT2);

        if(!_sensorB1 || !_sensorB2 || !_sensorT1 || !_sensorT2)
        {
            string error = "IBEO Sensor Model \"" + _parent->GetName() + "\" found that it's sensors arent of class GpuRaySensor. You must be really messed up\n";
            gzthrow(error);
            return;
        }

        _updateRate = _sensorT1->GetUpdateRate();
        this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&IBEO::OnUpdate, this, _1));
	_lastPublish = clock();
	_publishRate = 1;
        _publisher = _nodeHandle.advertise<gazebo_ibeo_gpu::MultiLaserScan>(TOPIC_NAME, 10);
	

    }
    

    void OnUpdate(const common::UpdateInfo & _info)
    {
        clock_t now_clock = clock();
        common::Time simTime = _info.simTime;
        if(simTime.Double()-_lastTime.Double() < 1.0/_updateRate) return;
        ClearRanges();
        //vector<double> rangesT1, rangesT2, rangesB1, rangesB2;
//        boost::thread_group threads;

//        threads.create_thread(boost::bind(&IBEO::getB1, this));
//        threads.create_thread(boost::bind(&IBEO::getT1, this));
//        threads.create_thread(boost::bind(&IBEO::getT2, this));
//        threads.create_thread(boost::bind(&IBEO::getB2, this));
//        threads.join_all();
        getB1();
        getB2();
        getT1();
        getT2();
        vector<double> data_times;
	ros::param::param("/IBEO/Rate", _publishRate, 2.8);

        //data_times.push_back(simTime.Double());
        data_times.push_back(_sensorB1->GetLastUpdateTime().Double());
        data_times.push_back(_sensorB2->GetLastUpdateTime().Double());
        data_times.push_back(_sensorT1->GetLastUpdateTime().Double());
        data_times.push_back(_sensorT2->GetLastUpdateTime().Double());
	double d = (clock() - _lastPublish) / (float)CLOCKS_PER_SEC;
	if ( d < 1.0 / _publishRate) return;
	_lastPublish = clock();
        double time_diff = ( math::max<double>(data_times) - math::min<double>(data_times) );
        double time_diff_limit = 0.2;
	
//        FILE *f;
//        f = fopen("/home/lar6/time_diff.txt", "a");
//        fclose(f);
        if (time_diff > time_diff_limit) {
            ROS_WARN("IBEO data generation time accede the aloud limit of %f sec, delta = %f" ,time_diff_limit ,time_diff);
            ROS_WARN("B1_time = %f : B2_time = %f  : T1_time = %f : T2_time = %f" ,data_times[0], data_times[1],data_times[2],data_times[3]);
        }
        PublishMessage(_rangesT1, _rangesT2, _rangesB1, _rangesB2);

        _lastTime = simTime;
        BroadcastTF();
//         cout << (clock() - now_clock) / (float)CLOCKS_PER_SEC << endl;
    }
    
    inline double DegreesToRad(double degrees)
    {
        return (degrees/(180/PI));
    }
    
    void BroadcastTF()
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        math::Pose pose= _sensorT1->GetPose();
        transform.setOrigin( tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z) );
        transform.setRotation( tf::Quaternion(pose.rot.x,pose.rot.y,pose.rot.z,pose.rot.w) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ibeo"));

    }
    
    void PublishMessage(vector<double>& rangesT1, vector<double>& rangesT2, vector<double>& rangesB1, vector<double>& rangesB2)
    {
        ros::Time scan_time = ros::Time::now();

        gazebo_ibeo_gpu::MultiLaserScan scan;
        scan.header.frame_id="ibeo";
        scan.header.stamp=scan_time;
        scan.angle_min_t = _sensorT1->GetAngleMin().Radian();
        scan.angle_max_t = _sensorT1->GetAngleMax().Radian();
        scan.angle_min_b = _sensorB1->GetAngleMin().Radian();
        scan.angle_max_b = _sensorB1->GetAngleMax().Radian();
        scan.angle_increment = 0.0023; //according to datasheet
        scan.time_increment = 1.0/ _publishRate /700; //YUVAL_SHEKER
        scan.angle_t1 = 0.014;
        scan.angle_t2 = 0.028;
        scan.angle_b1 = -0.014; //according to datasheet
        scan.angle_b2 = -0.028;
        scan.scan_time = 0.0;
        scan.range_min = _sensorT1->GetRangeMin();
        scan.range_max = _sensorT1->GetRangeMax();

        scan.ranges_t1.resize(rangesT1.size());
        scan.ranges_t2.resize(rangesT2.size());
        scan.ranges_b1.resize(rangesB1.size());
        scan.ranges_b2.resize(rangesB2.size());
        //scan.intensities.resize(SENSOR_SAMPLES);

        for(unsigned int i = 0; i < rangesT1.size(); ++i) scan.ranges_t1[i] = rangesT1[i];
        for(unsigned int i = 0; i < rangesT2.size(); ++i) scan.ranges_t2[i] = rangesT2[i];
        for(unsigned int i = 0; i < rangesB1.size(); ++i) scan.ranges_b1[i] = rangesB1[i];
        for(unsigned int i = 0; i < rangesB2.size(); ++i) scan.ranges_b2[i] = rangesB2[i];

        _publisher.publish(scan);

    }
    void getT1()
    {
        _sensorT1->GetRanges(_rangesT1);
    }
    void getB1()
    {
        _sensorB1->GetRanges(_rangesB1);
    }
    void getT2()
    {
        _sensorT2->GetRanges(_rangesT2);
    }
    void getB2()
    {
        _sensorB2->GetRanges(_rangesB2);
    }
    void ClearRanges()
    {
        _rangesB1.clear();
        _rangesB2.clear();
        _rangesT1.clear();
        _rangesT2.clear();
    }


private:
    
    physics::ModelPtr 			_model; // Pointer to the model
    event::ConnectionPtr 		_updateConnection; // Pointer to the update event connection
    common::Time			_lastTime;
    
    sensors::GpuRaySensorPtr 		_sensorT1, _sensorT2, _sensorB1, _sensorB2;
    vector<double> _rangesT1, _rangesT2, _rangesB1, _rangesB2;
    
    ros::NodeHandle		_nodeHandle;
    ros::Publisher 		_publisher;
    double 			_updateRate, _publishRate; //in Hz
    clock_t                     _lastPublish;
    
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(IBEO)
}

#endif
