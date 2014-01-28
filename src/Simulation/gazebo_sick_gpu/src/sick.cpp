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

#ifndef SICK_GPU_PLUGIN
#define SICK_GPU_PLUGIN

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/GpuRaySensor.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


#define PI 			3.14159
#define TOPIC_NAME 		"LMS511"
#define SENSOR_FREQUENCY	25
//sensor field angle (left+right) in DEGREES

using namespace gazebo;
using namespace std;



namespace gazebo
{
  class SickPlugin : public SensorPlugin
  {
  
    public:
      SickPlugin()
      {
      
	
      }

      virtual ~SickPlugin()
      {
	this->parentSensor->DisconnectNewLaserFrame(this->newLaserFrameConnection);
	this->newLaserFrameConnection.reset();

	this->parentSensor.reset();
	this->world.reset();
      }
      
      void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
      {
	
	//printf("I have loaded.!!!\n");
	this->parentSensor = boost::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);

	if (!this->parentSensor) gzthrow("RayPlugin requires a Ray Sensor as its parent");

	this->world = physics::get_world(this->parentSensor->GetWorldName());
  	this->newLaserFrameConnection = this->parentSensor->ConnectNewLaserFrame(boost::bind(&SickPlugin::OnNewLaserFrame, this));
	_publisher = _nodeHandle.advertise<sensor_msgs::LaserScan>(TOPIC_NAME, 10);
      
	
      }
      
      virtual void OnNewLaserFrame()
      {
	common::Time simTime = world->GetSimTime();
	if(simTime.Double() - _lastTime.Double() < 0.04) return;
	vector<double> ranges;
	parentSensor->GetRanges(ranges);
	//printf("update: %d\n", ranges.size());
	PublishMessage(ranges);
	_lastTime = simTime;
	
      }
      
      void PublishMessage(vector<double> &ranges)
      {
	ros::Time scan_time = ros::Time::now();
    
	//populate the LaserScan message
	sensor_msgs::LaserScan scan;
	scan.header.stamp = scan_time;
	scan.header.frame_id = "laser_frame";
	scan.angle_min = DegreesToRad(parentSensor->GetAngleMin().Degree());
	scan.angle_max = DegreesToRad(parentSensor->GetAngleMax().Degree());
	scan.angle_increment = 3.14 / (double)ranges.size();
	scan.time_increment = 1000/double(SENSOR_FREQUENCY);
	scan.range_min = parentSensor->GetRangeMin();
	scan.range_max = parentSensor->GetRangeMax();

	scan.ranges.resize(ranges.size());
	//scan.intensities.resize(SENSOR_SAMPLES);
	
	for(unsigned int i = 0; i < ranges.size(); ++i)
	{
	  scan.ranges[i] = ranges[i];
	  //scan.intensities[i] = 1;
	}

	_publisher.publish(scan);
	//poseCallback();
      }
      
      inline double DegreesToRad(double degrees)
      {
	return (degrees/(180/PI));
      }

      void poseCallback() 
      { 
	static tf::TransformBroadcaster br; 
	tf::Transform transform; 
	math::Pose pose= parentSensor->GetPose(); 
	transform.setOrigin( tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z) ); 
	transform.setRotation( tf::Quaternion(pose.rot.x,pose.rot.y,pose.rot.z,pose.rot.w) ); 
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "laser_frame")); 
	
      }

    protected: 
      physics::WorldPtr world;

    private: 
      common::Time 		_lastTime;
      sensors::GpuRaySensorPtr parentSensor;	
      event::ConnectionPtr newLaserFrameConnection;
      
      ros::NodeHandle		_nodeHandle;
      ros::Publisher 		_publisher;
  };
}


// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(SickPlugin)
#endif
