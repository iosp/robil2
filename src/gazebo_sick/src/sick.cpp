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
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>


#define PI 			3.14159
#define TOPIC_NAME 		"LMS511"
//sensor range in meters
#define SENSOR_RANGE		65.0 
//frequency between 2 consequent ray scans (in Hz)
#define SENSOR_FREQUENCY	25
//sensor field angle (left+right) in DEGREES
#define SENSOR_ANGLE		190.0
#define SENSOR_SAMPLES		760

#define SENSOR_NOISE		0.1

 
using namespace gazebo;
using namespace std;



namespace gazebo
{   
  class SICK : public ModelPlugin
  {

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->_model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SICK::OnUpdate, this, _1));
      _publisher = _nodeHandle.advertise<sensor_msgs::LaserScan>(TOPIC_NAME, 10);
      
      gazebo::physics::PhysicsEnginePtr engine =  _model->GetWorld()->GetPhysicsEngine();
      //engine->InitForThread();
      _ray = boost::shared_dynamic_cast<gazebo::physics::RayShape>(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

      _sample = 0;
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      _sample++;
      if(_sample >= SENSOR_SAMPLES) 
      {
	PublishMessage();
	_sample = 0;
      }
      double dist;
      std::string entityName;
      double alpha = -SENSOR_ANGLE/2 + _sample*(SENSOR_ANGLE/SENSOR_SAMPLES);
      //cout << "Alpha is: " << alpha << endl;
      RayTraceAtAngle(DegreesToRad(alpha), dist, entityName);
      if(dist > SENSOR_RANGE) dist = SENSOR_RANGE;
      dist = dist+((double)rand()/RAND_MAX)*SENSOR_NOISE;
      //dist = (int)dist;
      _ranges[_sample] = dist;
      /*
      _ray->SetPoints(pos4, end4);
      _ray->GetIntersection(dist, entityName);
      if(dist == 1000 || dist > SENSOR_RANGE) cout << "LB LOS is clear.." << endl;
      else cout << "LB Hit after " << dist << "m on " << entityName << endl;
    
      */
      poseCallback();
    }
    
    inline double DegreesToRad(double degrees)
    {
      return (degrees/(180/PI));
    }
    
    
    void RayTraceAtAngle(double alpha, double& dist, std::string& entityName)
    {
      math::Pose pose=_model->GetWorldPose();
      gazebo::math::Vector3 pos = pose.pos;
 
      math::Vector3 front, right, top;
      GetMuzzleTransform(pose.rot, &right, &front, &top);
      
      double F = SENSOR_RANGE*sin(alpha);
      double R = SENSOR_RANGE*cos(alpha);
      math::Vector3 start = pos;
      math::Vector3 end = front*F + right*R;
      
      _ray->SetPoints(start, end);
      _ray->GetIntersection(dist, entityName);
    }

    void GetMuzzleTransform(math::Quaternion& q, math::Vector3* right,math::Vector3* front, math::Vector3* up)
    {
      if(right)	*right 	= q.GetXAxis();
      if(front)	*front 	= q.GetYAxis();
      if(up) 	*up 	= q.GetZAxis();
    }

    //publishes the formatted GPS coords to a ROS topic
    
    void PublishMessage()
    {
      ros::Time scan_time = ros::Time::now();
   
      //populate the LaserScan message
      sensor_msgs::LaserScan scan;
      scan.header.stamp = scan_time;
      scan.header.frame_id = "laser_frame";
      scan.angle_min = DegreesToRad(-SENSOR_ANGLE/2);
      scan.angle_max = DegreesToRad(SENSOR_ANGLE/2);
      scan.angle_increment = 3.14 / SENSOR_SAMPLES;
      scan.time_increment = (1 / (double)SENSOR_FREQUENCY) / (SENSOR_SAMPLES);
      scan.range_min = 0.0;
      scan.range_max = SENSOR_RANGE;

      scan.ranges.resize(SENSOR_SAMPLES);
      //scan.intensities.resize(SENSOR_SAMPLES);
      
      for(unsigned int i = 0; i < SENSOR_SAMPLES; ++i)
      {
	scan.ranges[i] = _ranges[i];
	//scan.intensities[i] = 1;
      }

      _publisher.publish(scan);
     
    }

	void poseCallback()
	{
	  static tf::TransformBroadcaster br;  
	  tf::Transform transform;
	  math::Pose pose=_model->GetWorldPose();
	  gazebo::math::Vector3 pos = pose.pos;
	  gazebo::math::Quaternion rot = pose.rot;
	  double R = rot.GetRoll ( );
	  double P = rot.GetPitch ( );
	  double Y = rot.GetYaw ( );
	  
	  transform.setOrigin( tf::Vector3(pos.x, pos.y, pos.z) );
	  transform.setRotation( tf::Quaternion(Y,P,R) );
	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "laser_frame"));
	}

  protected:



  private:
    physics::ModelPtr 			_model; // Pointer to the model
    event::ConnectionPtr 		_updateConnection; // Pointer to the update event connection
    gazebo::physics::RayShapePtr	_ray;
    
    ros::NodeHandle		_nodeHandle;
    ros::Publisher 		_publisher;
    
    int 			_sample;
    double 			_ranges[SENSOR_SAMPLES];
    
  };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SICK)
}

#endif
