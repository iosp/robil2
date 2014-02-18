/*
 * ParameterType.h
 *
 *  Created on: Jan 21, 2014
 *      Author: Dan Erusalimchik (danerde@gmail.com)
 */

#ifndef PARAMETERTYPES_DEFS_H_
#define PARAMETERTYPES_DEFS_H_
#include <ros/ros.h>
#include <string>

#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
//#include <gps_common/GPSFix.h>
//#include <gps_common/GPSStatus.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <robil_msgs/Path.h>
#include <robil_msgs/IEDLocation.h>
#include <robil_msgs/Map.h>
#include <robil_msgs/AssignManipulatorTask.h>
#include <robil_msgs/AssignMission.h>
#include <robil_msgs/AssignNavTask.h>
#include <robil_msgs/MissionAcceptance.h>

#define DEF_PUB( COMP, TOPIC, TYPE )\
	namespace config{ namespace COMP{ namespace pub{ typedef TYPE TOPIC; } } }
#define DEF_SUB( COMP, TOPIC, COMP_SOURCE )\
	namespace config{ namespace COMP{ namespace sub{ typedef config::COMP_SOURCE::pub::TOPIC TOPIC; } } }

typedef std_msgs::String TEMP_TYPE;

namespace types{
	typedef geometry_msgs::PointStamped Point;
	typedef geometry_msgs::PoseStamped Pose;
	typedef geometry_msgs::PoseStamped BladPose; //??? JointState?
	typedef geometry_msgs::PoseWithCovarianceStamped PoseWithConf;
	typedef robil_msgs::Path Path;
	typedef nav_msgs::Odometry Odometry;
	typedef sensor_msgs::LaserScan Laser;
	typedef sensor_msgs::Image CamFrame;
	typedef sensor_msgs::CameraInfo CamInfo;
	typedef sensor_msgs::Imu Imu;
	typedef sensor_msgs::NavSatFix GPS;
	typedef sensor_msgs::NavSatStatus GPSStatus;
	typedef robil_msgs::Map Map;
	typedef std_msgs::Float64 WireLength;
	typedef geometry_msgs::TwistStamped Speed;
	typedef std_msgs::Int32 Effort;
	typedef robil_msgs::IEDLocation IEDLocation;
	typedef std_msgs::String Events;
	typedef robil_msgs::AssignManipulatorTask ManTask;
	typedef robil_msgs::AssignNavTask NavTask;
	typedef robil_msgs::AssignMission Mission;
	typedef robil_msgs::MissionAcceptance MissionAcceptance;
	typedef diagnostic_msgs::DiagnosticStatus Diagnostic;
	typedef robil_msgs::AssignManipulatorTask WSMData;
	typedef sensor_msgs::JointState Joints;
}


#endif /* PARAMETERTYPES_H_ */
