/*
 * ParameterType.h
 *
 *  Created on: Jan 21, 2014
 *      Author: Dan Erusalimchik (danerde@gmail.com)
 */

#ifndef PARAMETERTYPES_H_
#define PARAMETERTYPES_H_
#include <ros/ros.h>
#include <string>

#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
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
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#define DEF_PUB( COMP, TOPIC, TYPE )\
	namespace config{ namespace COMP{ namespace pub{ typedef TYPE TOPIC; } } }
#define DEF_SUB( COMP, TOPIC, COMP_SOURCE )\
	namespace config{ namespace COMP{ namespace sub{ typedef config::COMP_SOURCE::pub::TOPIC TOPIC; } } }

typedef std_msgs::String TEMP_TYPE;

namespace types{
	typedef geometry_msgs::PointStamped Point;
	typedef geometry_msgs::PoseStamped Pose; // ? tf::Transform
	typedef nav_msgs::Path Path;
	typedef nav_msgs::Odometry PoseVel;
	typedef Pose TF; // It must be tf::Transform, but tf::Transform is not a regular ROSMessage
	typedef sensor_msgs::LaserScan Laser;
	typedef sensor_msgs::PointCloud2 MultiLaser; // ? 4 topics of Laser
	typedef sensor_msgs::Image CamFrame;
	typedef sensor_msgs::CameraInfo CamInfo;
	typedef sensor_msgs::Imu IMU;
	typedef sensor_msgs::NavSatFix GPS; // ? gps_common::GPSFix
	typedef sensor_msgs::NavSatStatus GPSStatus;
	typedef nav_msgs::OccupancyGrid Map; // ??????
	typedef std_msgs::Float64 WireLength;
	typedef geometry_msgs::TwistStamped Speed;
	typedef std_msgs::Int32 Effort;
}

//----  PUB -----------------------

DEF_PUB( OCU, PositionUpdate, types::Pose )
DEF_PUB( OCU, MissionPlan, TEMP_TYPE )
DEF_PUB( OCU, Teleoperation, TEMP_TYPE )
DEF_PUB( OCU, IEDDetectionEvent, TEMP_TYPE )
DEF_PUB( OCU, IEDLocation, types::Point )

DEF_PUB( IEDSIM, IEDDetectionEvent, TEMP_TYPE )
DEF_PUB( IEDSIM, IEDLocation, types::Point )

DEF_PUB( SMME, MissionStatus, TEMP_TYPE )
DEF_PUB( SMME, MissionGlobalPath, types::Path )
DEF_PUB( SMME, IEDPosAtt, types::Pose )
DEF_PUB( SMME, ExecuteWorkSequenceCommand, TEMP_TYPE )

DEF_PUB( SSM, StatusData, TEMP_TYPE )

DEF_PUB( WSM, BladePosition, types::TF )
DEF_PUB( WSM, TrottleEffort, types::Effort )
DEF_PUB( WSM, SteeringEffort, types::Effort )
DEF_PUB( WSM, JointsEffort, types::Effort )

DEF_PUB( LLC, TrottleEffort, types::Effort )
DEF_PUB( LLC, SteeringEffort, types::Effort )
DEF_PUB( LLC, JointsEffort, types::Effort )

DEF_PUB( MAP, Map, types::Map )
DEF_PUB( MAP, MiniMap, types::Map )

DEF_PUB( LOC, PosAttVel, types::PoseVel )

DEF_PUB( PER, WiresLengths, types::WireLength )
DEF_PUB( PER, Camera, types::CamFrame )
DEF_PUB( PER, Laser, types::Laser )
DEF_PUB( PER, INS, types::IMU )
DEF_PUB( PER, GPS, types::GPS )
DEF_PUB( PER, TF,  types::TF)

DEF_PUB( SENSORS, Sensor_SICK, types::Laser )
DEF_PUB( SENSORS, Sensor_IBEO, types::MultiLaser )
DEF_PUB( SENSORS, Sensor_CAM_R, types::CamFrame )
DEF_PUB( SENSORS, Sensor_CAM_L, types::CamFrame )
DEF_PUB( SENSORS, Sensor_WIRE, types::WireLength )
DEF_PUB( SENSORS, Sensor_GPS, types::GPS )
DEF_PUB( SENSORS, Sensor_INS, types::IMU )

DEF_PUB( PP, LocalPathPlan, types::Path )

DEF_PUB( WPD, Speed, types::Speed )

DEF_PUB( VO, PosAttVel, types::Pose )

//----  SUB -----------------------

DEF_SUB( OCU, PosAttVel, LOC)
DEF_SUB( OCU, StatusData, SSM )
DEF_SUB( OCU, MissionStatus, SMME )
DEF_SUB( OCU, Map, MAP )
DEF_SUB( OCU, LocalPathPlan, PP )
DEF_SUB( OCU, IEDDetectionEvent, IEDSIM )
DEF_SUB( OCU, IEDLocation, IEDSIM )

DEF_SUB( IEDSIM, IEDDetectionEvent, OCU )
DEF_SUB( IEDSIM, IEDLocation, OCU )

DEF_SUB( SMME, IEDDetectionEvent, IEDSIM )
DEF_SUB( SMME, IEDLocation, IEDSIM )
DEF_SUB( SMME, MissionPlan, OCU )
DEF_SUB( SMME, StatusData, SSM )

DEF_SUB( SSM, MissionStatus, SMME )

DEF_SUB( WSM, ExecuteWorkSequenceCommand, SMME )
DEF_SUB( WSM, PosAttVel, LOC )
DEF_SUB( WSM, WiresLengths, PER )

DEF_SUB( LLC, TrottleEffort, WSM )
DEF_SUB( LLC, SteeringEffort, WSM )
DEF_SUB( LLC, JointsEffort, WSM )
DEF_SUB( LLC, Teleoperation, OCU )
DEF_SUB( LLC, Speed, WPD )

DEF_SUB( PL_R2U, TrottleEffort, LLC )
DEF_SUB( PL_R2U, SteeringEffort, LLC )
DEF_SUB( PL_R2U, JointsEffort, LLC )

DEF_SUB( MAP, BladePosition, WSM )
DEF_SUB( MAP, PosAttVel, LOC )
DEF_SUB( MAP, Laser, PER )
DEF_SUB( MAP, Camera, PER )

DEF_SUB( LOC, PosAttVel, VO )
DEF_SUB( LOC, PositionUpdate, OCU )

DEF_SUB( VO, Camera, PER )
DEF_SUB( VO, INS, PER )
DEF_SUB( VO, TF, PER )


DEF_SUB( PER, Sensor_SICK, SENSORS )
DEF_SUB( PER, Sensor_IBEO, SENSORS )
DEF_SUB( PER, Sensor_CAM_R, SENSORS )
DEF_SUB( PER, Sensor_CAM_L, SENSORS )
DEF_SUB( PER, Sensor_WIRE, SENSORS )
DEF_SUB( PER, Sensor_INS, SENSORS )
DEF_SUB( PER, Sensor_GPS, SENSORS )


DEF_SUB( SE_R2U, Sensor_WIRE, SENSORS )
DEF_SUB( SE_R2U, Sensor_GPS, SENSORS )
DEF_SUB( SE_R2U, Sensor_INS, SENSORS )

DEF_SUB( SE_R2RS, Sensor_SICK, SENSORS )
DEF_SUB( SE_R2RS, Sensor_IBEO, SENSORS )
DEF_SUB( SE_R2RS, Sensor_CAM_R, SENSORS )
DEF_SUB( SE_R2RS, Sensor_CAM_L, SENSORS )

DEF_SUB( PP, Map, MAP )
DEF_SUB( PP, MissionGlobalPath, SMME )
DEF_SUB( PP, IEDPosAtt, SMME )
DEF_SUB( PP, PosAttVel, LOC )

DEF_SUB( RPP, LocalPathPlan, PP )
DEF_SUB( RPP, MiniMap, MAP )
DEF_SUB( RPP, PosAttVel, LOC )

DEF_SUB( WPD, MiniMap, MAP )
DEF_SUB( WPD, LocalPathPlan, PP )
DEF_SUB( WPD, PosAttVel, LOC )


#undef DEF_PUB
#undef DEF_SUB
#endif /* PARAMETERTYPES_H_ */
