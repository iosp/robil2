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
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <robil_msgs/Path.h>
#include <robil_msgs/IEDLocation.h>
#include <robil_msgs/CommandsList.h>
#include <robil_msgs/Map.h>
#include <robil_msgs/WPDDecision.h>
#include <robil_msgs/ControlConfirm.h>
#include <robil_msgs/ControlRequest.h>

#define DEF_PUB( COMP, TOPIC, TYPE )\
	namespace config{ namespace COMP{ namespace pub{ typedef TYPE TOPIC; } } }
#define DEF_SUB( COMP, TOPIC, COMP_SOURCE )\
	namespace config{ namespace COMP{ namespace sub{ typedef config::COMP_SOURCE::pub::TOPIC TOPIC; } } }

typedef std_msgs::String TEMP_TYPE;

namespace types{
	typedef geometry_msgs::PointStamped Point;
	typedef geometry_msgs::PoseStamped Pose;
	typedef robil_msgs::Path Path;
	typedef nav_msgs::Odometry PoseVel;
	typedef sensor_msgs::LaserScan Laser;
	typedef sensor_msgs::Image CamFrame;
	typedef sensor_msgs::CameraInfo CamInfo;
	typedef sensor_msgs::Imu IMU;
	typedef sensor_msgs::NavSatFix GPS;
	typedef sensor_msgs::NavSatStatus GPSStatus;
	typedef robil_msgs::Map Map;
	typedef std_msgs::Float64 WireLength;
	typedef geometry_msgs::TwistStamped Speed;
	typedef std_msgs::Int32 Effort;
	typedef robil_msgs::IEDLocation IEDLocation;
	typedef std_msgs::Bool IEDDetection; // or if IEDLocation is a special type then IEDDetection=IEDLocation
	typedef std_msgs::Int32 Mode;
	typedef std_msgs::String Events;
	typedef std_msgs::String Command;
	typedef std_msgs::String Task;
	typedef diagnostic_msgs::DiagnosticStatus Diagnostic;
	typedef robil_msgs::CommandsList CommandsList;
	typedef std_msgs::String WSMData; // need custom message
	typedef robil_msgs::WPDDecision WPDDecision; // need custom message
}

//----  PUBLICATIONS -----------------------

DEF_PUB( OCU, PositionUpdate, types::Pose )
DEF_PUB( OCU, SMMECommand, types::Command )
DEF_PUB( OCU, SMMEControlMode, types::Mode )
DEF_PUB( OCU, NavigationTask, types::Task )
DEF_PUB( OCU, ManipulationTask, types::Task )
DEF_PUB( OCU, MissionTask, types::Task )
DEF_PUB( OCU, CustomPath, types::Path )
DEF_PUB( OCU, TeleoperationControlMode, types::Mode )
DEF_PUB( OCU, TeleoperationThrottle, types::Effort )
DEF_PUB( OCU, TeleoperationSteering, types::Effort )
DEF_PUB( OCU, TeleoperationJoints, types::Effort )
DEF_PUB( OCU, IEDDetectionEvent, types::IEDDetection )
DEF_PUB( OCU, IEDLocation, types::IEDLocation )

DEF_PUB( IEDSIM, IEDDetectionEvent, types::IEDDetection )
DEF_PUB( IEDSIM, IEDLocation, types::IEDLocation )
DEF_PUB( IEDSIM, IEDSIMState, types::Diagnostic )
DEF_PUB( IEDSIM, IEDSIMComponentState, types::Diagnostic )

DEF_PUB( SMME, WSMData, types::WSMData )
DEF_PUB( SMME, MissionAcceptance, types::Task )
DEF_PUB( SMME, GlobalPath, types::Path )
DEF_PUB( SMME, MaxSpeed, types::Speed )
DEF_PUB( SMME, SMMEMissionState, types::Diagnostic )
DEF_PUB( SMME, SMMEComponentState, types::Diagnostic )
DEF_PUB( SMME, WPDCommand, types::Command )
DEF_PUB( SMME, PPCommand, types::Command )
DEF_PUB( SMME, IEDSIMCommand, types::Command )
DEF_PUB( SMME, SSMCommand, types::Command )
DEF_PUB( SMME, WSMCommand, types::Command )
DEF_PUB( SMME, LLCCommand, types::Command )
DEF_PUB( SMME, LOCCommand, types::Command )
DEF_PUB( SMME, PERCommand, types::Command )

DEF_PUB( SSM, PlatformState, types::Diagnostic )
DEF_PUB( SSM, SoftwareState, types::Diagnostic )
DEF_PUB( SSM, SensorsState, types::Diagnostic )
DEF_PUB( SSM, SSMComponentState, types::Diagnostic )

DEF_PUB( WSM, BladePositionCommand, types::Pose )
DEF_PUB( WSM, WSMSpeed, types::Speed )
DEF_PUB( WSM, WSMExecutionState, types::Diagnostic )
DEF_PUB( WSM, WSMComponentState, types::Diagnostic )

DEF_PUB( LLC, TrottleEffort, types::Effort )
DEF_PUB( LLC, SteeringEffort, types::Effort )
DEF_PUB( LLC, JointsEffort, types::Effort )
DEF_PUB( LLC, LLCPlatformState, types::Diagnostic )
DEF_PUB( LLC, LLCComponentState, types::Diagnostic )

DEF_PUB( LOC, Odometry, types::PoseVel )
DEF_PUB( LOC, LOCState, types::Diagnostic )
DEF_PUB( LOC, LOCComponentState, types::Diagnostic )

DEF_PUB( PER, MiniMap, types::Map )
DEF_PUB( PER, Map, types::Map )
DEF_PUB( PER, INS, types::IMU )
DEF_PUB( PER, GPS, types::GPS )
DEF_PUB( PER, BladPosition,  types::Pose)
DEF_PUB( PER, VOOdometry, types::PoseVel )
DEF_PUB( PER, SensorsCommand,  types::Command)
DEF_PUB( PER, PERSensorsState,  types::Diagnostic)
DEF_PUB( PER, PERComponentState,  types::Diagnostic)

DEF_PUB( SENSORS, Sensor_SICK_1, types::Laser )
DEF_PUB( SENSORS, Sensor_SICK_2, types::Laser )
DEF_PUB( SENSORS, Sensor_IBEO_1, types::Laser )
DEF_PUB( SENSORS, Sensor_IBEO_2, types::Laser )
DEF_PUB( SENSORS, Sensor_IBEO_3, types::Laser )
DEF_PUB( SENSORS, Sensor_IBEO_4, types::Laser )
DEF_PUB( SENSORS, Sensor_CAM_R, types::CamFrame )
DEF_PUB( SENSORS, Sensor_CAM_L, types::CamFrame )
DEF_PUB( SENSORS, Sensor_WIRE, types::WireLength )
DEF_PUB( SENSORS, Sensor_GPS, types::GPS )
DEF_PUB( SENSORS, Sensor_INS, types::IMU )

DEF_PUB( PP, LocalPath, types::Path )
DEF_PUB( PP, PPExecutionState, types::Diagnostic )
DEF_PUB( PP, PPComponentState, types::Diagnostic )

DEF_PUB( WPD, WPDSpeed, types::Speed )
DEF_PUB( WPD, WPDDecisions, types::WPDDecision )
DEF_PUB( WPD, WPDState, types::Diagnostic )
DEF_PUB( WPD, WPDComponentState, types::Diagnostic )

//----  SUBSCRIPTIONS -----------------------

DEF_SUB( OCU, Odometry, LOC )
DEF_SUB( OCU, BladPosition, PER )
DEF_SUB( OCU, Map, PER )
DEF_SUB( OCU, WPDDecisions, WPD )
DEF_SUB( OCU, LocalPath, PP )
DEF_SUB( OCU, IEDDetectionEvent, IEDSIM )
DEF_SUB( OCU, PlatformState, SSM )
DEF_SUB( OCU, SoftwareState, SSM )
DEF_SUB( OCU, SensorsState, SSM )
DEF_SUB( OCU, SMMEMissionState, SMME )
DEF_SUB( OCU, MissionAcceptance, SMME )

DEF_SUB( IEDSIM, Odometry, LOC )
DEF_SUB( IEDSIM, IEDDetectionEvent, OCU )
DEF_SUB( IEDSIM, IEDLocation, OCU )
DEF_SUB( IEDSIM, IEDSIMCommand, SMME )

DEF_SUB( SMME, Odometry, LOC )
DEF_SUB( SMME, BladPosition, PER )
DEF_SUB( SMME, PPExecutionState, PP )
DEF_SUB( SMME, WSMExecutionState, WSM )
DEF_SUB( SMME, IEDDetectionEvent, IEDSIM )
DEF_SUB( SMME, IEDLocation, IEDSIM )
DEF_SUB( SMME, PlatformState, SSM )
DEF_SUB( SMME, SoftwareState, SSM )
DEF_SUB( SMME, SensorsState, SSM )
DEF_SUB( SMME, SMMECommand, OCU )
DEF_SUB( SMME, SMMEControlMode, OCU )
DEF_SUB( SMME, NavigationTask, OCU )
DEF_SUB( SMME, ManipulationTask, OCU )
DEF_SUB( SMME, MissionTask, OCU )

DEF_SUB( SSM, LLCPlatformState, LLC )
DEF_SUB( SSM, Odometry, LOC )
DEF_SUB( SSM, BladPosition, PER )
DEF_SUB( SSM, PERSensorsState, PER )
DEF_SUB( SSM, WPDDecisions, WPD )
DEF_SUB( SSM, WSMExecutionState, WSM )
DEF_SUB( SSM, SMMEMissionState, SMME )
DEF_SUB( SSM, IEDSIMComponentState, IEDSIM )
DEF_SUB( SSM, SMMEComponentState, SMME )
DEF_SUB( SSM, SSMComponentState, SSM )
DEF_SUB( SSM, LLCComponentState, LLC )
DEF_SUB( SSM, LOCComponentState, LOC )
DEF_SUB( SSM, PERComponentState, PER )
DEF_SUB( SSM, PPComponentState, PP )
DEF_SUB( SSM, WPDComponentState, WPD )
DEF_SUB( SSM, SSMCommand, SMME )

DEF_SUB( WSM, Odometry, LOC )
DEF_SUB( WSM, BladPosition, PER )
DEF_SUB( WSM, WSMCommand, SMME )
DEF_SUB( WSM, WSMData, SMME )

DEF_SUB( LLC, WSMSpeed, WSM )
DEF_SUB( LLC, BladePositionCommand, WSM )
DEF_SUB( LLC, WPDSpeed, WPD )
DEF_SUB( LLC, Odometry, LOC )
DEF_SUB( LLC, LLCCommand, SMME )
DEF_SUB( LLC, TeleoperationControlMode, OCU )
DEF_SUB( LLC, TeleoperationThrottle, OCU )
DEF_SUB( LLC, TeleoperationSteering, OCU )
DEF_SUB( LLC, TeleoperationJoints, OCU )

DEF_SUB( LOC, VOOdometry, PER )
DEF_SUB( LOC, GPS, PER )
DEF_SUB( LOC, INS, PER )
DEF_SUB( LOC, PositionUpdate, OCU )
DEF_SUB( LOC, LOCCommand, SMME )

DEF_SUB( PER, Sensor_SICK_1, SENSORS )
DEF_SUB( PER, Sensor_SICK_2, SENSORS )
DEF_SUB( PER, Sensor_IBEO_1, SENSORS )
DEF_SUB( PER, Sensor_IBEO_2, SENSORS )
DEF_SUB( PER, Sensor_IBEO_3, SENSORS )
DEF_SUB( PER, Sensor_IBEO_4, SENSORS )
DEF_SUB( PER, Sensor_CAM_R, SENSORS )
DEF_SUB( PER, Sensor_CAM_L, SENSORS )
DEF_SUB( PER, Sensor_WIRE, SENSORS )
DEF_SUB( PER, Sensor_INS, SENSORS )
DEF_SUB( PER, Sensor_GPS, SENSORS )
DEF_SUB( PER, Odometry, LOC )
DEF_SUB( PER, TrottleEffort, LLC )
DEF_SUB( PER, SteeringEffort, LLC )
DEF_SUB( PER, JointsEffort, LLC )
DEF_SUB( PER, PERCommand, SMME )

DEF_SUB( PP, Map, PER )
DEF_SUB( PP, Odometry, LOC )
DEF_SUB( PP, PPCommand, SMME )
DEF_SUB( PP, GlobalPath, SMME )
DEF_SUB( PP, MaxSpeed, SMME )

DEF_SUB( WPD, MiniMap, PER )
DEF_SUB( WPD, LocalPath, PP )
DEF_SUB( WPD, MaxSpeed, SMME )
DEF_SUB( WPD, Odometry, LOC )
DEF_SUB( WPD, WPDCommand, SMME )


#undef DEF_PUB
#undef DEF_SUB
#endif /* PARAMETERTYPES_H_ */
