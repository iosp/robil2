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

#define DEF_PUB( COMP, TOPIC, TYPE )\
	namespace config{ namespace COMP{ namespace pub{ typedef TYPE TOPIC; } } }
#define DEF_SUB( COMP, TOPIC, COMP_SOURCE )\
	namespace config{ namespace COMP{ namespace sub{ typedef config::COMP_SOURCE::pub::TOPIC TOPIC; } } }

//----  PUB -----------------------

DEF_PUB( OCU, PositionUpdate, std_msgs::String )
DEF_PUB( OCU, MissionPlan, std_msgs::String )
DEF_PUB( OCU, Teleoperation, std_msgs::String )
DEF_PUB( OCU, IEDDetectionEvent, std_msgs::String )
DEF_PUB( OCU, IEDLocation, std_msgs::String )

DEF_PUB( IEDSIM, IEDDetectionEvent, std_msgs::String )
DEF_PUB( IEDSIM, IEDLocation, std_msgs::String )

DEF_PUB( SMME, MissionStatus, std_msgs::String )
DEF_PUB( SMME, MissionGlobalPath, std_msgs::String )
DEF_PUB( SMME, IEDPosAtt, std_msgs::String )
DEF_PUB( SMME, ExecuteWorkSequenceCommand, std_msgs::String )

DEF_PUB( SSM, StatusData, std_msgs::String )

DEF_PUB( WSM, BladePosition, std_msgs::String )
DEF_PUB( WSM, TrottleEffort, std_msgs::String )
DEF_PUB( WSM, SteeringEffort, std_msgs::String )
DEF_PUB( WSM, JointsEffort, std_msgs::String )

DEF_PUB( LLI, Teleoperation, std_msgs::String )
DEF_PUB( LLI, TrottleEffort, std_msgs::String )
DEF_PUB( LLI, SteeringEffort, std_msgs::String )
DEF_PUB( LLI, JointsEffort, std_msgs::String )

DEF_PUB( MAP, Map, std_msgs::String )
DEF_PUB( MAP, MiniMap, std_msgs::String )

DEF_PUB( LOC, PosAttVel, std_msgs::String )

DEF_PUB( PER, WiresLengths, std_msgs::String )
DEF_PUB( PER, Camera, std_msgs::String )
DEF_PUB( PER, Laser, std_msgs::String )
DEF_PUB( PER, INS, std_msgs::String )
DEF_PUB( PER, GPS, std_msgs::String )
DEF_PUB( PER, TF, std_msgs::String )

DEF_PUB( SENSORS, Sensor_SICK, std_msgs::String )
DEF_PUB( SENSORS, Sensor_IBEO, std_msgs::String )
DEF_PUB( SENSORS, Sensor_CAM_R, std_msgs::String )
DEF_PUB( SENSORS, Sensor_CAM_L, std_msgs::String )
DEF_PUB( SENSORS, Sensor_WIRE, std_msgs::String )
DEF_PUB( SENSORS, Sensor_INSGPS, std_msgs::String )

DEF_PUB( PP, LocalPathPlan, std_msgs::String )

DEF_PUB( RPP, RPPPath, std_msgs::String )

DEF_PUB( WPD, TrottleEffort, std_msgs::String )
DEF_PUB( WPD, SteeringEffort, std_msgs::String )

DEF_PUB( VO, PosAttVel, std_msgs::String )

//----  SUB -----------------------

DEF_SUB( OCU, PosAttVel, LOC)
DEF_SUB( OCU, Teleoperation, LLI )
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

DEF_SUB( LLI, TrottleEffort, WSM )
DEF_SUB( LLI, SteeringEffort, WSM )
DEF_SUB( LLI, JointsEffort, WSM )
DEF_SUB( LLI, Teleoperation, OCU )

DEF_SUB( PL_R2U, TrottleEffort, LLI )
DEF_SUB( PL_R2U, SteeringEffort, LLI )
DEF_SUB( PL_R2U, JointsEffort, LLI )

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
DEF_SUB( PER, Sensor_INSGPS, SENSORS )


DEF_SUB( SE_R2U, Sensor_WIRE, SENSORS )
DEF_SUB( SE_R2U, Sensor_INSGPS, SENSORS )

DEF_SUB( SE_R2RS, Sensor_SICK, SENSORS )
DEF_SUB( SE_R2RS, Sensor_IBEO, SENSORS )
DEF_SUB( SE_R2RS, Sensor_CAM_R, SENSORS )
DEF_SUB( SE_R2RS, Sensor_CAM_L, SENSORS )

DEF_SUB( PP, Map, MAP )
DEF_SUB( PP, MissionGlobalPath, SMME )
DEF_SUB( PP, IEDPosAtt, SMME )
DEF_SUB( PP, PosAttVel, LOC )
DEF_SUB( PP, RPPPath, RPP )

DEF_SUB( RPP, LocalPathPlan, PP )
DEF_SUB( RPP, MiniMap, MAP )
DEF_SUB( RPP, PosAttVel, LOC )

DEF_SUB( WPD, RPPPath, RPP )
DEF_SUB( WPD, PosAttVel, LOC )


#endif /* PARAMETERTYPES_H_ */
