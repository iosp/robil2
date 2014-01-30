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

typedef std_msgs::String TEMP_TYPE;

//----  PUB -----------------------

DEF_PUB( OCU, PositionUpdate, TEMP_TYPE )
DEF_PUB( OCU, MissionPlan, TEMP_TYPE )
DEF_PUB( OCU, Teleoperation, TEMP_TYPE )
DEF_PUB( OCU, IEDDetectionEvent, TEMP_TYPE )
DEF_PUB( OCU, IEDLocation, TEMP_TYPE )

DEF_PUB( IEDSIM, IEDDetectionEvent, TEMP_TYPE )
DEF_PUB( IEDSIM, IEDLocation, TEMP_TYPE )

DEF_PUB( SMME, MissionStatus, TEMP_TYPE )
DEF_PUB( SMME, MissionGlobalPath, TEMP_TYPE )
DEF_PUB( SMME, IEDPosAtt, TEMP_TYPE )
DEF_PUB( SMME, ExecuteWorkSequenceCommand, TEMP_TYPE )

DEF_PUB( SSM, StatusData, TEMP_TYPE )

DEF_PUB( WSM, BladePosition, TEMP_TYPE )
DEF_PUB( WSM, TrottleEffort, TEMP_TYPE )
DEF_PUB( WSM, SteeringEffort, TEMP_TYPE )
DEF_PUB( WSM, JointsEffort, TEMP_TYPE )

DEF_PUB( LLI, Teleoperation, TEMP_TYPE )
DEF_PUB( LLI, TrottleEffort, TEMP_TYPE )
DEF_PUB( LLI, SteeringEffort, TEMP_TYPE )
DEF_PUB( LLI, JointsEffort, TEMP_TYPE )

DEF_PUB( MAP, Map, TEMP_TYPE )
DEF_PUB( MAP, MiniMap, TEMP_TYPE )

DEF_PUB( LOC, PosAttVel, TEMP_TYPE )

DEF_PUB( PER, WiresLengths, TEMP_TYPE )
DEF_PUB( PER, Camera, TEMP_TYPE )
DEF_PUB( PER, Laser, TEMP_TYPE )
DEF_PUB( PER, INS, TEMP_TYPE )
DEF_PUB( PER, GPS, TEMP_TYPE )
DEF_PUB( PER, TF, TEMP_TYPE )

DEF_PUB( SENSORS, Sensor_SICK, TEMP_TYPE )
DEF_PUB( SENSORS, Sensor_IBEO, TEMP_TYPE )
DEF_PUB( SENSORS, Sensor_CAM_R, TEMP_TYPE )
DEF_PUB( SENSORS, Sensor_CAM_L, TEMP_TYPE )
DEF_PUB( SENSORS, Sensor_WIRE, TEMP_TYPE )
DEF_PUB( SENSORS, Sensor_INSGPS, TEMP_TYPE )

DEF_PUB( PP, LocalPathPlan, TEMP_TYPE )

DEF_PUB( RPP, RPPPath, TEMP_TYPE )

DEF_PUB( WPD, TrottleEffort, TEMP_TYPE )
DEF_PUB( WPD, SteeringEffort, TEMP_TYPE )

DEF_PUB( VO, PosAttVel, TEMP_TYPE )

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


#undef DEF_PUB
#undef DEF_SUB
#endif /* PARAMETERTYPES_H_ */
