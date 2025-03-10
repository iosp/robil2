#ifndef PARAMETERTYPES_H_
#define PARAMETERTYPES_H_
#include "ParameterTypesDefs.h"

//PUBLICATIONS
DEF_PUB( IEDSIM, IEDLocation, types::IEDLocation )

DEF_PUB( LLC, EffortsTh, types::Effort )
DEF_PUB( LLC, EffortsSt, types::Effort )
DEF_PUB( LLC, EffortsJn, types::Joints )

DEF_PUB( LLI, ConnectedToPlatform, std_msgs::Bool )

DEF_PUB( OCU, CustomIED, types::IEDLocation )
DEF_PUB( OCU, PositionUpdate, types::Pose )
DEF_PUB( OCU, AssignNavTask, types::NavTask )
DEF_PUB( OCU, AssignManTask, types::ManTask )
DEF_PUB( OCU, AssignMission, types::Mission )

DEF_PUB( PER, GPS, types::GPS )
DEF_PUB( PER, INS, types::Imu )
DEF_PUB( PER, BladePosition, types::BladPose )
DEF_PUB( PER, Map, types::Map )
DEF_PUB( PER, MiniMap, types::Map )
DEF_PUB( PER, VOOdometry, types::Odometry )
DEF_PUB( PER, PerGpsSpeed, types::GpsSpeed )

DEF_PUB( PP, LocalPath, types::Path )

DEF_PUB( SMME, GlobalPath, types::Path )
DEF_PUB( SMME, WorkSeqData, types::WSMData )
DEF_PUB( SMME, MissionAcceptance, types::MissionAcceptance )


DEF_PUB( WPD, WPDVelocity, types::Speed )

DEF_PUB( WSM, WSMVelocity, types::Speed )
DEF_PUB( WSM, BladePositionCommand, types::BladPose )

DEF_PUB( LOC, Location, types::PoseWithConf )
DEF_PUB( LOC, PerVelocity, types::Speed )

DEF_PUB( SENSORS, SensorINS, types::Imu )
DEF_PUB( SENSORS, SensorGPS, types::GPS )
DEF_PUB( SENSORS, SensorCamL, types::CamFrame )
DEF_PUB( SENSORS, SensorCamR, types::CamFrame )
DEF_PUB( SENSORS, SensorWire, types::WireLength )
DEF_PUB( SENSORS, SensorSICK1, types::Laser )
DEF_PUB( SENSORS, SensorSICK2, types::Laser )
DEF_PUB( SENSORS, SensorIBEO, types::MultiLaser )
DEF_PUB( SENSORS, SensorGpsSpeed, types::GpsSpeed )

DEF_PUB( SHIFFON2ROS, INS, types::Imu )
DEF_PUB( SHIFFON2ROS, GPS, types::GPS )
DEF_PUB( SHIFFON2ROS, GpsSpeed, types::GpsSpeed )

//SUBSCRIPTIONS
DEF_SUB( IEDSIM, CustomIED, OCU )
DEF_SUB( IEDSIM, Location, LOC )

DEF_SUB( LLC, WPDVelocity, WPD )
DEF_SUB( LLC, WSMVelocity, WSM )
DEF_SUB( LLC, BladePositionCommand, WSM )
DEF_SUB( LLC, Location, LOC )
DEF_SUB( LLC, PerVelocity, LOC )

DEF_SUB( LLI, EffortsTh, LLC )
DEF_SUB( LLI, EffortsSt, LLC )
DEF_SUB( LLI, EffortsJn, LLC )

DEF_SUB( OCU, IEDLocation, IEDSIM )
DEF_SUB( OCU, MissionAcceptance, SMME )
DEF_SUB( OCU, LocalPath, PP )
DEF_SUB( OCU, BladePosition, PER )
DEF_SUB( OCU, Map, PER )
DEF_SUB( OCU, Location, LOC )
DEF_SUB( OCU, PerVelocity, LOC )

DEF_SUB( PER, Location, LOC )
DEF_SUB( PER, PerVelocity, LOC )
DEF_SUB( PER, SensorINS, SENSORS )
DEF_SUB( PER, SensorGPS, SENSORS )
DEF_SUB( PER, SensorCamL, SENSORS )
DEF_SUB( PER, SensorCamR, SENSORS )
DEF_SUB( PER, SensorWire, SENSORS )
DEF_SUB( PER, SensorSICK1, SENSORS )
DEF_SUB( PER, SensorSICK2, SENSORS )
DEF_SUB( PER, SensorIBEO, SENSORS )
DEF_SUB( PER, SensorGpsSpeed, SENSORS )
DEF_SUB( PER, EffortsTh, LLC )
DEF_SUB( PER, EffortsSt, LLC )
DEF_SUB( PER, EffortsJn, LLC )

DEF_SUB( PP, GlobalPath, SMME )
DEF_SUB( PP, BladePosition, PER )
DEF_SUB( PP, Map, PER )
DEF_SUB( PP, Location, LOC )

DEF_SUB( SMME, AssignNavTask, OCU )
DEF_SUB( SMME, AssignManTask, OCU )
DEF_SUB( SMME, AssignMission, OCU )
DEF_SUB( SMME, BladePosition, PER )
DEF_SUB( SMME, Location, LOC )

DEF_SUB( SSM, BladePosition, PER )
DEF_SUB( SSM, Location, LOC )
DEF_SUB( SSM, PerVelocity, LOC )

DEF_SUB( WPD, LocalPath, PP )
DEF_SUB( WPD, MiniMap, PER )
DEF_SUB( WPD, Location, LOC )

DEF_SUB( WSM, WorkSeqData, SMME )
DEF_SUB( WSM, BladePosition, PER )
DEF_SUB( WSM, MiniMap , PER)

DEF_SUB( LOC, PositionUpdate, OCU )
DEF_SUB( LOC, GPS, PER )
DEF_SUB( LOC, INS, PER )
DEF_SUB( LOC, VOOdometry, PER )
DEF_SUB( LOC, PerGpsSpeed, PER )


DEF_SUB( PLATFORM, EffortsTh, LLC )
DEF_SUB( PLATFORM, EffortsSt, LLC )
DEF_SUB( PLATFORM, EffortsJn, LLC )

#undef DEF_PUB
#undef DEF_SUB
#endif
