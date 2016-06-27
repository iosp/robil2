
#ifndef GAZEBO_ROBIL_IBEO_PLUGIN
#define GAZEBO_ROBIL_IBEO_PLUGIN


//#include "ibeo/MultiLaserScan.h"
#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
//#include <string>

#include <visualization_msgs/Marker.h>

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>

#include <boost/thread.hpp>
#include <atomic>

#define PI 			3.14159265359

    /* NEW  */

#pragma pack(1)

typedef struct
{
	 unsigned int   	MagicWord;
	 unsigned int       SizePreviousMessage;
	 unsigned int	    SizeCurrentMessage;
	 unsigned char		Reserved;
	 unsigned char	    DeviceID;
	 unsigned short		DataType;
	 unsigned int      	time_up;	// NTP64    in seconds
	 unsigned int      	time_down;  // NTP64   after the floathing point

}ibeoScanDataHeader;

typedef struct
{
	unsigned char   	Layer_Echo;
	unsigned char   	Flags;
	short           	HorizontalAngel;
	unsigned short  	RadialDistance;
	unsigned short  	EchoPulseWidth;
	unsigned short  	Reserved;
}IbeoScanPoint;

typedef struct
{
	 unsigned short   	ScanNumber;
	 unsigned short   	ScannerStatus;
	 unsigned short	  	SyncPhaseOffset;
	 unsigned int    	ScanStratTimeDOWN;	// NTP64   after the floathing point
	 unsigned int    	ScanStratTimeUP;		// NTP64   in seconds
	 unsigned int    	ScanEndTimeDOWN;
	 unsigned int    	ScanEndTimeUP;
	 unsigned short   	AngelsTicks;
	 short			  	StartAngel;
	 short		      	EndAngel;
	 unsigned short   	ScanPoints;
	 short			  	PositionYaw;
	 short			  	PositionPitch;
	 short            	PositionRoll;
	 short			  	PositionX;
	 short			  	PositionY;
	 short			  	PositionZ;
	 unsigned short   	Reserved;
}IbeoScanHeader;


typedef struct
{
	 unsigned short   	ScanNumber;
	 unsigned short   	ScannerStatus;
	 unsigned short	  	SyncPhaseOffset;
	 unsigned int    	ScanStratTimeDOWN;	// NTP64   after the floathing point
	 unsigned int    	ScanStratTimeUP;		// NTP64   in seconds
	 unsigned int	    ScanEndTimeDOWN;
	 unsigned int   	ScanEndTimeUP;
	 unsigned short   	AngelsTicks;
	 short			  	StartAngel;
	 short		      	EndAngel;
	 unsigned short   	ScanPoints;
	 short			  	PositionYaw;
	 short			  	PositionPitch;
	 short            	PositionRoll;
	 short			  	PositionX;
	 short			  	PositionY;
	 short			  	PositionZ;
	 unsigned short   	Reserved;
}IbeoScanHeaderTEST;

typedef struct
{
	ibeoScanDataHeader  Header;
	IbeoScanHeader		Scan;
	IbeoScanPoint*		Point;
}SibeoScanData;

typedef struct
{
	ibeoScanDataHeader  Header;
	unsigned short commandID;
//	unsigned short reserved;
}ReplayMSG;

#endif
