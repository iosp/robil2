#ifndef MAPPER__H
#define MAPPER__H

#include <std_msgs/String.h>
#include <ParameterTypes.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include "../helpermath.h"
#include "../heightmap.h"
#include "../heightmap_projection.h"
#include "../../roscomm/RosComm.h"

#define VISUALIZE_NONE		0
#define VISUALIZE_MAP		1
#define VISUALIZE_MINIMAP 	2
#define VISUALIZE_STEREO	4
#define VISUALIZE_TYPES		8
#define VISUALIZE_FULLMAP	16

using namespace cv;
//static singleton
class Mapper
{
  public:
    static void MainLoop();
    
    static void VisualizeLoop();
    
    static void handleIBEO(const config::PER::sub::SensorIBEO& msg);
     
    static void handleSickL(const config::PER::sub::SensorSICK1& msg);
    
    static void handleSickR(const config::PER::sub::SensorSICK2& msg);
    
    static void handleLocation(const config::PER::sub::Location& msg);
    
    static void handleCamR(const config::PER::sub::SensorCamR& msg);
    
    static void handleCamL(const config::PER::sub::SensorCamL& msg);
    
    static void publishMap();
    
    static void publishMiniMap();
    
    static void setVisualize(unsigned char flags);
    
    static Rotation myRot, ibeoRot, leftSickRot, rightSickRot;
    static Vec3D position;
    static Quaternion myQuat;
    static bool loc_received;
    static bool camL;
    static bool camR;
    static Mat camLImg, camRImg;
    static unsigned char visualize;
    static boost::mutex lock;
    static HeightMap* height_map;
    static RosComm* roscomm;
    static unsigned char flags;
};


#endif