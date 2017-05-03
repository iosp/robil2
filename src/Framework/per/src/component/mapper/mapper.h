#ifndef MAPPER__H
#define MAPPER__H

#include <std_msgs/String.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include "../helpermath.h"
#include "../heightmap.h"
#include "../heightmap_projection.h"
#include "../ComponentMain.h"
#include "per/configConfig.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#define VISUALIZE_NONE		0
#define VISUALIZE_MAP		1
#define VISUALIZE_MINIMAP 	2
#define VISUALIZE_STEREO	4
#define VISUALIZE_TYPES		8
#define VISUALIZE_FULLMAP	16

/// walrus includes:
// #include <per/roadLanes.h>
// #include <per/lane.h>
#include <vector>


#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <robil_msgs/MultiLaserScan.h>

#ifndef HEARTBEAT_FREQUANCY
#define HEARTBEAT_FREQUANCY 2 //Hz
#endif

#ifndef HEARTBEAT_FREQUENCY
#define HEARTBEAT_FREQUENCY 2 //Hz
#endif


using namespace std;
// using namespace per;
/// until here

using namespace cv;
//static singleton
class Mapper
{
  public:
    static void MainLoop(per::configConfig *p);
    
    /// walrus function:
    static void StereoThread();
    /// 'till here
    
    static void VisualizeLoop();
    
    static void handleIBEO(const robil_msgs::MultiLaserScan& msg, ros::Publisher pcpubworld, ros::Publisher pcpub);
     
    static void handleSickL(const sensor_msgs::LaserScan& msg);
    
    static void handleSickR(const sensor_msgs::LaserScan& msg);
    
    static void handleLocation(const nav_msgs::Odometry& msg);
    
    static void handleCamR(const sensor_msgs::Image& msg);
    
    static void handleCamL(const sensor_msgs::Image& msg);
    
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
    static ComponentMain* component;
    static unsigned char flags;
    //static tf::TransformListener *listener;
    /**
     * Walrus Changes:
     */
    static void setLanes(Mat lanes);

};


#endif
