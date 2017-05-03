#include "mapper.h"
#include <stdio.h>
#include <stdlib.h>
#include <boost/thread.hpp>
#include "../stereo.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/ChannelFloat32.h>

bool Mapper::camL;
bool Mapper::camR;
bool Mapper::loc_received;
unsigned char Mapper::visualize;
boost::mutex Mapper::lock; 
boost::mutex disparity;
boost::mutex road_mutex;
Rotation Mapper::myRot, Mapper::ibeoRot, Mapper::leftSickRot, Mapper::rightSickRot;
Vec3D Mapper::position;
Quaternion Mapper::myQuat;
Mat Mapper::camLImg, Mapper::camRImg;
Mat stereo;
HeightMap* Mapper::height_map;
//RosComm* Mapper::roscomm;
ComponentMain* Mapper::component;

//tf::TransformListener* Mapper::listener = NULL;

/// walrus declares:
Mat _lanes;
#define IBEO_PITCH 0.210
#define IBEO_X -0.375
#define IBEO_Y 0.055
#define IBEO_Z 1.89
#define GPS_Z 0.5
/// Until here

void Mapper::MainLoop(per::configConfig *p)
{  
    camL = false;
    camR = false;
    loc_received = false;
    visualize = 0x00;
    height_map = new HeightMap(500,500, p);
    per::configConfig *dynamic_param = p;
    int i = 0;
    ros::Duration rate(1);
    while(ros::ok())
    {

//        boost::this_thread::sleep(boost::posix_time::milliseconds(100)); //10hz cycle
        rate.sleep();
        int check; ros::param::param("/LOC/Ready",check,0); if(!check) continue;
        if(++i < 30) continue;
        //printf("MAPPER\n");
        lock.lock();

        height_map->calculateTypes();//position, myRot);

//        if(camL && camR)
//        {
//            camL = camR = false;
//            Quaternion& q = myQuat;
//            Vec3D front = GetFrontVector(q.x,q.y,q.z,q.w);
//            Vec3D right = GetRightVector(q.x,q.y,q.z,q.w);
//            Vec3D up = GetUpVector(q.x,q.y,q.z,q.w);
//            //ProjectDepthImage(height_map, stereo, right, front, up, position.add(up.multiply(1.6)), _lanes);
//        }

        publishMap();
        publishMiniMap();
        if (dynamic_param->resetMap)
        {
            delete(height_map);
            height_map = new HeightMap(500,500, p);
        }
        lock.unlock();
    }
    delete(height_map);
}

void Mapper::VisualizeLoop()
{
    while(ros::ok())
    {
        if(height_map != NULL)// && visualize > 0)
        {
            lock.lock();
            bool debug;
            ros::param::param("/PER/DEBUG", debug, false);
            if (debug)
            {
                HeightMap m = height_map->deriveMap(position.x, position.y, myRot);
                Mat im = m.generateMat(0,-5,0,1, true);
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im).toImageMsg();
                HeightMap m2 = height_map->deriveMap(position.x, position.y, myRot);
                Mat im2 = m.generateMat(1);
                sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im2).toImageMsg();
                component->publishDebug(msg, msg2);
            }

            if((visualize & VISUALIZE_MAP) != 0) //map needed
            {
                HeightMap m = height_map->deriveMap(position.x, position.y, myRot);
                m.displayGUI(0,-5,0);
            }
            if((visualize & VISUALIZE_MINIMAP) != 0 ) //mini-map needed
            {
                HeightMap m = height_map->deriveMiniMap(position.x, position.y, myRot);
                m.displayGUI(0,-4.5,0);
            }
            if((visualize & VISUALIZE_TYPES) != 0) //map types needed
            {
                HeightMap m = height_map->deriveMap(position.x, position.y, myRot);
                m.displayTypesGUI(_lanes);
            }
            if((visualize & VISUALIZE_FULLMAP) != 0) //global map needed
            {
                height_map->displayGUI(myRot.yaw*180/3.14159, position.x, position.y, 2);
            }
            if((visualize & VISUALIZE_STEREO) != 0) //disparity needed
            {
                if (!stereo.empty())
                    imshow("stereo", stereo);
            }
            if (!visualize)
                destroyAllWindows();
            lock.unlock();
            waitKey(20);
        }
    }
}

/**
 * Walrus Cahnges:
 */

void Mapper::setLanes(Mat lanes)
{
    road_mutex.lock();
    _lanes = lanes;
    road_mutex.unlock();
}

/** Until Here**/
void Mapper::handleIBEO(const robil_msgs::MultiLaserScan& msg, ros::Publisher pcpubworld, ros::Publisher pcpub)
{
    if(!loc_received) return;
    //return;
    static tf::TransformListener listener;
    lock.lock();

    ros::Time now = ros::Time(0);
    sensor_msgs::PointCloud ibeo_points, base_point;
    ibeo_points.channels.resize(1);
//    ibeo_points.channels[0].values.resize(size);
//    ibeo_points.channels[0].name = "intensities";
//    ibeo_points.points.resize(size);
    ibeo_points.header.frame_id = "IBEO";
    ibeo_points.header.stamp = now;

    int k = 0;
    float incrtop = msg.angle_increment, min_ang;
    for (int ray = 0;ray < 4; ray++)
    {
        vector<float> array;
        float phi;
        if (ray == 0)
            array = msg.ranges_t1, phi = msg.angle_t1, min_ang = msg.angle_min_t;
        else if (ray == 1)
            array = msg.ranges_t2, phi = msg.angle_t2, min_ang = msg.angle_min_t;
        else if (ray == 2)
            array = msg.ranges_b1, phi = msg.angle_b1, min_ang = msg.angle_min_b;
        else if (ray == 3)
            array = msg.ranges_b2, phi = msg.angle_b2, min_ang = msg.angle_min_b;
//        cout << "ray: " << ray << "   " << array.size() << " =?= " << (msg.angle_max_t - msg.angle_min_t) / msg.angle_increment << endl;

        for(int i = 0; i < array.size(); i++)
        {
            if (array[i] == 0.0)
                continue;

            geometry_msgs::Point32 p;
            float theta = min_ang + i * incrtop;
            float alfa = 1/(sqrt(1+pow(cos(phi),2)*pow(tan(theta),2)));
            p.x = array[i] * alfa * cos(phi);
            p.y = p.x * tan(theta);
            p.z = array[i] * alfa * sin(phi);
            ibeo_points.points.push_back(p);
            ibeo_points.channels[0].values.push_back(100.0);
        }
    }
    try{
//        listener.waitForTransform("WORLD", "IBEO", now, ros::Duration(0.5));
        listener.transformPointCloud("WORLD", ibeo_points, base_point);
        geometry_msgs::PoseStamped tracks_height, world_height;
        world_height.header.frame_id = "TRACKS_BOTTOM";
        world_height.header.stamp = now;
        world_height.pose.orientation.w = 1;
//        listener.waitForTransform("WORLD", "TRACKS_BOTTOM", now, ros::Duration(0.5));
        listener.transformPose("WORLD", world_height, tracks_height);
        pcpubworld.publish(base_point);
        pcpub.publish(ibeo_points);
        ProjectLaserRange(height_map, &base_point, tracks_height.pose.position.z);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("PER: %s", ex.what());
    }

    lock.unlock();
}

void Mapper::handleSickL(const sensor_msgs::LaserScan& msg)
{
    if(!loc_received) return;
    lock.lock();
    Quaternion q = GetFromRPY(leftSickRot);
    Vec3D front = GetFrontVector(q.x,q.y,q.z,q.w);
    Vec3D right = GetRightVector(q.x,q.y,q.z,q.w);
    Vec3D up = GetUpVector(q.x,q.y,q.z,q.w);

    Vec3D pos = position.add(front.multiply(-0.2187)).add(right.multiply(0.85)).add(up.multiply(0.631));

    for(int i = 0; i < msg.ranges.size(); i++)
        if(msg.ranges[i] < 0.5*msg.range_max && msg.ranges[i] > 2)
            ProjectLaserRange(
                        height_map,
                        right,
                        front,
                        pos,
                        msg.ranges[i],
                        msg.angle_min + i*msg.angle_increment);
    lock.unlock();
}

void Mapper::handleSickR(const sensor_msgs::LaserScan& msg)
{
    if(!loc_received) return;
    lock.lock();

    Quaternion q = GetFromRPY(rightSickRot);
    Vec3D front = GetFrontVector(q.x,q.y,q.z,q.w);
    Vec3D right = GetRightVector(q.x,q.y,q.z,q.w);
    Vec3D up = GetUpVector(q.x,q.y,q.z,q.w);

    Vec3D pos = position.add(front.multiply(-0.2187)).add(right.multiply(-0.85)).add(up.multiply(0.631));//why these values?

    for(int i = 0; i < msg.ranges.size(); i++)
        if(msg.ranges[i] < 0.5*msg.range_max && msg.ranges[i]>2)
            ProjectLaserRange(
                        height_map,
                        right,
                        front,
                        pos,
                        msg.ranges[i],
                        msg.angle_min + i*msg.angle_increment);

    lock.unlock();
}

void Mapper::handleLocation(const nav_msgs::Odometry& msg)
{
    lock.lock();
    geometry_msgs::Pose pose = msg.pose.pose;
    position = Vec3D(pose.position.x, pose.position.y, pose.position.z);
    myQuat = Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    myRot = GetRotation(myQuat);
    //if (myRot.yaw < 0)
    //  myRot.yaw += 3.14159;
    ibeoRot = myRot.add(Rotation(0, IBEO_PITCH, -0));
    //ROS_INFO("Ibeo rot: %.2f", ibeoRot.yaw * 180 / 3.14159);
    leftSickRot = Rotation(myRot.pitch, myRot.roll, myRot.yaw+1.57);
    rightSickRot = Rotation(-myRot.pitch, -myRot.roll, myRot.yaw-1.57);
    loc_received = true;

    lock.unlock();

}

void Mapper::handleCamR(const sensor_msgs::Image& msg)
{
    lock.lock();
    camR = true;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        lock.unlock();
        return;
    }
    //rdbg("leftcam");
    camRImg = cv_ptr->image;


    lock.unlock();
}

void Mapper::handleCamL(const sensor_msgs::Image& msg)
{
    lock.lock();
    camL = true;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        lock.unlock();
        return;
    }
    //rdbg("leftcam");
    camLImg = cv_ptr->image;

    lock.unlock();
}

void Mapper::publishMap()
{
    robil_msgs::Map msg;
    static int seq = 0;
    msg.header.seq = seq++;
    msg.header.stamp.sec = ros::Time::now().sec;
    msg.header.stamp.nsec = ros::Time::now().nsec;
    msg.header.frame_id = "map";

    msg.info.map_load_time.sec = ros::Time::now().sec;
    msg.info.map_load_time.nsec = ros::Time::now().nsec;
    msg.info.width = 150;
    msg.info.height = 150;
    msg.info.resolution = 0.2;
    msg.info.origin.position.x = position.x;
    msg.info.origin.position.y = position.y;
    msg.info.origin.position.z = position.z;
    msg.info.origin.orientation.x = myQuat.x;
    msg.info.origin.orientation.y = myQuat.y;
    msg.info.origin.orientation.z = myQuat.z;
    msg.info.origin.orientation.w = myQuat.w;
    msg.data.resize(150*150);

    HeightMap Oded = height_map->deriveMap(position.x, position.y, myRot);
    vector<double>& heights = Oded.getHeights();
    vector<int>& types = Oded.getTypes();
    vector<int>& features = Oded.getFeatures();
    for(int i = 0; i < 150; i++)
        for(int j = 0; j < 150; j++)
        {
            msg.data[j*150+i].height = heights[j*150+i];
            msg.data[j*150+i].type = types[j*150+i];
            msg.data[j*150+i].feature = features[j*150+i];
        }
    component->publishMap(msg);

}

void Mapper::publishMiniMap()
{
    robil_msgs::Map msg;
    static int seq = 0;

    msg.header.seq = seq++;
    msg.header.stamp.sec = ros::Time::now().sec;
    msg.header.stamp.nsec = ros::Time::now().nsec;
    msg.header.frame_id = "map";

    msg.info.map_load_time.sec = ros::Time::now().sec;
    msg.info.map_load_time.nsec = ros::Time::now().nsec;
    msg.info.width = 50;
    msg.info.height = 30;
    msg.info.resolution = 0.2;
    msg.info.origin.position.x = position.x;
    msg.info.origin.position.y = position.y;
    msg.info.origin.position.z = position.z;
    msg.info.origin.orientation.x = myQuat.x;
    msg.info.origin.orientation.y = myQuat.y;
    msg.info.origin.orientation.z = myQuat.z;
    msg.info.origin.orientation.w = myQuat.w;
    msg.data.resize(50*30);

    HeightMap Oded = height_map->deriveMiniMap(position.x, position.y, myRot);
    //Oded.calculateTypes();
    vector<double>& heights = Oded.getHeights();
    vector<int>& types = Oded.getTypes();
    vector<int>& features = Oded.getFeatures();
    for(int j = 0; j < 50; j++) //height
        for(int i = 0; i < 30; i++) //width
        {
            msg.data[j*30+i].height = heights[j*30+i];
            msg.data[j*30+i].type = types[j*30+i];
            msg.data[j*30+i].feature = features[j*30+i];
        }
    component->publishMiniMap(msg);

}


void Mapper::setVisualize(unsigned char flags)
{
    lock.lock();
    Mapper::visualize = flags;
    lock.unlock();
}


const int MAPPING_FREQUENCY = 15; 

void Mapper::StereoThread()
{
    while(1)
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000/MAPPING_FREQUENCY));
        Mat _stereo;
        lock.lock();
        if(!camR || !camL) //all data arrived at least once
        {
            lock.unlock();
            continue;
        }

        _stereo = handleStereo(camLImg, camRImg);
        lock.unlock();
        disparity.lock();
        stereo = _stereo;
        disparity.unlock();
    }
} 
