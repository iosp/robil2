#include "mapper.h"
#include <stdio.h>
#include <stdlib.h>
#include <boost/thread.hpp>
#include "../stereo.h"

#include <sensor_msgs/image_encodings.h>

bool Mapper::camL;
bool Mapper::camR;
bool Mapper::loc_received;
unsigned char Mapper::visualize;
boost::mutex Mapper::lock; 
Rotation Mapper::myRot, Mapper::ibeoRot, Mapper::leftSickRot, Mapper::rightSickRot;
Vec3D Mapper::position;
Quaternion Mapper::myQuat;
Mat Mapper::camLImg, Mapper::camRImg;
HeightMap* Mapper::height_map;
RosComm* Mapper::roscomm;

void Mapper::MainLoop()
{
  camL = false;
  camR = false;
  loc_received = false;
  visualize = 0x00;
  height_map = new HeightMap(500,500);
  
  while(1)
  {
    //printf("MAPPER\n");
    lock.lock();
    height_map->calculateTypes();
    publishMap();
    publishMiniMap();
    if(camL && camR) 
    {
      handleStereo(camRImg, camRImg);
      //printf("WORKS\n\n\n\n");
    }
    lock.unlock();
    boost::this_thread::sleep(boost::posix_time::milliseconds(100)); //10hz cycle
  }
}

void Mapper::VisualizeLoop()
{
  while(1)
  {
    if(height_map != NULL)// && visualize > 0)
    {
      lock.lock();
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
	m.displayTypesGUI();
      }
      if((visualize & VISUALIZE_FULLMAP) != 0) //global map needed
      {
	height_map->displayGUI(myRot.yaw*180/3.14159, position.x, position.y, 2);
      }
      
      lock.unlock();
      waitKey(100);
    }
  }
}

void Mapper::handleIBEO(const config::PER::sub::SensorIBEO& msg)
{
  if(!loc_received) return;
  lock.lock();
  Rotation t2 = ibeoRot.add(Rotation(0, -msg.angle_t2, 0));
  Rotation t1 = ibeoRot.add(Rotation(0, -msg.angle_t1, 0));
  Rotation b1 = ibeoRot.add(Rotation(0, -msg.angle_b1, 0));
  Rotation b2 = ibeoRot.add(Rotation(0, -msg.angle_b2, 0));
  Quaternion qt2 = GetFromRPY(t2);
  Quaternion qt1 = GetFromRPY(t1);
  Quaternion qb1 = GetFromRPY(b1);
  Quaternion qb2 = GetFromRPY(b2);
  Vec3D t2front = GetFrontVector(qt2.x,qt2.y,qt2.z,qt2.w), t2right = GetRightVector(qt2.x,qt2.y,qt2.z,qt2.w);
  Vec3D t1front = GetFrontVector(qt1.x,qt1.y,qt1.z,qt1.w), t1right = GetRightVector(qt1.x,qt1.y,qt1.z,qt1.w);
  Vec3D b1front = GetFrontVector(qb1.x,qb1.y,qb1.z,qb1.w), b1right = GetRightVector(qb1.x,qb1.y,qb1.z,qb1.w);
  Vec3D b2front = GetFrontVector(qb2.x,qb2.y,qb2.z,qb2.w), b2right = GetRightVector(qb2.x,qb2.y,qb2.z,qb2.w);
  
  Quaternion q = GetFromRPY(ibeoRot);
  Vec3D front = GetFrontVector(q.x,q.y,q.z,q.w);
  Vec3D right = GetRightVector(q.x,q.y,q.z,q.w);
  Vec3D up = GetUpVector(q.x,q.y,q.z,q.w);
  
  Vec3D pos = position.add(front.multiply(-0.375)).add(right.multiply(0.055)).add(up.multiply(1.89));
  
  
  double incrtop = msg.angle_increment;
  double incrbottom = msg.angle_increment;
  
    for(int i = 0; i < msg.ranges_t2.size(); i++) 
    {
      if(msg.ranges_t2[i] < 0.5*msg.range_max)
      ProjectLaserRange(
			height_map, 
			t2right, 
			t2front,
			pos, 
			msg.ranges_t2[i],
			msg.angle_min_t + i*incrtop);
    }  
    
    for(int i = 0; i < msg.ranges_t1.size(); i++) 
    {
      if(msg.ranges_t1[i] < 0.5*msg.range_max)
      ProjectLaserRange(
			height_map, 
			t1right, 
			t1front,
			pos, 
			msg.ranges_t1[i],
			msg.angle_min_t + i*incrtop); 
    }
    for(int i = 0; i < msg.ranges_b1.size(); i++) 
    {
      if(msg.ranges_b1[i] < 0.5*msg.range_max)
      ProjectLaserRange(
		      height_map, 
		      b1right, 
		      b1front,
		      pos, 
		      msg.ranges_b1[i],
		      msg.angle_min_b + i*incrbottom);
    }
    for(int i = 0; i < msg.ranges_b2.size(); i++) 
    {
      if(msg.ranges_b2[i] < 0.5*msg.range_max)
      ProjectLaserRange(
		      height_map, 
		      b2right, 
		      b2front,
		      pos, 
		      msg.ranges_b2[i],
		      msg.angle_min_b + i*incrbottom);
    }
    
    lock.unlock();
}
    
void Mapper::handleSickL(const config::PER::sub::SensorSICK1& msg)
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

void Mapper::handleSickR(const config::PER::sub::SensorSICK2& msg)
{
  if(!loc_received) return;
  lock.lock();
  
  Quaternion q = GetFromRPY(rightSickRot);
  Vec3D front = GetFrontVector(q.x,q.y,q.z,q.w);
  Vec3D right = GetRightVector(q.x,q.y,q.z,q.w);
  Vec3D up = GetUpVector(q.x,q.y,q.z,q.w);
  
  Vec3D pos = position.add(front.multiply(-0.2187)).add(right.multiply(-0.85)).add(up.multiply(0.631));
  
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

void Mapper::handleLocation(const config::PER::sub::Location& msg)
{
  lock.lock();
  geometry_msgs::Pose pose = msg.pose.pose;
  position = Vec3D(pose.position.x, pose.position.y, pose.position.z);
  myQuat = Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  myRot = GetRotation(myQuat);
  ibeoRot = myRot.add(Rotation(0, 0.284, -0));
  leftSickRot = Rotation(myRot.pitch, myRot.roll, myRot.yaw+1.57);
  rightSickRot = Rotation(-myRot.pitch, -myRot.roll, myRot.yaw-1.57);
  loc_received = true;
  
  lock.unlock();

}

void Mapper::handleCamR(const config::PER::sub::SensorCamR& msg)
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
    
void Mapper::handleCamL(const config::PER::sub::SensorCamL& msg)
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
  config::PER::pub::Map msg;
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
  roscomm->publishMap(msg);
  
}

void Mapper::publishMiniMap()
{
  config::PER::pub::Map msg;
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
  roscomm->publishMiniMap(msg);
  
}


void Mapper::setVisualize(unsigned char flags)
{
  lock.lock();
  Mapper::visualize = flags;
  lock.unlock();
}