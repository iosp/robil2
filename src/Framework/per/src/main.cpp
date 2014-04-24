#include <ros/ros.h>
#include "component/ComponentMain.h"
#include "component/ComponentStates.h"
#include <ros/spinner.h>
#include <boost/thread/thread.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>


/*
#define DEGREE_TO_M		111000

using namespace std; 
typedef string String;
typedef bool boolean;
using namespace cv;

HeightMap* height_map;

bool ref_pos_set = false;
bool imu_set = false;
Vec3D position, ref_position;
Quaternion myQuat;

Rotation myRot, ibeoRot, leftSickRot, rightSickRot;
//Vec3D ibeoPos, leftSickPos, rightSickPos;


Mat getDisparity(Mat left_image, Mat right_image)
{
    cvtColor( left_image,left_image, CV_BGR2GRAY);
    cvtColor( right_image,right_image,CV_BGR2GRAY);
    IplImage temp=left_image;
    IplImage temp2=right_image;
    CvMat *matf= cvCreateMat ( temp.height, temp.width, CV_16S);
    CvStereoBMState * state=cvCreateStereoBMState(CV_STEREO_BM_NARROW,16*10);
    cvFindStereoCorrespondenceBM(&temp,&temp2,matf,state);
    CvMat * disp_left_visual= cvCreateMat(temp.height, temp.width, CV_8U);
    cvConvertScale( matf, disp_left_visual, -16 );
    cvNormalize( matf, matf, 0, 256, CV_MINMAX, NULL );
    int i, j;
    uchar *ptr_dst;
    IplImage *cv_image_depth_aux = cvCreateImage (cvGetSize(&temp),IPL_DEPTH_8U, 3);
    for ( i = 0; i < matf->rows; i++)
    {
    	ptr_dst = (uchar*)(cv_image_depth_aux->imageData + i*cv_image_depth_aux->widthStep);
    	for ( j = 0; j < matf->cols; j++ )
    	{
    		ptr_dst[3*j] = (uchar)((short int*)(matf->data.ptr + matf->step*i))[j];
    		ptr_dst[3*j+1] = (uchar)((short int*)(matf->data.ptr + matf->step*i))[j];
    		ptr_dst[3*j+2] = (uchar)((short int*)(matf->data.ptr + matf->step*i))[j];
    	}
    }
    //system("pause");
    //cvSaveImage("disp.ppm", &cv_image_depth_aux);
    //system("pause");
    Mat img(cv_image_depth_aux, true);
	//system("pause");
    cvReleaseImage(&cv_image_depth_aux);
	cvtColor(img,img, CV_BGR2GRAY);
    for(int i = 0; i < img.rows; i++)
	   for(int j = 0; j < img.cols; j++)
	       if(img.at<uchar>(i, j) > 150) img.at<uchar>(i, j) = 0;
	       else img.at<uchar>(i, j) = (int)(img.at<uchar>(i, j)*3) ;
    return img;
}

void handleLeftSICK(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if(!imu_set || !ref_pos_set) return;
  Quaternion q = GetFromRPY(leftSickRot);
  Vec3D front = GetFrontVector(q.x,q.y,q.z,q.w);
  Vec3D right = GetRightVector(q.x,q.y,q.z,q.w);
  Vec3D up = GetUpVector(q.x,q.y,q.z,q.w);
  
  Vec3D pos = position.add(front.multiply(-0.2187)).add(right.multiply(0.85)).add(up.multiply(0.631));
  
  for(int i = 0; i < msg->ranges.size(); i++) 
    if(msg->ranges[i] < 0.5*msg->range_max && msg->ranges[i] > 5)
      ProjectLaserRange(
		      height_map, 
		      right, 
		      front,
		      pos, 
		      msg->ranges[i],
		      msg->angle_min + i*msg->angle_increment);
    
  
  //height_map->displayGUI();
}

void handleRightSICK(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if(!imu_set || !ref_pos_set) return;
  Quaternion q = GetFromRPY(rightSickRot);
  Vec3D front = GetFrontVector(q.x,q.y,q.z,q.w);
  Vec3D right = GetRightVector(q.x,q.y,q.z,q.w);
  Vec3D up = GetUpVector(q.x,q.y,q.z,q.w);
  
  Vec3D pos = position.add(front.multiply(-0.2187)).add(right.multiply(-0.85)).add(up.multiply(0.631));
  
  for(int i = 0; i < msg->ranges.size(); i++) 
    if(msg->ranges[i] < 0.5*msg->range_max && msg->ranges[i]>5)
      ProjectLaserRange(
		      height_map, 
		      right, 
		      front,
		      pos, 
		      msg->ranges[i],
		      msg->angle_min + i*msg->angle_increment);
    
  
  //height_map->displayGUI();
}

void handleCameraL(const  sensor_msgs::Image::ConstPtr& msg)
{
  
}

void handleCameraR(const  sensor_msgs::Image::ConstPtr& msg)
{
  
}


void handleIBEO(const robil_msgs::MultiLaserScan::ConstPtr& msg)
{
  //delete height_map;
  //height_map = new HeightMap(500,500);
  if(!imu_set || !ref_pos_set) return;
  Rotation t2 = ibeoRot.add(Rotation(0, -msg->angle_t2, 0));
  Rotation t1 = ibeoRot.add(Rotation(0, -msg->angle_t1, 0));
  Rotation b1 = ibeoRot.add(Rotation(0, -msg->angle_b1, 0));
  Rotation b2 = ibeoRot.add(Rotation(0, -msg->angle_b2, 0));
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
  
  
  double incrtop = msg->angle_increment;
  double incrbottom = msg->angle_increment;
  
    for(int i = 0; i < msg->ranges_t2.size(); i++) 
    {
      if(msg->ranges_t2[i] < 0.5*msg->range_max)
      ProjectLaserRange(
			height_map, 
			t2right, 
			t2front,
			pos, 
			msg->ranges_t2[i],
			msg->angle_min_t + i*incrtop);
    }  
    
    for(int i = 0; i < msg->ranges_t1.size(); i++) 
    {
      if(msg->ranges_t1[i] < 0.5*msg->range_max)
      ProjectLaserRange(
			height_map, 
			t1right, 
			t1front,
			pos, 
			msg->ranges_t1[i],
			msg->angle_min_t + i*incrtop); 
    }
    for(int i = 0; i < msg->ranges_b1.size(); i++) 
    {
      if(msg->ranges_b1[i] < 0.5*msg->range_max)
      ProjectLaserRange(
		      height_map, 
		      b1right, 
		      b1front,
		      pos, 
		      msg->ranges_b1[i],
		      msg->angle_min_b + i*incrbottom);
    }
    for(int i = 0; i < msg->ranges_b2.size(); i++) 
    {
      if(msg->ranges_b2[i] < 0.5*msg->range_max)
      ProjectLaserRange(
		      height_map, 
		      b2right, 
		      b2front,
		      pos, 
		      msg->ranges_b2[i],
		      msg->angle_min_b + i*incrbottom);
    }
    
    
}

void handleGPS(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  double longitude = msg->longitude;
  double latitude = msg->latitude;
  double altitude = msg->altitude;
  if(!ref_pos_set) 
  {
    ref_position = Vec3D(latitude, longitude, altitude);
    ref_pos_set = true;
  }
  position = Vec3D((latitude-ref_position.x)*DEGREE_TO_M, (longitude-ref_position.y)*DEGREE_TO_M, altitude);
  if(!imu_set) return;
  
  //Vec3D up = GetUpVector(myQuat.x,myQuat.y,myQuat.z,myQuat.w);
  //position = position.add(up.multiply(-0.1)); //gps -> bobcat
  
}

void handleIMU(const sensor_msgs::Imu::ConstPtr& msg)
{
  double x = msg->orientation.x;
  double y = msg->orientation.y;
  double z = msg->orientation.z;
  double w = msg->orientation.w;
  
  myQuat = Quaternion(x,y,z,w);
  myRot = GetRotation(myQuat);
  ibeoRot = myRot.add(Rotation(0, 0.284, -0));
  leftSickRot = Rotation(myRot.pitch, myRot.roll, myRot.yaw+1.57);
  rightSickRot = Rotation(-myRot.pitch, -myRot.roll, myRot.yaw-1.57);
  
  //Rotation rot = GetRotation(myQuat);
  //myQuat = GetFromRPY(rot); 
  //Vec3D up = GetUpVector(myQuat.x,myQuat.y,myQuat.z,myQuat.w);
  //Vec3D front = GetFrontVector(myQuat.x,myQuat.y,myQuat.z,myQuat.w);
  //Vec3D right = GetRightVector(myQuat.x,myQuat.y,myQuat.z,myQuat.w);
  imu_set = true;
  //printf("[%s]\n[%s]\n[%s]\n\n", right.toString().c_str(),front.toString().c_str(), up.toString().c_str());
  //printf("%s\n", rot.toString().c_str()); 
  if(ref_pos_set) height_map->displayGUI(myRot.yaw*180/3.14159, 10*position.x, 10*position.y);
}


int main(int argc, char** argv) 
{
    //system("del heightmap.o");
    
    //height_map->displayGUI();
    
    ros::init(argc, argv, "per_node");
    ros::NodeHandle n;
    ros::Subscriber subsick = n.subscribe("/right_sick/scan", 5, handleRightSICK);
    ros::Subscriber subsick2 = n.subscribe("/left_sick/scan", 5, handleLeftSICK);
    ros::Subscriber subibeo = n.subscribe("/LD_MRS", 5, handleIBEO);
    ros::Subscriber subGPS = n.subscribe("/SENSORS/GPS", 5, handleGPS);
    ros::Subscriber subINS = n.subscribe("/SENSORS/IMU", 5, handleIMU);
    ros::Subscriber subCamL = n.subscribe("/SENSORS/CAM/L", 5, handleCameraL);
    ros::Subscriber subCamR = n.subscribe("/SENSORS/CAM/R", 5, handleCameraR);
    ros::spin();
   
    
    
    return 0;
}

*/

int main(int argc,char** argv)
{
  ComponentMain comp(argc,argv);
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  runComponent(argc,argv, comp);
  return 0;
}
