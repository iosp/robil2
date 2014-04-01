#include "ekf_class.h"
#include "helpermath.h"

ekf::ekf()
{
  this->GPS_flag = 1;
  this->estimatedPose.pose.pose.position.x = 0;
  this->estimatedPose.pose.pose.position.y = 0;
  this->estimatedPose.pose.pose.position.z = 0;
  
  this->estimatedPose.pose.pose.orientation.x = 0;
  this->estimatedPose.pose.pose.orientation.y = 0;
  this->estimatedPose.pose.pose.orientation.z = 0;
  this->estimatedPose.pose.pose.orientation.w = 0;
  
  this->velocity.twist.linear.x = 0;
  this->velocity.twist.linear.y = 0;
  this->velocity.twist.linear.z = 0;
 
}

ekf::~ekf()
{
 std::cout << "EKF says: bye bye!!" << std::endl;
}
void ekf::setInitGPS(sensor_msgs::NavSatFix initGPS)
{
  this->initialGPS = initGPS;      
}

void ekf::setGPSMeasurement(sensor_msgs::NavSatFix measurement)
{
  if(GPS_flag)
  {
    std::cout << "setting first GPS location as (0,0)" << std::endl;
    setInitGPS(measurement); 
    GPS_flag = 0;
  }
  this->GPSmeasurement = measurement;
}
void ekf::setIMUMeasurement(sensor_msgs::Imu measurement)
{
  this->IMUmeasurement = measurement;
}
double ekf::calcDistance(sensor_msgs::NavSatFix p1,sensor_msgs::NavSatFix p2)
{
  double R = 6371; //[km]
  double lat1 = p1.latitude*PI/180;
  double lat2 = p2.latitude*PI/180;
  double dLat = lat2 - lat1;
  double dLon = (p2.longitude - p1.longitude)*PI/180;
  
  double a = sin(dLat/2) * sin(dLat/2) + 
	     sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  double d = R*c;
  
  return d*1000; //return distance in [m]
}
double ekf::calcBearing(sensor_msgs::NavSatFix p1,sensor_msgs::NavSatFix p2)
{
  double lat1 = p1.latitude*PI/180;
  double lat2 = p2.latitude*PI/180;
  double dLat = lat2 - lat1;
  double dLon = (p2.longitude - p1.longitude)*PI/180;
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) -
        sin(lat1) * cos(lat2) * cos(dLon);
  double brng = atan2(y, x);
  
  return brng;
}
geometry_msgs::PoseWithCovarianceStamped ekf::getEstimatedPose()
{
  return this->estimatedPose; 
}
geometry_msgs::TwistStamped ekf::getEstimatedSpeed()
{
  return this->velocity;  
}
void ekf::estimator()
{
  double d = calcDistance(GPSmeasurement,initialGPS);
  double theta = calcBearing(initialGPS,GPSmeasurement);
  double x = d * cos(theta);
  double y = d * sin(theta);
  ros::Time time,dt;
  time = ros::Time::now();
  dt = time;
  dt.sec -= this->estimatedPose.header.stamp.sec;
  dt.nsec -= this->estimatedPose.header.stamp.nsec;
  this->velocity.twist.linear.x = (x-this->estimatedPose.pose.pose.position.x)/(dt.toSec());
  this->velocity.twist.linear.y = (y-this->estimatedPose.pose.pose.position.y)/(dt.toSec());
  this->velocity.header.stamp = time;

  Quaternion qut(this->estimatedPose.pose.pose.orientation);
  Rotation rot1 = GetRotation(qut);
  Quaternion qut2(IMUmeasurement.orientation);
  Rotation rot2 = GetRotation(qut2);
  rot2 = rot2.add(Rotation(-rot1.roll, -rot1.pitch, -rot1.yaw));
  rot2.pitch /= dt.toSec();
  rot2.roll /= dt.toSec();
  rot2.yaw /= dt.toSec();
  
  
  this->velocity.twist.angular.x = rot2.roll;
  this->velocity.twist.angular.y = rot2.pitch;
  this->velocity.twist.angular.z = rot2.yaw;

    
  this->estimatedPose.pose.pose.position.x = x;
  this->estimatedPose.pose.pose.position.y = y;
  this->estimatedPose.pose.pose.orientation = IMUmeasurement.orientation;
  this->estimatedPose.header.stamp = time;
}
