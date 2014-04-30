#include "ekf_class.h"
#include "helpermath.h"

ekf::ekf()
{
  __init__props();
  this->first_GPS_flag = 1;
  this->GPS_flag = 1;
  this->IMU_flag = 1;
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
  if(first_GPS_flag)
  {
    std::cout << "setting first GPS location as (0,0)" << std::endl;
    setInitGPS(measurement); 
    first_GPS_flag = 0;
  }
  this->GPSmeasurement = measurement;
  //transform GPS measurement to x-y measurement
  double d = calcDistance(GPSmeasurement,initialGPS);
  double theta = calcBearing(initialGPS,GPSmeasurement);
  double x = d * cos(theta);
  double y = d * sin(theta);
  z.at<double>(0,0) = x;
  z.at<double>(1,0) = y;
}
void ekf::setIMUMeasurement(sensor_msgs::Imu measurement)
{
  this->IMUmeasurement = measurement;
  Quaternion qut2(measurement.orientation);
  Rotation rot2 = GetRotation(qut2);
  z.at<double>(2,0) = rot2.yaw;
  z.at<double>(3,0) = sqrt(measurement.angular_velocity.x*measurement.angular_velocity.x+
		  	  	  	  measurement.angular_velocity.y*measurement.angular_velocity.y+
		  	  	  	  measurement.angular_velocity.z*measurement.angular_velocity.z);
  z.at<double>(4,0) = sqrt(measurement.linear_acceleration.x*measurement.linear_acceleration.x+
		  	  	  	  measurement.linear_acceleration.y*measurement.linear_acceleration.y);//- 60*9.81*pow10(-6);
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
	/*
	 Kalman filter

	ros::Time time;
	time = ros::Time::now();
	measurement_update();
	time_propagation();
	this->velocity.twist.linear.x = xk.at<double>(3,0) * cos(xk.at<double>(2,0));
	this->velocity.twist.linear.y = xk.at<double>(3,0) * sin(xk.at<double>(2,0));
	this->velocity.header.stamp = time;

	this->estimatedPose.pose.pose.position.x = xk.at<double>(0,0);
	this->estimatedPose.pose.pose.position.y = xk.at<double>(1,0);
	this->estimatedPose.pose.pose.orientation = IMUmeasurement.orientation;
	this->estimatedPose.header.stamp = time;*/
	/* non KF estimation*/
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
	this->estimatedPose.pose.pose.position.z = GPSmeasurement.altitude;
	this->estimatedPose.pose.pose.orientation = IMUmeasurement.orientation;
	this->estimatedPose.header.stamp = time;


}

void ekf::measurement_update()
{
	Mat A = H*P1*H.t()+R;
	K = P1*H.t()*A.inv();
	xk = xk1 + K*(z-H*xk1);
	Mat L = Mat::eye(s,s,CV_64F) - K*H;
	P = L*P1*L.t() + K*R*K.t();
	modify_F(0.1,xk.at<double>(2,0));
}

void ekf::time_propagation()
{
	xk1 = F*xk;
	P1 = F*P*F.t() + Q;
}


