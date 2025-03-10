#include "noiseless_estimator.h"
#include "helpermath.h"
#include "gps_calculator.h"
#include "GPS2file.h"
Observer::Observer()
{
	_ready = 0;
	dx = 0; dy = 0;
	this->first_GPS_flag = 1;
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

	this->velocity.twist.angular.x = 0;
	this->velocity.twist.angular.y = 0;
	this->velocity.twist.angular.z = 0;
}

Observer::~Observer()
{
	std::cout << "Localization says: bye bye!!" << std::endl;
	ros::param::set("/LOC/Ready",0);
}
void Observer::setInitGPS(sensor_msgs::NavSatFix initGPS)
{
	this->initialGPS = initGPS;
	gps2file(this->initialGPS.altitude,this->initialGPS.latitude,this->initialGPS.longitude);
}

void Observer::setGPSMeasurement(sensor_msgs::NavSatFix measurement)
{
	if(first_GPS_flag)
	{
		std::cout << "setting first GPS location as (0,0)" << std::endl;
		setInitGPS(measurement);
		first_GPS_flag = 0;
	}
	this->GPSmeasurement = measurement;
}
void Observer::setIMUMeasurement(sensor_msgs::Imu measurement)
{
	this->IMUmeasurement = measurement;
}
void Observer::setGPSSpeedMeasurement(robil_msgs::GpsSpeed measurement)
{
	this->speedMeasurement = measurement;
}
geometry_msgs::PoseWithCovarianceStamped Observer::getEstimatedPose()
{
	return this->estimatedPose;
}
geometry_msgs::TwistStamped Observer::getEstimatedSpeed()
{
	return this->velocity;
}

geometry_msgs::TwistStamped filter(geometry_msgs::TwistStamped prev,geometry_msgs::TwistStamped current)
{
	geometry_msgs::TwistStamped avg;
	double alpha = 0.1;
	avg.twist.linear.x = prev.twist.linear.x * (1 - alpha) + current.twist.linear.x * alpha;
	avg.twist.linear.y = prev.twist.linear.y * (1 - alpha) + current.twist.linear.y * alpha;
	avg.twist.linear.z = prev.twist.linear.z * (1 - alpha) + current.twist.linear.z * alpha;
	
	avg.twist.angular.x = prev.twist.angular.x * (1 - alpha) + current.twist.angular.x * alpha;
	avg.twist.angular.y = prev.twist.angular.y * (1 - alpha) + current.twist.angular.y * alpha;
	avg.twist.angular.z = prev.twist.angular.z * (1 - alpha) + current.twist.angular.z * alpha;
	
	if (avg.twist.angular.z > PI)
	  avg.twist.angular.z -= 2*PI;
	if (avg.twist.angular.z < -PI)
	  avg.twist.angular.z += 2*PI;
	return avg;
}

bool isNan(geometry_msgs::TwistStamped avg)
{
  if (avg.twist.linear.x != avg.twist.linear.x) return true;
  if (avg.twist.linear.y != avg.twist.linear.y) return true;
  if (avg.twist.linear.z != avg.twist.linear.z) return true;
  
  if (avg.twist.angular.x != avg.twist.angular.x) return true;
  if (avg.twist.angular.y != avg.twist.angular.y) return true;
  if (avg.twist.angular.z != avg.twist.angular.z) return true;
  
  return false;
  
  
}
void Observer::estimator()
{	
	if(_ready == 11)
	  ros::param::set("/LOC/Ready",1);
	_ready++;
	/* non KF estimation*/
	this->lastPose = this->estimatedPose;
	double d = calcDistance(GPSmeasurement,initialGPS);
	double theta = calcBearing(initialGPS,GPSmeasurement);
	double x = d * cos(theta);
	double y = d * sin(theta);
	geometry_msgs::TwistStamped prev_vel = this->velocity;
	ros::Time time,dt;
	time = ros::Time::now();
	dt = time;
	dt.sec -= this->estimatedPose.header.stamp.sec;
	dt.nsec -= this->estimatedPose.header.stamp.nsec;

	Quaternion qut(this->estimatedPose.pose.pose.orientation);
	Rotation rot1 = GetRotation(qut);
	Quaternion qut2(IMUmeasurement.orientation);
	Rotation rot2 = GetRotation(qut2);
	
	this->estimatedPose.pose.pose.position.x = x;
	this->estimatedPose.pose.pose.position.y = y;
	this->estimatedPose.pose.pose.position.z = GPSmeasurement.altitude;
	this->estimatedPose.pose.pose.orientation = IMUmeasurement.orientation;
	this->estimatedPose.header.stamp = time;
	
	double yaw = rot2.yaw;
	while(yaw < 0)
	  yaw += 2*PI;
	while (yaw > 2* PI)
	  yaw -= 2*PI;
	double mulx = 1, muly = 1;
	dx = (int) ((this->estimatedPose.pose.pose.position.x - this->lastPose.pose.pose.position.x)*100) * 0.4 + 0.6 * dx;
	dy = (int) ((this->estimatedPose.pose.pose.position.y - this->lastPose.pose.pose.position.y)*100) * 0.4 + 0.6 * dy;
	if ( dx < 0)
	  mulx = -1;
	if (!(yaw < PI/2 || yaw > 3*PI/2)) mulx *= -1;
	if (dy < 0)
	  muly = -1;
	if (yaw < 3*PI/2 && yaw > PI/2) muly *= -1;
	
// 	printf("dx: %.2f | dy: %.2f | yaw: %.3f\n",dx,dy,yaw * 180 / PI);
	
	this->velocity.twist.linear.x = (this->speedMeasurement.speed * cos(yaw) * cos(rot2.pitch)) * mulx;
	this->velocity.twist.linear.y = (this->speedMeasurement.speed * sin(yaw) * cos(rot2.pitch)) * mulx;
	this->velocity.twist.linear.z = this->speedMeasurement.speed * sin(rot2.pitch);
	this->velocity.header.stamp = time;
		

	rot2 = rot2.add(Rotation(-rot1.roll, -rot1.pitch, -rot1.yaw));
	
	rot2.pitch /= dt.toSec();
	rot2.roll /= dt.toSec();
	rot2.yaw /= dt.toSec();
	
	this->velocity.twist.angular.x = rot2.roll;
	this->velocity.twist.angular.y = rot2.pitch;
	this->velocity.twist.angular.z = rot2.yaw;
	if(isNan(this->velocity))
	  this->velocity = prev_vel;
	else
	  this->velocity = filter(prev_vel,this->velocity);

}


