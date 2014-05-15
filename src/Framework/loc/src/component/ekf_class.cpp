#include "ekf_class.h"
#include "helpermath.h"
#include "std_msgs/Float64.h"
#include "boost/assign.hpp"
#include "gps_calculator.h"
ekf::ekf() : _Egps(100),_Eimu(100)
{
	_while_standing = false;
	std::cout << "setting properties" << std::endl;
	__init__props(0);
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

	this->gas_pedal_state = 0;
	this->last_pose = this->estimatedPose;
}

ekf::~ekf()
{
	std::cout << "EKF says: bye bye!!" << std::endl;
}

void ekf::setGPSMeasurement(sensor_msgs::NavSatFix measurement)
{
	if (_init_latitude != -1 && _init_longitude != -1)
	{
		std::cout << "Using Initial coordinates as specified in the userHeader.h file." << std::endl;
		_Egps.on = false;
	}
	if (first_GPS_flag)
	{
		std::cout << "setting first GPS location as (0,0)" << std::endl;
		setInitGPS(measurement);
		first_GPS_flag = 0;
	}
	if (_Egps.on)
		_Egps.updateEgps(measurement);
	else if (_Egps.set)
	{
		this->changeInitCoor(_Egps.getGPScoor());
		_Egps.set = false;
		std::cout << "changing the initial GPS coordinates" << std::endl;
		std::cout << "------------------------------------" << std::endl;
		printf("lat: %.8f\n",_Egps.getGPScoor().latitude);
		printf("long: %.8f\n",_Egps.getGPScoor().longitude);
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
	if (_Eimu.on)
		_Eimu.updateEimu(measurement);
	else if (_Eimu.set)
	{
		Eacc = _Eimu.getEIMU().linear_acceleration.x;
		_Eimu.set = false;
		std::cout << "changing the mean of Acc coordinates" << std::endl;
		std::cout << "------------------------------------" << std::endl;
		std::cout << "Eacc: " << _Eimu.getEIMU().linear_acceleration.x << std::endl;
	}
	this->IMUmeasurement = measurement;
	Quaternion qut2(measurement.orientation);
	Rotation rot2 = GetRotation(qut2);
	z.at<double>(3,0) = rot2.yaw;
	z.at<double>(4,0) = measurement.angular_velocity.z;
	z.at<double>(2,0) = (measurement.linear_acceleration.x - Eacc);
}

void ekf::estimator()
{
	if(_Eimu.set || _Egps.set)
		return;
	/*
	 * Kalman filter
	 * -------------
	 */
	ros::Time time = ros::Time::now();
	setdt(time.toSec());
	if (_while_standing)
		modify_Q(0);
	else
		modify_Q(0.02);
	measurement_update();
	if (_while_standing)
	{
		xk.at<double>(2,0) = 0.0;
		xk.at<double>(3,0) = 0.0;
	}
	time_propagation();

	this->last_pose = this->estimatedPose;
	this->velocity.twist.linear.x = xk.at<double>(2,0) * cos(xk.at<double>(4,0));
	this->velocity.twist.linear.y = xk.at<double>(2,0) * sin(xk.at<double>(4,0));
	this->velocity.header.stamp = time;

	this->estimatedPose.pose.pose.position.x = xk.at<double>(0,0);
	this->estimatedPose.pose.pose.position.y = xk.at<double>(1,0);
	this->estimatedPose.pose.pose.position.z = GPSmeasurement.altitude;
	this->estimatedPose.pose.pose.orientation = IMUmeasurement.orientation;
	this->estimatedPose.header.stamp = time;

	this->estimatedPose.pose.covariance = boost::assign::list_of (P.at<double>(0,0)) (P.at<double>(1,0)) (0) (P.at<double>(4,0)) (0) (0)
		                                                        		 (P.at<double>(0,1)) (P.at<double>(1,1)) (0) (P.at<double>(4,1)) (0) (0)
		                                                        		 (0) (0) (0) (0) (0) (0)
		                                                        		 (P.at<double>(0,4)) (P.at<double>(1,4)) (0) (P.at<double>(4,4)) (0) (0)
		                                                        		 (0) (0) (0) (0) (0) (0)
		                                                        		 (0) (0) (0) (0) (0) (0);
}

void ekf::measurement_update()
{
	Mat A = H*P1*H.t()+R;
	K = P1*H.t()*A.inv();
	xk = xk1 + K*(z-H*xk1);
	Mat L = Mat::eye(s,s,CV_64F) - K*H;
	P = L*P1*L.t() + K*R*K.t();
}

void ekf::time_propagation()
{
	xk1 = F*xk;
	P1 = F*P*F.t() + Q;
}
void ekf::setGasPedalState(std_msgs::Float64 value)
{
	if (value.data>0.05)
	{
		this->gas_pedal_state = 1;
		_while_standing = false;
	}
	else
	{
		this->gas_pedal_state = 0;
		if (this->xk.at<double>(2,0) < 0.3)
			_while_standing = true;
	}
}
geometry_msgs::PoseWithCovarianceStamped ekf::getEstimatedPose()
{
	return this->estimatedPose;
}
geometry_msgs::TwistStamped ekf::getEstimatedSpeed()
{
	return this->velocity;
}
void ekf::changeEacc(sensor_msgs::Imu _Eacc)
{
	this->Eacc = _Eacc.linear_acceleration.x;
}
void ekf::changeInitCoor(sensor_msgs::NavSatFix coor)
{
	this->setInitGPS(coor);
}
void ekf::setInitGPS(sensor_msgs::NavSatFix initGPS)
{
	if (_init_latitude == -1 || _init_longitude == -1)
		this->initialGPS = initGPS;
	else
	{
		this->initialGPS.latitude = _init_latitude;
		this->initialGPS.longitude = _init_longitude;
	}
}
