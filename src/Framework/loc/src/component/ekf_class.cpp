#include "ekf_class.h"
#include "helpermath.h"
#include "boost/assign.hpp"
#include "gps_calculator.h"
#include "GPS2file.h"
#include <math.h>
#include <tf/tf.h>

const char frame_name[10] = "ODOM";

ekf::ekf() : _Egps(20),_Eimu(20)
{
	//ros::param::set("/LOC/Ready",0); 
	_ready = 0;
	_gps_height = true;
	_while_standing = false;
	std::cout << "setting properties" << std::endl;
	__init__props(0);
	this->first_GPS_flag = 1;
	this->GPS_flag = 1;
	this->IMU_flag = 1;
	this->estimatedPose.pose.pose.position.x = 0;
	this->estimatedPose.pose.pose.position.y = 0;
	this->estimatedPose.pose.pose.position.z = 1.65;

	this->estimatedPose.pose.pose.orientation.x = 0;
	this->estimatedPose.pose.pose.orientation.y = 0;
	this->estimatedPose.pose.pose.orientation.z = 0;
	this->estimatedPose.pose.pose.orientation.w = 1;

	this->velocity.twist.linear.x = 0;
	this->velocity.twist.linear.y = 0;
	this->velocity.twist.linear.z = 0;

	this->gas_pedal_state = 0;
	this->last_pose = this->estimatedPose;
    this->calibrate(20);
}

ekf::~ekf()
{
	std::cout << "EKF says: bye bye!!" << std::endl;
	ros::param::set("/LOC/Ready",0);
}

void ekf::calibrate(int numOfMeasurements)
{
    ROS_INFO("LOC: Performing calibration with %d measurements.\n Try not to move vehicle!", numOfMeasurements);
    _Egps = Egps(numOfMeasurements);
}

void ekf::setGPSMeasurement(sensor_msgs::NavSatFix measurement)
{
	if (_init_latitude != -1 && _init_longitude != -1 && first_GPS_flag)
	{
		std::cout << "Using Initial coordinates as specified in the userHeader.h file." << std::endl;
		_Egps.on = false;
        _Egps.set = false;
        this->initialGPS.latitude = _init_latitude;
        this->initialGPS.longitude = _init_longitude;
        first_GPS_flag = 0;
	}
	if (first_GPS_flag)
	{
	    //z.at<double>(2,0) = measurement.altitude;
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
		printf("lat: %.8f\n",initialGPS.latitude);
		printf("long: %.8f\n",initialGPS.longitude);
        this->xk.at<double>(0,0) = 0;
        this->xk.at<double>(1,0) = 0;
	}
	this->GPSmeasurement = measurement;

	//transform GPS measurement to x-y measurement
	double d = calcDistance(GPSmeasurement,initialGPS);
    double theta = -calcBearing(initialGPS,GPSmeasurement);
    double x = d * cos(theta);
    double y = d * sin(theta);
    z.at<double>(0,0) = x;
    if (_dyn.right_hand_axes)
        z.at<double>(1,0) = -y;
    else
        z.at<double>(1,0) = y;
    if (_gps_height) z.at<double>(2,0) = measurement.altitude;
    else z.at<double>(2,0) = 0;//measurement.altitude;//z.at<double>(2,0);
	_received_gps = true;
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
    tf::Quaternion qut(measurement.orientation.x, measurement.orientation.y, measurement.orientation.z, measurement.orientation.w);

    tf::Matrix3x3 m(qut);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    qut.setRPY(roll, pitch, yaw);
    //qut.setRPY(-roll, pitch, -yaw); yaw *= -1;

    z.at<double>(5,0) = (measurement.linear_acceleration.x - Eacc) * cos(yaw);
    z.at<double>(6,0) = (measurement.linear_acceleration.x - Eacc) * sin(yaw);
    z.at<double>(7,0) = qut.x();
    z.at<double>(8,0) = qut.y();
    z.at<double>(9,0) = qut.z();
    z.at<double>(10,0) = qut.w();
	z.at<double>(11,0) = measurement.angular_velocity.x;
	z.at<double>(12,0) = measurement.angular_velocity.y;
	z.at<double>(13,0) = measurement.angular_velocity.z;

}
void ekf::setGPSSpeedMeasurement(sensor_msgs::NavSatFix _speed)
{
//    tf::Quaternion qut (xk.at<double>(7,0), xk.at<double>(8,0), xk.at<double>(9,0), xk.at<double>(10,0));
//    tf::Matrix3x3 m(qut);
//    double roll, pitch, yaw;
//    m.getRPY(roll, pitch, yaw);
//    if (isnan(yaw) || isinf(yaw))
//        yaw = 0;

//    z.at<double>(3,0) = _speed.speed * cos(yaw);// * cos(xk.at<double>(9,0));
//    z.at<double>(4,0) = _speed.speed * sin(yaw);// * sin(xk.at<double>(9,0));
    z.at<double>(3,0) = _speed.latitude;
    z.at<double>(4,0) = -_speed.longitude;
}

void ekf::estimator()
{
	if(_Eimu.set || _Egps.set)
	    return;
	
	if(_ready == 11)
	    ros::param::set("/LOC/Ready",1); 
	_ready++;
	/*
	 * Kalman filter
	 * -------------
     */

    Quaternion qut2(this->IMUmeasurement.orientation);
    Rotation rot2 = GetRotation(qut2);
    double x = z.at<double>(0,0);
    double y = z.at<double>(1,0);
    double x2 = xk.at<double>(0,0);
    double y2 = xk.at<double>(1,0);
    double yaw = atan2(y - y2, x - x2);
//    FILE *f; f = fopen("test_gps_imu.txt", "a");
//    fprintf(f,"%f,%f,%f\n", x2, y2, rot2.yaw);
//    fclose(f);
//    std::cout << z.at<double>(0) << ", " << z.at<double>(1) << ", " << rot2.yaw * 180 / 3.14159 <<", " << yaw * 180 / 3.14159 << std::endl;
	ros::Time time = ros::Time::now();
	setdt(time.toSec());
	time_propagation();
	if(!_received_gps) repairMeasurement();
	measurement_update();
	_received_gps = false;
	
	this->velocity.twist.linear.x = xk.at<double>(3,0);
	this->velocity.twist.linear.y = xk.at<double>(4,0);
	this->velocity.twist.angular.x = xk.at<double>(11,0);
	this->velocity.twist.angular.y = xk.at<double>(12,0);
	this->velocity.twist.angular.z = xk.at<double>(13,0);
	this->velocity.header.stamp = time;
    this->velocity.header.frame_id = frame_name;
	this->estimatedPose.pose.pose.position.x = xk.at<double>(0,0);
	this->estimatedPose.pose.pose.position.y = xk.at<double>(1,0);
	this->estimatedPose.pose.pose.position.z = xk.at<double>(2,0);
	// Rotation rot2(xk.at<double>(7,0), xk.at<double>(8,0), xk.at<double>(9,0));
	// Quaternion quat2 = GetFromRPY(rot2);
	this->estimatedPose.pose.pose.orientation.x = xk.at<double>(7,0);
	this->estimatedPose.pose.pose.orientation.y = xk.at<double>(8,0);
	this->estimatedPose.pose.pose.orientation.z = xk.at<double>(9,0);
	this->estimatedPose.pose.pose.orientation.w = xk.at<double>(10,0);
	this->estimatedPose.header.stamp = time;
    this->estimatedPose.header.frame_id = frame_name;
	this->estimatedPose.pose.covariance = boost::assign::list_of (P.at<double>(0,0)) (P.at<double>(1,0)) (P.at<double>(2,0)) (P.at<double>(5,0)) (P.at<double>(6,0)) (P.at<double>(7,0))
								    (P.at<double>(0,1)) (P.at<double>(1,1)) (P.at<double>(2,1)) (P.at<double>(5,1)) (P.at<double>(6,1)) (P.at<double>(7,1))
								    (P.at<double>(0,2)) (P.at<double>(1,2)) (P.at<double>(2,2)) (P.at<double>(5,2)) (P.at<double>(6,2)) (P.at<double>(7,2))
								    (P.at<double>(0,5)) (P.at<double>(1,5)) (P.at<double>(2,5)) (P.at<double>(5,5)) (P.at<double>(6,5)) (P.at<double>(7,5))
								    (P.at<double>(0,6)) (P.at<double>(1,6)) (P.at<double>(2,6)) (P.at<double>(5,6)) (P.at<double>(6,6)) (P.at<double>(7,6))
								    (P.at<double>(0,7)) (P.at<double>(1,7)) (P.at<double>(2,7)) (P.at<double>(5,7)) (P.at<double>(6,7)) (P.at<double>(7,7));
}

void ekf::broadcastTF()
{
    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(this->estimatedPose.pose.pose.position.x,
                                    this->estimatedPose.pose.pose.position.y,
                                    this->estimatedPose.pose.pose.position.z));
    transform.setRotation(tf::Quaternion(this->estimatedPose.pose.pose.orientation.x,
                                         this->estimatedPose.pose.pose.orientation.y,
                                         this->estimatedPose.pose.pose.orientation.z,
                                         this->estimatedPose.pose.pose.orientation.w));

    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                   "WORLD", frame_name));

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
    int mod;
    ros::param::param("/LOC/Model",mod,0);
    if (mod)
        xk1 = F*xk + B*u;
    else
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
    this->initialGPS = initGPS;
	gps2file(this->initialGPS.altitude,this->initialGPS.latitude,this->initialGPS.longitude);
}



void ekf::setSteeringInput(double msg){
  this->u.at<double>(0,1) = msg * 12 / 0.6 / 0.95;
}

void ekf::setThrottleInput(double msg){
  this->u.at<double>(0,0) = msg * 12;
}



void ekf::positionUpdate(geometry_msgs::PoseStamped msg)
{
  xk.at<double>(0,0) = msg.pose.position.x;
  xk.at<double>(1,0) = msg.pose.position.y;
  xk.at<double>(2,0) = msg.pose.position.z;  
  Quaternion qut2(msg.pose.orientation);
  Rotation rot2 = GetRotation(qut2);
  xk.at<double>(7,0) = rot2.roll;
  xk.at<double>(8,0) = rot2.pitch;
  xk.at<double>(9,0) = rot2.yaw;
}


void ekf::repairMeasurement()
{
    z.at<double>(0,0) = xk1.at<double>(0,0);
    z.at<double>(1,0) = xk1.at<double>(1,0);
    z.at<double>(2,0) = xk1.at<double>(2,0);
    //z.at<double>(3,0) = xk1.at<double>(3,0);
    //z.at<double>(4,0) = xk1.at<double>(4,0);
}

