#include "userHeader.h"
#include <math.h>
#include "EKF_properties.h"
#include "ros/ros.h"
#include <ParameterTypes.h>
#include <opencv2/opencv.hpp>
#include "Egps.h"
#include "Eimu.h"



class ekf:private ekf_props {
	/*
	 * This class performs an Extended Kalman Filter estimation for a noisey environment.
	 * The function receives measurements from the GPS and the IMU and stores the data in Z.
	 *
	 * Then there is a measurement_update stage and a time_propagation stage. This steps are called from within the estimator function.
	 * To retrieve the estimated location and speed, call getEstimatedPose and getEstimatedSpeed functions.
	 */
public:
	/**
	 * Constructor:
	 */
	ekf();
	/**
	 * Destructor:
	 */
	virtual ~ekf();
	/*
	 * Change the init coordinates of the GPS.
	 */
	void changeInitCoor(sensor_msgs::NavSatFix coor);
	/**
	 * set private variable GPSmeasurement
	 */
	void setGPSMeasurement(sensor_msgs::NavSatFix measurement);
	/**
	 * set private variable IMUmeasurement
	 */
	void setIMUMeasurement(sensor_msgs::Imu measurement);

	void estimator();
	/**
	 * return the estimated position
	 */
	geometry_msgs::PoseWithCovarianceStamped getEstimatedPose();
	/**
	 * return the estimated speed
	 */
	geometry_msgs::TwistStamped getEstimatedSpeed();
	/*
	 * Kalman measurement update stage.
	 */
	void measurement_update();
	/*
	 * Kalman time propagation stage.
	 */
	void time_propagation();
	/*
	 * Set gas pedal state
	 */
	void setGasPedalState(std_msgs::Float64 value);
	/*
	 * Change the mean of the accelerometer.
	 */
	void changeEacc(sensor_msgs::Imu _Eacc);
private:
	Egps _Egps;
	Eimu _Eimu;
	geometry_msgs::PoseWithCovarianceStamped estimatedPose,last_pose;
	geometry_msgs::TwistStamped velocity;
	sensor_msgs::NavSatFix initialGPS, GPSmeasurement;
	sensor_msgs::Imu IMUmeasurement;
	bool first_GPS_flag,GPS_flag,IMU_flag, _while_standing;
	/**
	 * set GPS coordinates
	 */
	void setInitGPS(sensor_msgs::NavSatFix initGPS);

};
