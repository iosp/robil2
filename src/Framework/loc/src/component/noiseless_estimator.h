#include <math.h>
#include "ros/ros.h"
#include <ParameterTypes.h>
#include <opencv2/opencv.hpp>

class Observer {
	/*
	 * The observer class is a noise free environment estimator.
	 * It receives GPS & IMU measurements and calculates the location and speed (X,Y,Z,roll,Pitch,yaw, linear speed and angular speed).
	 *
	 */
public:
  /**
   * Constructor:
   */
	Observer();
  /**
   * Destructor:
   */
  virtual ~Observer();
  /**
   * set private variable GPSmeasurement
   */
  void setGPSMeasurement(sensor_msgs::NavSatFix measurement);
  /**
   * set private variable IMUmeasurement
   */
  void setIMUMeasurement(sensor_msgs::Imu measurement);
  /*
  * set the speed measurement that the GPS provides
  */
  void setGPSSpeedMeasurement(robil_msgs::GpsSpeed measurement);
  
  /**
   * This function performs the estimation of location.
   */
  void estimator();
  /**
   * return the estimated position
   */
  geometry_msgs::PoseWithCovarianceStamped getEstimatedPose();
  /**
   * return the estimated speed
   */
  geometry_msgs::TwistStamped getEstimatedSpeed();
private:
    geometry_msgs::PoseWithCovarianceStamped estimatedPose, lastPose;
    geometry_msgs::TwistStamped velocity;
    sensor_msgs::NavSatFix initialGPS, GPSmeasurement;
    sensor_msgs::Imu IMUmeasurement;
    robil_msgs::GpsSpeed speedMeasurement;
    bool first_GPS_flag;
    double dx, dy;
    int _ready;
    /**
    * set GPS coordinates
    */
    void setInitGPS(sensor_msgs::NavSatFix initGPS);
};
