#include <math.h>
#include "EKF_properties.h"
//#include "geometry_msgs/PoseWithCovarianceStamped.h"
//#include "geometry_msgs/TwistStamped.h"
//#include "sensor_msgs/NavSatFix.h"
//#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include <ParameterTypes.h>
#include <opencv2/opencv.hpp>
const double PI  =3.141592653589793238463;

class ekf:private ekf_props {
public:
  /**
   * Constructor:
   */
  ekf();
  /**
   * Destructor:
   */
  virtual ~ekf();
  /**
   * set private variable GPSmeasurement
   */
  void setGPSMeasurement(sensor_msgs::NavSatFix measurement);
  /**
   * set private variable IMUmeasurement
   */
  void setIMUMeasurement(sensor_msgs::Imu measurement);
  /**
   * calc_distance calculates the distance between two latitude/longitude points.
   * The equations are taken from the following site:
   * http://www.movable-type.co.uk/scripts/latlong.html
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
  /*
   * Kalman measurement update stage.
   */
  void measurement_update();
  /*
   * Kalman time propagation stage.
   */
  void time_propagation();
private:
    geometry_msgs::PoseWithCovarianceStamped estimatedPose;
    geometry_msgs::TwistStamped velocity;
    sensor_msgs::NavSatFix initialGPS, GPSmeasurement;
    sensor_msgs::Imu IMUmeasurement;
    bool first_GPS_flag,GPS_flag,IMU_flag;
    
    /**
    * set GPS coordinates
    */
    void setInitGPS(sensor_msgs::NavSatFix initGPS);
    double calcDistance(sensor_msgs::NavSatFix p1,sensor_msgs::NavSatFix p2);
    /**
    * calc the bearing from my initial position
    */
    double calcBearing(sensor_msgs::NavSatFix p1,sensor_msgs::NavSatFix p2);


};
