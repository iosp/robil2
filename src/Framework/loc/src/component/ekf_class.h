#include <math.h>
//#include "geometry_msgs/PoseWithCovarianceStamped.h"
//#include "geometry_msgs/TwistStamped.h"
//#include "sensor_msgs/NavSatFix.h"
//#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include <ParameterTypes.h>
const double PI  =3.141592653589793238463;

class ekf {
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
   * calc_distance calculates the distance between two latitude/longitutde points.
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
private:
    geometry_msgs::PoseWithCovarianceStamped estimatedPose;
    geometry_msgs::TwistStamped velocity;
    sensor_msgs::NavSatFix initialGPS, GPSmeasurement;
    sensor_msgs::Imu IMUmeasurement;
    bool GPS_flag;
    
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