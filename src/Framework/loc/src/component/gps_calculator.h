#ifndef GPS_CALCULATOR_H_
#define GPS_CALCULATOR_H_
const double PI  = 3.141592653589793238463;
inline double calcDistance(sensor_msgs::NavSatFix p1,sensor_msgs::NavSatFix p2)
{
	/**
	 * calc_distance calculates the distance between two latitude/longitude points.
	 * The equations are taken from the following site:
	 * http://www.movable-type.co.uk/scripts/latlong.html
	 */
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
inline double calcBearing(sensor_msgs::NavSatFix p1,sensor_msgs::NavSatFix p2)
{
	/**
	 * calc the bearing from my initial position
	 */
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



#endif /* GPS_CALCULATOR_H_ */
