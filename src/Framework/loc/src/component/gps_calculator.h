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
	double R = 6371.0; //[km]
	double lat1 = p1.latitude*PI/180.0;
	double lat2 = p2.latitude*PI/180.0;
	double dLat = lat2 - lat1;
	double dLon = (p2.longitude - p1.longitude)*PI/180.0;

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
inline sensor_msgs::NavSatFix calcLatLon(sensor_msgs::NavSatFix p1,double distance, double brng)
{
	/**
	 * calculate the latitude and longitude given distance and bearing
	 * distnace input in [m] brng in [rad]
	 */
	double R = 6371.0 * 1000; //[km]
	sensor_msgs::NavSatFix p2;
	double lat1 = p1.latitude*PI/180.0; //Current lat point converted to radians
	double lon1 = p1.longitude*PI/180.0; //Current long point converted to radians
		
	p2.latitude = (asin( sin(lat1)*cos(distance/R) + cos(lat1)*sin(distance/R)*cos(brng)));
	p2.longitude = (lon1 + atan2(sin(brng)*sin(distance/R)*cos(lat1),cos(distance/R)-sin(lat1)*sin(p2.latitude)))*180 / PI;
	p2.latitude = p2.latitude *180 / PI;

	return p2;
}


#endif /* GPS_CALCULATOR_H_ */
