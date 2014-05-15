/*
 * Egps.h
 *
 *  Created on: May 4, 2014
 *      Author: lar5
 */
#include "sensor_msgs/NavSatFix.h"
#include "iostream"
#ifndef EGPS_H_
#define EGPS_H_

class Egps
{
    /*
   * This class collect data from the GPS in order to estimate the initial location of the GPS.
   * This class does not have any effect on the estimation process or on the quality of the estimator. It is only for gazebo reference to try and eliminate the initial position bias.
   * The input of the class is num, and this determines how many measurements will be drawn from the GPS in order to estimate the true initial coordinates.
   */
public:
	bool on,set;
	sensor_msgs::NavSatFix GPSmeasurement;
	int ctr,number_of_measurements;

	Egps(int num)
	{
		set = true;
		on = true;
		ctr = 0;
		number_of_measurements = num;
		std::cout << "number of measurements for GPS: " << number_of_measurements << std::endl;
	}
	~Egps()
	{

	}

	void updateEgps(sensor_msgs::NavSatFix newGPSmeasurement)
	{
		GPSmeasurement.altitude = (GPSmeasurement.altitude*ctr + newGPSmeasurement.altitude)/(ctr+1);
		GPSmeasurement.longitude = (GPSmeasurement.longitude*ctr + newGPSmeasurement.longitude)/(ctr+1);
		GPSmeasurement.latitude = (GPSmeasurement.latitude*ctr + newGPSmeasurement.latitude)/(ctr+1);
		ctr++;
		if (ctr == number_of_measurements)
			on = false;
	}

	sensor_msgs::NavSatFix getGPScoor()
	{
		return GPSmeasurement;
	}
};



#endif /* EGPS_H_ */
