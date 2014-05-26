/*
 * Eimu.h
 *
 *  Created on: May 11, 2014
 *      Author: lar5
 */

#ifndef EIMU_H_
#define EIMU_H_
#include "sensor_msgs/Imu.h"
class Eimu
{
  /*
   * This class collect data from the IMU in order to estimate the bias of the IMU.
   * The input of the class is num, and this determines how many measurements will be drawn from the IMU in order to estimate the bias.
   */
public:
	bool on,set;
	sensor_msgs::Imu _Eimu;
	int ctr,number_of_measurements;

	Eimu(int num)
	{
		set = true;
		on = true;
		ctr = 0;
		number_of_measurements = num;
		std::cout << "number of measurements for IMU: " << number_of_measurements << std::endl;
	}
	~Eimu()
	{

	}

	void updateEimu(sensor_msgs::Imu newIMUmeasurement)
	{
		_Eimu.linear_acceleration.x = (_Eimu.linear_acceleration.x * ctr + newIMUmeasurement.linear_acceleration.x) / (ctr + 1);
		ctr++;
		if (ctr == number_of_measurements)
			on = false;
	}

	sensor_msgs::Imu getEIMU()
	{
		return _Eimu;
	}
};



#endif /* EIMU_H_ */
