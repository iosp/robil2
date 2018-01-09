/*
 * Filename: AckermannController.h
 *   Author: Igor Makhtes
 *     Date: Jun 23, 2015
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef INCLUDE_NAVEX_CONTROLLER_ACKERMANNCONTROLLER_H_
#define INCLUDE_NAVEX_CONTROLLER_ACKERMANNCONTROLLER_H_


#include <string>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <navex/trajectory/simulator/models/AckermannModel.h>

#include <navex/controller/IController.h>


using namespace std;


class AckermannController : public IController {

public:

	/**
	 * Constructs ackermann controller
	 * @param topicName ackermann command topic name
	 * @param scaleSpeed The scaling factor of speed (speed * scaleSpeed)
	 * @param scaleSteerAngle The scaling factor of steering angle (steering_angle * scaleSteerAngle)
	 */
	AckermannController(const string& topicName,
			double scaleSpeed = 1.0, double scaleSteerAngle = 1.0)
		: scaleSpeed_(scaleSpeed), scaleSteerAngle_(scaleSteerAngle) {

		ros::NodeHandle node;
		ackermannPublisher_ = node.advertise<ackermann_msgs::AckermannDriveStamped>(
				topicName, 10, true);
	}

	virtual ~AckermannController() { }

public:

	/**
	 * Set drive command
	 * @param motion
	 */
	virtual void drive(const IMotionModel* motion) const {
		ackermann_msgs::AckermannDriveStamped drive =
				dynamic_cast<const AckermannModel*>(motion)->getAckermannMessage();

		drive.drive.speed *= scaleSpeed_;
		drive.drive.steering_angle *= scaleSteerAngle_;

		ackermannPublisher_.publish(drive);
	}

	/**
	 * Drives forward or backward
	 * @param speed Speed in m/s
	 */
	virtual void driveLinear(double speed) const {
		ackermann_msgs::AckermannDriveStamped drive;

		drive.drive.speed = speed * scaleSpeed_;

		ackermannPublisher_.publish(drive);
	}

	/**
	 * Stops the robot
	 */
	void stop() const {
		driveLinear(0.0);
	}

	inline void setSpeedScale(double scale) {
		scaleSpeed_ = scale;
	}

	inline void setSteerScale(double scale) {
		scaleSteerAngle_ = scale;
	}

private:

	double scaleSpeed_;
	double scaleSteerAngle_;

	ros::Publisher ackermannPublisher_;

};

#endif /* INCLUDE_NAVEX_CONTROLLER_ACKERMANNCONTROLLER_H_ */
