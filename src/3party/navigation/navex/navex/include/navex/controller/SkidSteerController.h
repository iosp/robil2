/*
 * Filename: SkidSteerController.h
 *   Author: Igor Makhtes
 *     Date: Jun 28, 2015
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

#ifndef INCLUDE_NAVEX_CONTROLLER_SKIDSTEERCONTROLLER_H_
#define INCLUDE_NAVEX_CONTROLLER_SKIDSTEERCONTROLLER_H_


#include <geometry_msgs/Twist.h>

#include <navex/controller/IController.h>
#include <navex/trajectory/simulator/models/SkidSteerModel.h>


/**
 * Skid steer drive model controller
 */
class SkidSteerController: public IController {

public:

	SkidSteerController(const string& topicName = "/cmd_vel",
			double scaleLinear = 1.0, double scaleAngular = 1.0)
		: scaleLinear_(scaleLinear), scaleAngular_(scaleAngular) {
		ros::NodeHandle node;

		skidPublisher_ = node.advertise<geometry_msgs::Twist>(
				topicName, 10, true);
	}

	virtual ~SkidSteerController() { }

public:

	inline void setLinearScale(double scale) {
		scaleLinear_ = scale;
	}

	inline void setAngularScale(double scale) {
		scaleAngular_ = scale;
	}

private:

	virtual void drive(const IMotionModel* motion) const {
		geometry_msgs::Twist drive =
				dynamic_cast<const SkidSteerModel*>(motion)->getTwistMessage();

		drive.linear.x  *= scaleLinear_;
		drive.angular.z *= scaleAngular_;

		skidPublisher_.publish(drive);
	}

	/**
	 * Drives forward or backward
	 * @param speed Speed in m/s
	 */
	virtual void driveLinear(double speed) const {
		geometry_msgs::Twist drive;

		drive.linear.x = speed * scaleLinear_;

		skidPublisher_.publish(drive);
	}

	/**
	 * Stops the robot
	 */
	inline virtual void stop() const {
		driveLinear(0.0);
	}

private:

	double scaleLinear_;
	double scaleAngular_;

	ros::Publisher skidPublisher_;

};

#endif /* INCLUDE_NAVEX_CONTROLLER_SKIDSTEERCONTROLLER_H_ */
