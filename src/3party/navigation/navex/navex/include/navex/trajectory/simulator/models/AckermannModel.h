/*
 * Filename: AckermannModel.h
 *   Author: Igor Makhtes
 *     Date: Dec 1, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014
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


#ifndef ACKERMANNMODEL_H_
#define ACKERMANNMODEL_H_


#include <ackermann_msgs/AckermannDriveStamped.h>

#include <navex/trajectory/simulator/models/IMotionModel.h>


/**
 * Simplified ackermann motion model
 * @note Motion model of a bicycle - doesn't take into account two front steering wheels
 */
class AckermannModel : public IMotionModel {

public:

	/**
	 * Constructs the ackermann motion model
	 * @param wheelSeparation The distance between front and rear wheels in meters
	 * @param speed Speed in m/s
	 * @param steeringAngle Steering angle in radians
	 */
	AckermannModel(double wheelSeparation, double speed,
			double steeringAngle)
		: IMotionModel(true), wheelBase_(wheelSeparation),
		  speed_(speed), steeringAngle_(steeringAngle) { }

	/**
	 * Simulate the transformation of a single step
	 * @param timeDelta
	 * @return
	 */
	virtual tf::Transform simulateStep(double timeDelta) const {
		const double steeringAngle = steeringAngle_;
		const double distance = speed_ * timeDelta;

		double offsetX = 0;
		double offsetY = 0;
		double heading = 0;

		if (fabs(steeringAngle_) < 0.001) {
			offsetX = distance;
			offsetY = 0;
		} else {
			double radius = wheelBase_ / tan(steeringAngle);
			heading = distance / radius;
			offsetX = radius * sin(heading);
			offsetY = radius - radius * cos(heading);

			if (isnan(offsetX) || isnan(offsetY)) {
				offsetX = 0;
				offsetY = 0;
			}
		}

		return tf::Transform(
				tf::createQuaternionFromYaw(heading),
				tf::Vector3(offsetX, offsetY, 0));
	}

	/**
	 * Returns the distance between front and rear wheels in meters
	 * @return
	 */
	inline double getWheelSeparation() const {
		return wheelBase_;
	}

	/**
	 * Return the speed in m/s
	 * @return
	 */
	inline double getSpeed() const {
		return speed_;
	}

	/**
	 * Returns the steering angle in radians
	 * @return
	 */
	inline double getSteeringAngle() const {
		return steeringAngle_;
	}

	/**
	 * Returns speed and steering angle as @see ackermann_msgs::AckermannDriveStamped
	 * @return
	 */
	inline ackermann_msgs::AckermannDriveStamped getAckermannMessage() const {
		ackermann_msgs::AckermannDriveStamped message;
		message.drive.speed = speed_;
		message.drive.steering_angle = steeringAngle_;
		return message;
	}

private:

	/**
	 * The distance between the centers of the front and rear wheels
	 */
	double wheelBase_;
	double steeringAngle_;
	double speed_;

};

#endif /* ACKERMANNMODEL_H_ */
