/*
 * Filename: SkidSteerModel.h
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


#ifndef INCLUDE_NAVEX_TRAJECTORY_SIMULATOR_MODELS_SKIDSTEERMODEL_H_
#define INCLUDE_NAVEX_TRAJECTORY_SIMULATOR_MODELS_SKIDSTEERMODEL_H_


#include <geometry_msgs/Twist.h>

#include <navex/trajectory/simulator/models/IMotionModel.h>


/**
 * Skid steer motion model
 */
class SkidSteerModel : public IMotionModel {

public:

	/**
	 * Constructs a skid steer model with provided velocities
	 * @param linearVelocity Linear velocity
	 * @param angularVelocity Angular velocity
	 */
	SkidSteerModel(double linearVelocity, double angularVelocity)
		: IMotionModel(true), linearVelocity_(linearVelocity),
		  angularVelocity_(angularVelocity) { }

	SkidSteerModel(const SkidSteerModel& other)
		: IMotionModel(other.isConstantMotion()) {
		linearVelocity_ = other.getLinearVelocity();
		angularVelocity_ = other.getAngularVelocity();
	}

public:

	virtual tf::Transform simulateStep(double timeDelta) const {
		tf::Transform velocityVector;
		velocityVector.setOrigin(tf::Vector3(linearVelocity_ * timeDelta, 0, 0));
		velocityVector.setRotation(tf::createQuaternionFromYaw(angularVelocity_ *  timeDelta));
		return velocityVector;
	}

	/**
	 * Returns linear velocity
	 * @return
	 */
	inline double getLinearVelocity() const {
		return linearVelocity_;
	}

	/**
	 * Returns angular velocity
	 * @return
	 */
	inline double getAngularVelocity() const {
		return angularVelocity_;
	}

	/**
	 * Returns linear and angular velocities as geometry_msgs::Twist
	 * @return
	 */
	geometry_msgs::Twist getTwistMessage() const {
		geometry_msgs::Twist twistMessage;
		twistMessage.linear.x = getLinearVelocity();
		twistMessage.angular.z = getAngularVelocity();
		return twistMessage;
	}

private:

	double linearVelocity_;
	double angularVelocity_;

};

#endif /* INCLUDE_NAVEX_TRAJECTORY_SIMULATOR_MODELS_SKIDSTEERMODEL_H_ */
