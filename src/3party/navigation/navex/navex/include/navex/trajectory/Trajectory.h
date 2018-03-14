/*
 * Filename: Trajectory.h
 *   Author: Igor Makhtes
 *     Date: Nov 25, 2014
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


#ifndef INCLUDE_NAVEX_TRAJECTORY_H_
#define INCLUDE_NAVEX_TRAJECTORY_H_


#include <vector>

#include <boost/foreach.hpp>

#include <nav_msgs/Path.h>
#include <navex/trajectory/simulator/models/IMotionModel.h>
#include <tf/tf.h>


using namespace std;


#define foreach BOOST_FOREACH


/**
 * Represents a single simulated trajectory
 */
class Trajectory {

public:

	typedef boost::shared_ptr<Trajectory> Ptr;
	typedef boost::shared_ptr<Trajectory const> ConstPtr;
	typedef vector<Trajectory::Ptr> Vector;
	typedef boost::shared_ptr<Vector> VectorPtr;

public:

	Trajectory(const IMotionModel* motionModel, double weight = 1);

public:

	/**
	 * Adds position to the path with zero rotation
	 * @param x
	 * @param y
	 */
	void addPosition(double x, double y);

	/**
	 * Adds position to the path
	 * @param position
	 * @param orientation
	 */
	void addPosition(const tf::Vector3& position, const tf::Quaternion& orientation);

	/**
	 * Returns path with updated time stamp and frame id
	 * @param updateStamp if true, sets stamp to current time
	 * @param frameId frame id
	 * @return
	 */
	nav_msgs::Path::Ptr getPath(
			bool updateStamp, const string& frameId);

	/**
	 * Returns path with a default frame id and old time stamp
	 * @return
	 */
	inline nav_msgs::Path::Ptr getPath() const {
		return path_;
	}

	/**
	 * Removes all points from the path
	 */
	inline void clearPath() {
		path_->poses.clear();
	}

	/**
	 * Sets weight
	 * @param weight Value in range [0, 1]
	 * @throw std::invalid_argument
	 */
	void setWeight(double weight);

	/**
	 * Gets the weight
	 * @return
	 */
	inline double getWeight() const {
		return weight_;
	}

	/**
	 * Gets underlying motion model
	 * @return
	 */
	inline const IMotionModel* getMotionModel() const {
		return motionModel_.get();
	}

	/**
	 * Gets underlying typed motion model
	 * @return
	 */
	template <class MotionModelType>
	inline const MotionModelType* getMotionModelAs() const {
		return (MotionModelType*)getMotionModel();
	}

	/**
	 * Rotates all path by specified angle
	 * @param angle in radians
	 */
	void rotate(double angle);

	/**
	 * Returns true if trajectory changes the orientation only
	 * @return
	 */
	bool isInPlace() const;

	/**
	 * Returns true if the trajectory leads forward at the beginning
	 * @return
	 */
	bool isForward() const;

	/**
	 * Returns true if the trajectory leads backward at the beginning
	 * @return
	 */
	inline bool isBackward() const {
		return !isForward() && !isInPlace();
	}

	/**
	 * Concatenates provided trajectory to the end of this trajectory
	 * @param trajectory
	 */
	void concatenate(const Trajectory& trajectory);

	double getF1()const;
	double getF2()const;
	void setFactors(double f1, double f2);

	/**
	 *
	 * @param override
	 * @param value
	 */
	inline void overrideInPlace(bool override, bool value) {
		overrideInPlace_ = override;
		overrideInPlaceValue_ = value;
	}

private:

	nav_msgs::Path::Ptr path_;

	/**
	 * Trajectory's weight
	 */
	double weight_;

	/**
	 * Underlying motion model
	 */
	boost::shared_ptr<IMotionModel> motionModel_;

	/**
	 * If true, overrides inPlace value with overrideInPlaceValue
	 */
	bool overrideInPlace_;

	/**
	 * If overrideInPlace set to true, this value is returned instead of
	 * the actual path trajectory
	 */
	bool overrideInPlaceValue_;

	double f1_, f2_;

};

#endif /* INCLUDE_NAVEX_TRAJECTORY_H_ */
