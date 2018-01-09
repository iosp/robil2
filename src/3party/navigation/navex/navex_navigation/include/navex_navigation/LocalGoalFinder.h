/*
 * Filename: LocalGoalFinder.h
 *   Author: Igor Makhtes
 *     Date: Jul 14, 2015
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


#ifndef SRC_LOCALGOALFINDER_H_
#define SRC_LOCALGOALFINDER_H_


#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <navex/costmap/CostMap.h>


using namespace std;


/**
 * Finds local goal on path for local planner
 */
class LocalGoalFinder {

public:

	LocalGoalFinder(const string& robotFrameId);

	virtual ~LocalGoalFinder();

public:

	inline void setPath(const nav_msgs::Path& path) {
		path_ = path;
		lastClosestPointIndex_ = 0;
	}

	int findLocalGoal();

	inline void setGoalDistance(double distance) {
		goalDistance_ = distance;
	}


	inline int getLastLocalGoal() const {
		return lastGoal_;
	}

private:

	int lastClosestPointIndex_;

	int lastGoal_;

	tf::TransformListener tfListener_;

	nav_msgs::Path path_;
	string robotFrameId_;

	double goalDistance_;

private:

	size_t findClosestPoint(const nav_msgs::Path& path) const;

};

#endif /* SRC_LOCALGOALFINDER_H_ */
