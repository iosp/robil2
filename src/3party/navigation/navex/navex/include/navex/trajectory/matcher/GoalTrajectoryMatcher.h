/*
 * Filename: GoalTrajectoryMatcher.h
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


#ifndef INCLUDE_NAVEX_TRAJECTORY_MATCHER_GOALTRAJECTORYMATCHER_H_
#define INCLUDE_NAVEX_TRAJECTORY_MATCHER_GOALTRAJECTORYMATCHER_H_



#include <cmath>
#include <navex/trajectory/matcher/ITrajectoryMatcher.h>


/**
 * Trajectory matcher which gives a higher score to trajectories
 * that are closer to goal
 */
class GoalTrajectoryMatcher: public ITrajectoryMatcher {

public:

	GoalTrajectoryMatcher();

	virtual ~GoalTrajectoryMatcher();

public:

	using ITrajectoryMatcher::match;

	/**
	 * Sets goal
	 * @param goal
	 */
	inline void setGoal(const cv::Point& goal) {
		goal_ = goal;
	}

	/**
	 * Gets the current goal
	 * @return
	 */
	inline cv::Point getGoal() const {
		return goal_;
	}

	/**
	 * If true, scores the trajectory only by the distance of the last pose on path
	 * @param checkLastPose
	 */
	inline void setCheckLastPose(bool checkLastPose) {
		lastPoseDistance_ = checkLastPose;
	}

	/**
	 * Evaluate one trajectory
	 * @param costMap
	 * @param trajectory
	 * @return
	 */
	virtual TrajectoryMatch::Ptr match(const CostMap& localCostMap,
			const CostMap& globalCostMap,
			const Trajectory::Ptr& trajectory,
			const ITrajectoryMatcher::MatchStrategy strategy ) const;
	virtual TrajectoryMatch::Ptr matchOld(const CostMap& localCostMap,
			const CostMap& globalCostMap,
			const Trajectory::Ptr& trajectory) const;

private:

	cv::Point goal_;

	/**
	 * If true, scores the trajectory only by the distance of the last pose on path
	 */
	bool lastPoseDistance_;

private:

	/**
	 * Sigmoid function - monotonically increasing function
	 * @param x Real number
	 * @return A value between 0.0 and 1.0 exclusive
	 */
	inline double sigmoid(double x) const {
		return 1.0 / (1.0 + exp(-x));
	}

	/**
	 * Inverted sigmoid function - monotonically decreasing function
	 * @param x Real number
	 * @return A vlue between 0.0 and 1.0 exclusive
	 */
	inline double invsigmoid(double x) const {
		return 1.0 / (1.0 + exp(x));
	}

	double Pg(const tf::Vector3 & goal, const tf::Vector3 & robot, const tf::Vector3 & pose)const;
	double Po(const tf::Vector3 & goal, const tf::Vector3 & robot, const tf::Vector3 & pose)const;
	double Pf(const CostMap & localCostMap, const geometry_msgs::PoseStamped & pose)const;
};

#endif /* INCLUDE_NAVEX_TRAJECTORY_MATCHER_GOALTRAJECTORYMATCHER_H_ */
