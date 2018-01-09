/*
 * Filename: NavexGoalTrajectoryMatcher.h
 *   Author: Igor Makhtes
 *     Date: Jul 23, 2015
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

#ifndef INCLUDE_NAVEX_NAVIGATION_NAVEXGOALTRAJECTORYMATCHER_H_
#define INCLUDE_NAVEX_NAVIGATION_NAVEXGOALTRAJECTORYMATCHER_H_


#include <dynamic_reconfigure/server.h>

#include <navex/trajectory/matcher/GoalTrajectoryMatcher.h>

#include <navex_navigation/GoalTrajectoryMatcherConfig.h>


/**
 * Goal trajectory matcher with dynamic reconfigure
 */
class NavexGoalTrajectoryMatcher : public GoalTrajectoryMatcher {

public:

	NavexGoalTrajectoryMatcher()
		: configServer_(ros::NodeHandle("~/goal_matcher")) {
		configServer_.setCallback(
				boost::bind(&NavexGoalTrajectoryMatcher::dynamicConfigCallback,
						this, _1, _2));
	}

	virtual ~NavexGoalTrajectoryMatcher() { }

private:

	dynamic_reconfigure::Server<
		navex_navigation::GoalTrajectoryMatcherConfig> configServer_;

private:

	/**
	 * Dynamic reconfigure config update callback
	 * @param config
	 * @param level
	 */
	void dynamicConfigCallback(
			navex_navigation::GoalTrajectoryMatcherConfig& config, uint32_t level) {
		setCheckLastPose(config.last_point);
	}

};

#endif /* INCLUDE_NAVEX_NAVIGATION_NAVEXGOALTRAJECTORYMATCHER_H_ */
