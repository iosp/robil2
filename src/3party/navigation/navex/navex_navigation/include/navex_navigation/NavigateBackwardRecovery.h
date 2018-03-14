/*
 * Filename: NavigateBackwardRecovery.h
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


#ifndef INCLUDE_NAVEX_NAVIGATION_NAVIGATEBACKWARDRECOVERY_H_
#define INCLUDE_NAVEX_NAVIGATION_NAVIGATEBACKWARDRECOVERY_H_


#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <navex/planner/local/RecoveryPlannerBase.h>
#include <navex/trajectory/simulator/models/AckermannModel.h>
#include <navex/trajectory/simulator/models/SkidSteerModel.h>
#include <navex/trajectory/matcher/GoalTrajectoryMatcher.h>
#include <navex/trajectory/simulator/TrajectorySimulator.h>

#include <navex_navigation/LocalGoalFinder.h>
#include <navex_navigation/NavexGoalTrajectoryMatcher.h>


/**
 * Conitnues backward navigation for a short period of time
 */
class NavigateBackwardRecovery : public RecoveryPlannerBase {

public:

	NavigateBackwardRecovery(CostMap* localCostmap, CostMap* globalCostmap,
			IController* robotController, double maxDurationSec);

	virtual ~NavigateBackwardRecovery();

public:

	/**
	 * Main control loop
	 * @return
	 */
	virtual PlannerStatus update();

	/**
	 * Gets planner name
	 * @return
	 */
	virtual inline string getName() const {
		return "navigate_backward_recovery";
	}

protected:

	/**
	 * Path updated event callback
	 */
	virtual void planUpdated() {
		localGoalFinder_.setPath(this->getPlan());
	}

private:

	ros::Publisher localGoalPublisher_;
	ros::Publisher localPlanPublisher_;
	ros::Publisher trajectoriesPublisher_;

	LocalGoalFinder localGoalFinder_;

	Trajectory::VectorPtr trajectories_;

	NavexGoalTrajectoryMatcher goalTrajectoryMatcher_;

	boost::posix_time::ptime startTime_;

	double durationSec_;

private:

	void generateTrajectories();

	bool isFrontClear(TrajectoryMatch::SetPtr matches) const;

	bool isLocalGoalInFront() const;

	bool updateLocalGoal();

	void publishLocalGoal(const geometry_msgs::PoseStamped pose);

	TrajectoryMatch::SetPtr evaluateTrajectories( const ITrajectoryMatcher::MatchStrategy strategy );

	void publishTrajectories(const TrajectoryMatch::SetPtr& matches);

};

#endif /* INCLUDE_NAVEX_NAVIGATION_NAVIGATEBACKWARDRECOVERY_H_ */
