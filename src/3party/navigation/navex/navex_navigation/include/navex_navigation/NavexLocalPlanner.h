/*
 * Filename: NavexLocalPlanner.h
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

#ifndef SRC_NAVEXLOCALPLANNER_H_
#define SRC_NAVEXLOCALPLANNER_H_


#include <iostream>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <navex/planner/local/LocalPlannerBase.h>
#include <navex/trajectory/simulator/models/AckermannModel.h>
#include <navex/trajectory/simulator/models/SkidSteerModel.h>
//#include <navex/trajectory/matcher/GoalTrajectoryMatcher.h>
#include <navex/trajectory/simulator/TrajectorySimulator.h>

#include <navex_navigation/LocalGoalFinder.h>
#include <navex_navigation/NavexLocalPlannerConfig.h>
#include <navex_navigation/NavexGoalTrajectoryMatcher.h>

#include <navex_navigation/FuzzyTrajectoryMatcher.h>

using namespace std;


/**
 * Navex navigation local planner
 */
class NavexLocalPlanner: public LocalPlannerBase {

public:

	/**
	 * Constructs navex local planner
	 * @param costmap Local costmap
	 * @param robotController
	 * @param simulationTime Trajectories simulation time in seconds
	 * @param granularity Trajectory simulation granularity
	 */
	NavexLocalPlanner(CostMap* localCostmap,
			CostMap* globalCostmap,
			IController* robotController, double simulationTime = 2.0,
			double granularity = 0.1);

	virtual ~NavexLocalPlanner();

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
	virtual string getName() const {
		return "navex_local_planner";
	}

	/**
	 * Sets new plan (thread safe)
	 * @param path
	 */
	virtual inline void setPlan(const nav_msgs::Path& path) {
		boost::mutex::scoped_lock lock(planMutex_);

		path_ = path;
		localGoalFinder_.setPath(path);

		planUpdated();
	}

	/**
	 * Gets path (thread safe)
	 * @return
	 */
	virtual inline nav_msgs::Path getPlan() const {
		boost::mutex::scoped_lock lock(planMutex_);

		return path_;
	}

private:

	TrajectorySimulator trajectorySimulator_;
	FuzzyTrajectoryMatcher* trajectoryMathcer_;

	Trajectory::VectorPtr trajectories_;

	ros::Publisher localGoalPublisher_;
	ros::Publisher localPlanPublisher_;
	ros::Publisher trajectoriesPublisher_;

	tf::TransformListener tfListener_;

	mutable boost::mutex planMutex_;

	LocalGoalFinder localGoalFinder_;

	navex_navigation::NavexLocalPlannerConfig config_;

	dynamic_reconfigure::Server<
		navex_navigation::NavexLocalPlannerConfig> configServer_;

private:

	void generateTrajectories();
	void generateTrajectoriesOld();

	/**
	 * Calculates and updates local goal for GoalTrajectoryMatcher
	 * @return True if valid goal found, false otherwise
	 */
	bool updateLocalGoal();

	/**
	 * Evaluates generated trajectories using trajectory matcher
	 */
	TrajectoryMatch::SetPtr evaluateTrajectories( const ITrajectoryMatcher::MatchStrategy strategy );

	bool isLocalGoalInFront() const;

	void publishLocalGoal(const geometry_msgs::PoseStamped pose);

	void publishTrajectories(const TrajectoryMatch::SetPtr& matches);

	void dynamicConfigCallback(navex_navigation::NavexLocalPlannerConfig& config, uint32_t level);

};

#endif /* SRC_NAVEXLOCALPLANNER_H_ */
