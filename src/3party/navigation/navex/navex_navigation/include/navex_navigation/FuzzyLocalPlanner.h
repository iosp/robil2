/*
 * FuzzyLocalPlanner.h
 *
 *  Created on: Jan 15, 2017
 *      Author: assaf
 */

#ifndef NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZYLOCALPLANNER_H_
#define NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZYLOCALPLANNER_H_

#include <iostream>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <navex/planner/local/LocalPlannerBase.h>
#include <navex/trajectory/simulator/models/SkidSteerModel.h>
#include <navex/trajectory/simulator/TrajectorySimulator.h>

#include <navex_navigation/LocalGoalFinder.h>
#include <navex_navigation/NavexLocalPlannerConfig.h>
#include <navex_navigation/NavexGoalTrajectoryMatcher.h>

#include <navex_navigation/FuzzyTrajectoryMatcher.h>


class FuzzyLocalPlanner : public LocalPlannerBase
{
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

	std::string global_navigation_frame_id_;

private:
	TrajectoryMatch::SetPtr evaluateTrajectories( const ITrajectoryMatcher::MatchStrategy strategy );
	void publishLocalGoal(const geometry_msgs::PoseStamped pose);
	void publishTrajectories(const TrajectoryMatch::SetPtr& matches);
	bool updateLocalGoal();
	void generateTrajectories();

	void generateTrajectories_forward(Trajectory::VectorPtr trajectories);
	void generateTrajectories_backward(Trajectory::VectorPtr trajectories);
	void generateTrajectories_inplace_turn(Trajectory::VectorPtr trajectories);
	void generateTrajectories_angle_iterations(
			Trajectory::VectorPtr trajectories, double speed, double skip_angles_range, bool dynamic_step,
			double minSpeed, double maxSpeed);

	void dynamicConfigCallback(navex_navigation::NavexLocalPlannerConfig& config, uint32_t level);
	void createTrajectory(double angle, double speed, Trajectory::VectorPtr& trajectories);

public:
	FuzzyLocalPlanner(CostMap* localCostmap,
			CostMap* globalCostmap,
			IController* robotController,
			const std::string & global_navigation_frame_id = "WORLD",
			double simulationTime = 2.0,
			double granularity = 0.1);

	virtual ~FuzzyLocalPlanner();

	virtual PlannerStatus update();

public:
	virtual std::string getName() const
	{
		return "navex_local_planner";
	}

	void setNavigationGlobalFrameID(const std::string & frame_id)
	{
		global_navigation_frame_id_ = frame_id;
	}

	virtual inline void setPlan(const nav_msgs::Path& path)
	{
		boost::mutex::scoped_lock lock(planMutex_);

		path_ = path;
		localGoalFinder_.setPath(path);

		planUpdated();
	}

	virtual inline nav_msgs::Path getPlan() const
	{
		boost::mutex::scoped_lock lock(planMutex_);

		return path_;
	}
};



#endif /* NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZYLOCALPLANNER_H_ */
