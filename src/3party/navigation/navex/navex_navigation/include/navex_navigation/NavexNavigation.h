/*
 * Filename: NavexNavigation.h
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


#ifndef SRC_NAVEXNAVIGATION_H_
#define SRC_NAVEXNAVIGATION_H_


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib_msgs/GoalID.h>
#include <navex_msgs/VersionService.h>

#include <navex/costmap/datasource/CostMapDataSourceBase.h>
#include <navex/costmap/datasource/LaserScanDataSource.h>
#include <navex/costmap/datasource/PointCloudDataSource.h>
#include <navex/costmap/datasource/RosTopicMapDataSource.h>
#include <navex/costmap/parameters/RosParametersProvider.h>
#include <navex/costmap/parameters/DynamicConfigParametersProvider.h>
#include <navex/path_search/AStar.h>
#include <navex/planner/global/AStarGlobalPlanner.h>
#include <navex/planner/local/RecoveryPlannerBase.h>
#include <navex/controller/AckermannController.h>
#include <navex/controller/SkidSteerController.h>
#include <navex/costmap/datasource/PointCloudReconfiguredDataSource.h>

#include <navex_navigation/FuzzyLocalPlanner.h>
#include <navex_navigation/FuzzyGlobalPlanner.h>
#include <navex_navigation/NavexLocalPlanner.h>
#include <navex_navigation/NavexGlobalPlanner.h>
#include <navex_navigation/MoveBackwardRecovery.h>
#include <navex_navigation/NavigateBackwardRecovery.h>
#include <navex_navigation/NavigationParametersConfig.h>

#include <move_base_msgs/MoveBaseActionGoal.h>

#include <std_msgs/Float32.h>

using namespace std;

#define NAVEX_ROBIL_VERSION "Cogniteam-Robil 1.0.0"

/**
 * Navex navigation control class
 */
class NavexNavigation {

public:

	enum NavigationState {
		STATE_IDLE,
		STATE_NAVIGATION,
		STATE_PLANNING,
		STATE_RECOVERY,
		STATE_PLANNING_FAILED
	};

public:

	NavexNavigation(double loopRate);

	virtual ~NavexNavigation();

public:

	void spin();

	/**
	 * Gets current navigation state
	 * @return
	 */
	inline NavigationState getState() const {
		boost::mutex::scoped_lock lock(stateMutex_);
		return navigationState_;
	}

	/**
	 * Sets navigation state
	 * @param state
	 */
	inline void setState(NavigationState state) {
		boost::mutex::scoped_lock lock(stateMutex_);
		navigationState_ = state;
	}

	inline void setTemporalGoal(const geometry_msgs::PoseStamped& pose) {
		boost::mutex::scoped_lock lock(stateMutex_);
		temporal_goal_ = pose;
	}

	/**
	 * Immediately stops the navigation process and stops the robot
	 */
	void stopNavigation();

private:

	ros::NodeHandle node_;

	ros::Timer localCostmapPublishTimer_;
	ros::Timer globalCostmapPublishTimer_;

	CostMap* localCostmap_;
	CostMap* globalCostmap_;
	IController* robotController_;
	GlobalPlannerBase* globalPlanner_;
	LocalPlannerBase* localPlanner_;

	double loopRate_;

	bool globalMapPublished_;

	tf::TransformListener tfListener_;

	ros::Publisher globalCostmapPublisher_;
	ros::Publisher localCostmapPublisher_;
	ros::Publisher pathPublisher_;
	ros::Publisher statePublisher_;
	ros::Publisher goalPublisher_;
	ros::Publisher temporalGoalPublisher_;
	ros::Publisher distanceToGoalPublisher_;

	ros::Subscriber goalSubscriber_;
	ros::Subscriber simpleGoalSubscriber_;
	ros::Subscriber stopSubscriber_;

	ros::ServiceServer versionServer_;

	geometry_msgs::PoseStamped goal_;
	geometry_msgs::PoseStamped temporal_goal_;
	boost::shared_ptr<boost::thread> backgroundPlanThread_;
	volatile NavigationState navigationState_;

	bool firstPlanning_;

	mutable boost::mutex stateMutex_;

	RecoveryPlannerBase* recoveryPlanner_;

	ros::ServiceServer makePlanService_;

	navex_navigation::NavigationParametersConfig dynamicConfig_;

	dynamic_reconfigure::Server<
		navex_navigation::NavigationParametersConfig> dynamicConfigServer_;

private:

	void preparePlanAsync(const geometry_msgs::PoseStamped& goal,
			bool forceNavigation = true);

	std::string globalFrameId()const;
	void transform_path_to_golobal_frame(nav_msgs::Path& path);
	void transform_to_goal_frame(geometry_msgs::PoseStamped& pose);
	void transform_to_global_map_frame(geometry_msgs::PoseStamped& pose);
	void preparePlanThread(const geometry_msgs::PoseStamped goal);

	void simpleGoalCallback(const geometry_msgs::PoseStamped::Ptr& goal);
	void goalCallback(const move_base_msgs::MoveBaseActionGoal::Ptr& goal);

	void stopCallback(const actionlib_msgs::GoalID::Ptr& stop);

	bool isGoalReached() const;
	bool isTemporalGoalReached() const;
	bool isTemporalGoalExists() const;

	bool makePlanCallback(nav_msgs::GetPlan::Request& request,
			nav_msgs::GetPlan::Response& response);

	/**
	 * Returns the position of base_frame in global map's frame
	 * @return
	 */
	geometry_msgs::PoseStamped getBaseFramePose() const;
	geometry_msgs::PoseStamped getGlobalFramePose() const;

	/**
	 * Publishes current navigation state as visualization marker
	 */
	void publishStateMarker() const;

	/**
	 * Local cost map publish loop callback
	 * @param timer
	 */
	void publishLocalCostmapCallback(ros::TimerEvent timer) const;

	/**
	 * Global cost map publish loop callback
	 * @param timer
	 */
	void publishGlobalCostmapCallback(ros::TimerEvent timer) const;

	bool versionServiceCallback(navex_msgs::VersionService::Request & req, navex_msgs::VersionService::Response & resp);

	/**
	 * Dynamic reconfiguration callback
	 * @param config
	 * @param level
	 */
	void dynamicConfigCallback(
			navex_navigation::NavigationParametersConfig& config, uint32_t level);

};

#endif /* SRC_NAVEXNAVIGATION_H_ */
