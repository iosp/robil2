/*
 * Filename: NavexNavigation.cpp
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


#include <navex_navigation/NavexNavigation.h>
#include <geometry_msgs/PointStamped.h>


NavexNavigation::NavexNavigation(double loopRate)
	:localCostmap_(NULL),globalCostmap_(NULL),
		robotController_(new SkidSteerController("/cmd_vel", 1.0, 1.0)),
		loopRate_(loopRate), globalMapPublished_(false),
		navigationState_(STATE_IDLE), firstPlanning_(true),
		recoveryPlanner_(NULL), dynamicConfigServer_(ros::NodeHandle("~/navigation")) {

	ros::NodeHandle nodePrivate("~");

	double global_update_rate = nodePrivate.param<double>("global_costmap/update_rate",0);
	double local_update_rate = nodePrivate.param<double>("local_costmap/update_rate",0);

	globalCostmap_ = new CostMap(new PointCloudReconfiguredDataSource<pcl::PointXYZ>("/scan",new PCDataDynamicConfigParameterProvider("global_costmap/data"),global_update_rate,ros::NodeHandle()),
			new DynamicConfigParametersProvider("global_costmap"));

	localCostmap_ = new CostMap(new PointCloudReconfiguredDataSource<pcl::PointXYZ>("/scan",new PCDataDynamicConfigParameterProvider("local_costmap/data"),local_update_rate,ros::NodeHandle()),
			new DynamicConfigParametersProvider("local_costmap"));

	localPlanner_ = new FuzzyLocalPlanner(localCostmap_, globalCostmap_, robotController_);
	globalPlanner_ = new FuzzyGlobalPlanner();

	pathPublisher_ = node_.advertise<nav_msgs::Path>("path", 1, true);

	globalCostmapPublisher_ =
			node_.advertise<nav_msgs::OccupancyGrid>("global_costmap", 1, true);

	localCostmapPublisher_ =
			node_.advertise<nav_msgs::OccupancyGrid>("local_costmap", 1, true);

	goalPublisher_ =
			node_.advertise<geometry_msgs::PointStamped>("global_goal", 1, true);
	temporalGoalPublisher_ =
			node_.advertise<geometry_msgs::PointStamped>("global_temporal_goal", 1, true);

	distanceToGoalPublisher_ =
			node_.advertise<std_msgs::Float32>("distance_to_goal",1,true);

	goalSubscriber_ = nodePrivate.subscribe("goal", 1, &NavexNavigation::goalCallback, this);

	simpleGoalSubscriber_ = nodePrivate.subscribe("simple_goal", 1, &NavexNavigation::simpleGoalCallback, this);

	stopSubscriber_ = nodePrivate.subscribe("cancel", 1, &NavexNavigation::stopCallback, this);

	statePublisher_ = node_.advertise<visualization_msgs::Marker>("navigation_state", 1, true);

	localCostmapPublishTimer_ = node_.createTimer(ros::Duration(0.2),
			boost::bind(&NavexNavigation::publishLocalCostmapCallback, this, _1));

	globalCostmapPublishTimer_ = node_.createTimer(ros::Duration(2.0),
			boost::bind(&NavexNavigation::publishGlobalCostmapCallback, this, _1));

	makePlanService_ = node_.advertiseService("/move_base/NavfnROS/make_plan",
			&NavexNavigation::makePlanCallback, this);

	dynamicConfigServer_.setCallback(
			boost::bind(&NavexNavigation::dynamicConfigCallback, this, _1, _2));

	versionServer_ = node_.advertiseService("/move_base/version",&NavexNavigation::versionServiceCallback,this);
}

NavexNavigation::~NavexNavigation() {
	delete localCostmap_;
	delete globalCostmap_;
	delete localPlanner_;
	delete globalPlanner_;
	delete robotController_;

	if (recoveryPlanner_ == NULL)
		delete recoveryPlanner_;
}

void NavexNavigation::spin() {
	ros::NodeHandle node;

	ros::Rate rate(loopRate_);
	static ros::Time prev_time = ros::Time::now();

	while (ros::ok()) {

		if (globalCostmap_->getOccupancyGrid()->data.size() > 0 && !globalMapPublished_) {
			// Global map ready
			globalCostmapPublisher_.publish(globalCostmap_->getOccupancyGrid());
			globalMapPublished_ = true;

			ROS_INFO_NAMED("NavexNavigation", "Global map ready!");
		}

		if (!globalMapPublished_) {
			ROS_WARN_ONCE_NAMED("NavexNavigation", "Global map not ready");
		}

		if (getState() == STATE_NAVIGATION) {
			/*
			 * Navigation state
			 */
			LocalPlannerBase::PlannerStatus status = localPlanner_->update();

			if (status == LocalPlannerBase::STATUS_OK) {
				// OK!
			} else if (status == LocalPlannerBase::STATUS_IN_OBSTACLE) {
				// Start recovery behavior
			} else if (status == LocalPlannerBase::STATUS_NO_PLAN_FOUND) {
				// Start recovery

				if (recoveryPlanner_ != NULL) {
					delete recoveryPlanner_;
					recoveryPlanner_ = NULL;
				}

				recoveryPlanner_ = new NavigateBackwardRecovery(
						localCostmap_, globalCostmap_, robotController_, 10.0);

				recoveryPlanner_->setPlan(localPlanner_->getPlan());

				ROS_INFO("Local plan not found, starting recovery");
				ROS_INFO("  - Recovery: %s", recoveryPlanner_->getName().c_str());

				setState(STATE_RECOVERY);

			} else if (status == LocalPlannerBase::STATUS_PATH_BLOCKED) {
				// Path goes through obstacle - replan
				// Update global map with local map

				ROS_INFO("Local path is blocked!");
				ROS_INFO(" - Merging local map on global map...");

				globalCostmap_->merge(*localCostmap_, true);

				ROS_INFO(" - Merge done!");
				ROS_INFO(" - Executing replanning...");

				prev_time = ros::Time::now();
				preparePlanAsync(goal_);

				globalCostmapPublisher_.publish(globalCostmap_->getOccupancyGrid());

			} else if (status == LocalPlannerBase::STATUS_UNKNOWN_ERROR) {
				ROS_ERROR_NAMED("NavexNavigation", "Unknown local planner error");
			} else {
				ROS_ERROR_NAMED("NavexNavigation", "Unknown local planner status returned");
			}

			if (isTemporalGoalExists()) {
				if (isTemporalGoalReached()) {
					static ros::Time prev_T_time = ros::Time::now();
					if( (ros::Time::now() - prev_T_time).toSec() > 5 )
					{
						prev_T_time = ros::Time::now();
						prev_time = ros::Time::now();
						ROS_INFO("Temporal goal is reached");
						ROS_INFO(" - Executing replanning...");
						preparePlanAsync(goal_);
					}
				}
			}

			if (isGoalReached()) {
				stopNavigation();
				ROS_INFO("Goal reached!");
			}

			if( dynamicConfig_.replanning_frequency>0 and (ros::Time::now() - prev_time).toSec() > (1./dynamicConfig_.replanning_frequency) )
			{
				prev_time = ros::Time::now();
				ROS_INFO("Plan TIMEOUT");
				ROS_INFO(" - Executing replanning...");
				preparePlanAsync(goal_);
			}


		} else if (!firstPlanning_ && getState() == STATE_PLANNING) {
			/*
			 * Planning state
			 */

			// Continue obstacle avoidance while planning
			localPlanner_->update();
		}
		else if (getState() == STATE_RECOVERY) { // Recovery
			/*
			 * Recovery state
			 */

			// Execute recovery planners
			LocalPlannerBase::PlannerStatus recoveryStatus = recoveryPlanner_->update();

			if (recoveryStatus == LocalPlannerBase::STATUS_RECOVERY_OK) {
				// Ok, continue
			} else if (recoveryStatus == LocalPlannerBase::STATUS_RECOVERY_FAILED) {
				// Recovery failed, try another one

				delete recoveryPlanner_;
				recoveryPlanner_ = NULL; // new AnotherRecovery(...)

				ROS_INFO("Recovery failed to finish");

				/// TODO: switch to another recovery
				prev_time = ros::Time::now();
				preparePlanAsync(goal_, false);

			} else if (recoveryStatus == LocalPlannerBase::STATUS_RECOVERY_FINISHED) {
				// Recovery finished
				delete recoveryPlanner_;
				recoveryPlanner_ = NULL;

				ROS_INFO("Recovery successfully finished!");

				// Replan & Resume navigation
				prev_time = ros::Time::now();
				preparePlanAsync(goal_, false);
			}
		} else if (getState() == STATE_IDLE) {

		} else if (getState() == STATE_PLANNING_FAILED) {
			prev_time = ros::Time::now();
			preparePlanAsync(goal_, false);
		}

		publishStateMarker();

		ros::spinOnce();
		rate.sleep();
	}
}

void NavexNavigation::simpleGoalCallback(
		const geometry_msgs::PoseStamped::Ptr& goal) {
	firstPlanning_ = true;
	stopNavigation();
	preparePlanAsync(*goal);
}

void NavexNavigation::goalCallback(
		const move_base_msgs::MoveBaseActionGoal::Ptr& goal) {
	firstPlanning_ = true;
	stopNavigation();
	preparePlanAsync(goal->goal.target_pose);
}

geometry_msgs::PoseStamped NavexNavigation::getBaseFramePose() const {
	tf::StampedTransform transform;
	geometry_msgs::PoseStamped pose;

	try {
		tfListener_.lookupTransform(globalCostmap_->getFrameId(),
				localCostmap_->getFrameId(), ros::Time(0), transform);
	} catch (tf::TransformException& err) {
		ROS_ERROR("Failed to find robot's tf");

		transform.setIdentity();
	}

	tf::poseTFToMsg(transform, pose.pose);
	pose.header.frame_id = transform.frame_id_;

	return pose;
}

geometry_msgs::PoseStamped NavexNavigation::getGlobalFramePose() const {

	tf::StampedTransform transform;
	geometry_msgs::PoseStamped pose;

	try {
		tfListener_.lookupTransform(globalFrameId(),
				localCostmap_->getFrameId(), ros::Time(0), transform);
	} catch (tf::TransformException& err) {
		ROS_ERROR("Failed to find robot's tf");

		transform.setIdentity();
	}

	tf::poseTFToMsg(transform, pose.pose);
	pose.header.frame_id = transform.frame_id_;

	return pose;
}


void NavexNavigation::preparePlanAsync(const geometry_msgs::PoseStamped& goal,
		bool forceNavigation) {

	boost::mutex::scoped_lock lock(stateMutex_);

	if (navigationState_ == STATE_PLANNING) {
		//ROS_WARN("Can't start planning, already running.");
		//return;
		ROS_INFO("Previous planning aborted.");
		backgroundPlanThread_->interrupt();
	}

	boost::shared_ptr<boost::thread> tmp = backgroundPlanThread_;
	if(tmp and tmp->joinable()){
		lock.unlock();
			tmp->join();
		lock.lock();
	}

	goal_ = goal;
	goal_.header.stamp = ros::Time::now();

	localPlanner_->setForceNavigation(forceNavigation);
	//setState(STATE_PLANNING);
	navigationState_ = STATE_PLANNING;

	ROS_INFO("Planning started...");

	backgroundPlanThread_ =
			boost::shared_ptr<boost::thread>(
					new boost::thread (
							boost::bind(&NavexNavigation::preparePlanThread, this, goal_)));

}

std::string NavexNavigation::globalFrameId()const
{
	return dynamicConfig_.global_frame_id;
}

void NavexNavigation::transform_path_to_golobal_frame(nav_msgs::Path& path)
{
	for(size_t i=0; i<path.poses.size(); i++)
	{
		try {
			tfListener_.waitForTransform(globalFrameId(), path.header.frame_id,
					path.header.stamp, ros::Duration(5.0));
			tfListener_.transformPose(globalFrameId(),
					path.poses[i], path.poses[i]);
			path.poses[i].header.frame_id = globalFrameId();
		}
		catch (tf::TransformException& exception) {
			ROS_ERROR("Failed to transform pose: \n%s [%s,%d]", exception.what(), __FILE__, __LINE__);
			throw;
		}
	}
	path.header.frame_id = globalFrameId();
}

void NavexNavigation::transform_to_goal_frame(geometry_msgs::PoseStamped& pose)
{
	if(goal_.header.frame_id == pose.header.frame_id) return;

	try {
		tfListener_.waitForTransform(goal_.header.frame_id, pose.header.frame_id,
				pose.header.stamp, ros::Duration(5.0));
		tfListener_.transformPose(goal_.header.frame_id,
				pose, pose);
		pose.header.frame_id = goal_.header.frame_id ;
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR("Failed to transform pose: \n%s [%s,%d]", exception.what(), __FILE__, __LINE__);
		throw;
	}
}
void NavexNavigation::transform_to_global_map_frame(geometry_msgs::PoseStamped& pose)
{
	string global_map_frame_id = globalCostmap_->getOccupancyGrid()->header.frame_id;

	if(global_map_frame_id == pose.header.frame_id) return;

	try {
		tfListener_.waitForTransform(global_map_frame_id, pose.header.frame_id,
				pose.header.stamp, ros::Duration(5.0));
		tfListener_.transformPose(global_map_frame_id,
				pose, pose);
		pose.header.frame_id = global_map_frame_id ;
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR("Failed to transform pose: \n%s [%s,%d]", exception.what(), __FILE__, __LINE__);
		throw;
	}
}

void publish_goal(ros::Publisher& pub, const geometry_msgs::PoseStamped& goal )
{
	geometry_msgs::PointStamped current_goal_msg;
	current_goal_msg.header = goal.header;
	current_goal_msg.point.x = goal.pose.position.x;
	current_goal_msg.point.y = goal.pose.position.y;
	current_goal_msg.point.z = goal.pose.position.z;
	pub.publish(current_goal_msg);
}

void NavexNavigation::preparePlanThread(geometry_msgs::PoseStamped goal) {
	ROS_INFO_NAMED("NavexNavigation", "Goal received [WORLD]: x = %f, y = %f",
			goal.pose.position.x, goal.pose.position.y);

	publish_goal(goalPublisher_, goal);

	struct PlanningInterruptionDetected : public std::exception {};
	#define RETURN_IF_INTERRUPTED \
		if( boost::this_thread::interruption_requested() or getState() != STATE_PLANNING ) \
			throw PlanningInterruptionDetected();

	try{

		geometry_msgs::PoseStamped startPose;
		geometry_msgs::PoseStamped goalPose = goal;

		try {

			startPose = getBaseFramePose();
			transform_to_global_map_frame(goalPose);

			ROS_INFO_NAMED("NavexNavigation", "Goal received [ROBOT]: x = %f, y = %f; Pose [ROBOT]: x = %f, y = %f; distance = %f m",
					goalPose.pose.position.x, goalPose.pose.position.y,
					startPose.pose.position.x, startPose.pose.position.y,
					hypot(goalPose.pose.position.x-startPose.pose.position.x, goalPose.pose.position.y-startPose.pose.position.y)
			);


		} catch (tf::TransformException& ex) {
			RETURN_IF_INTERRUPTED
			ROS_ERROR("Failed to get transform of starting pose: \n%s", ex.what());
			setState(STATE_IDLE);
			return;
		}

		RETURN_IF_INTERRUPTED

		nav_msgs::Path path;

		bool path_created = globalPlanner_->makePlan(*globalCostmap_, startPose, goalPose, path);

		if (!path_created) {

			RETURN_IF_INTERRUPTED

			ROS_ERROR("Failed to build a plan");
			setState(STATE_PLANNING_FAILED);
			return;
		}

		ROS_INFO_NAMED("NavexNavigation", "Path created (%lu poses)", path.poses.size());

		transform_path_to_golobal_frame(path);

		geometry_msgs::PoseStamped temporal_goal = path.poses.back();
		transform_to_goal_frame(temporal_goal);
		temporal_goal.header.stamp = goal.header.stamp;

		setTemporalGoal(temporal_goal);
		publish_goal(temporalGoalPublisher_, temporal_goal);


		localPlanner_->setPlan(path);
		pathPublisher_.publish(path);

		localPlanner_->setForceNavigation(false);
		firstPlanning_ = false;

		RETURN_IF_INTERRUPTED

		setState(STATE_NAVIGATION);

	}
	catch(const PlanningInterruptionDetected& exception_planningInterrupted)
	{

	}
	#undef RETURN_IF_INTERRUPTED
}

bool NavexNavigation::isGoalReached() const {
	geometry_msgs::PoseStamped pose = getGlobalFramePose();

	tf::Vector3 currentPose;
	tf::Vector3 goal;

	tf::pointMsgToTF(pose.pose.position, currentPose);
	tf::pointMsgToTF(goal_.pose.position, goal);

	double distance = currentPose.distance(goal);

	std_msgs::Float32 f;
	f.data = distance;

	distanceToGoalPublisher_.publish(f);

	return distance < dynamicConfig_.goal_tolerance;
}

bool NavexNavigation::isTemporalGoalReached() const {
	geometry_msgs::PoseStamped pose = getGlobalFramePose();

	tf::Vector3 currentPose;
	tf::Vector3 goal;

	tf::pointMsgToTF(pose.pose.position, currentPose);
	tf::pointMsgToTF(temporal_goal_.pose.position, goal);

	double map_w = globalCostmap_->getWidth()*globalCostmap_->getOccupancyGrid()->info.resolution;
	return currentPose.distance(goal) < map_w*0.5*0.5;
}

bool NavexNavigation::isTemporalGoalExists() const {
	geometry_msgs::PoseStamped pose = getGlobalFramePose();

	tf::Vector3 tgoal;
	tf::Vector3 goal;

	tf::pointMsgToTF(goal_.pose.position, goal);
	tf::pointMsgToTF(temporal_goal_.pose.position, tgoal);

	return goal.distance(tgoal) > dynamicConfig_.goal_tolerance;
}


bool NavexNavigation::makePlanCallback(nav_msgs::GetPlan::Request& request,
		nav_msgs::GetPlan::Response& response) {
	return globalPlanner_->makePlan(*globalCostmap_, request.start,
			request.goal, response.plan);
}

void NavexNavigation::publishLocalCostmapCallback(ros::TimerEvent timer) const {
	localCostmapPublisher_.publish(localCostmap_->getOccupancyGrid());
}

void NavexNavigation::publishGlobalCostmapCallback(ros::TimerEvent timer) const {
	globalCostmapPublisher_.publish(globalCostmap_->getOccupancyGrid());
}

bool NavexNavigation::versionServiceCallback(navex_msgs::VersionService::Request & req,
		navex_msgs::VersionService::Response & resp)
{
	resp.version = NAVEX_ROBIL_VERSION;
	return true;
}

void NavexNavigation::stopNavigation() {
	setState(NavexNavigation::STATE_IDLE);
	robotController_->stop();
}

void NavexNavigation::stopCallback(const actionlib_msgs::GoalID::Ptr& stop) {
	stopNavigation();
}

void NavexNavigation::publishStateMarker() const {
	visualization_msgs::Marker marker;

	marker.id = 1;

	marker.header.frame_id = localCostmap_->getFrameId();
	marker.header.stamp = ros::Time::now();

	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = 0;

	marker.scale.z = 0.1;
	marker.pose.position.x = -0.2;

	marker.color.a = 1.0f;

	marker.pose.orientation.w = 1;
	marker.lifetime = ros::Duration(1.0 / 20.0);
	marker.frame_locked = true;

	NavigationState state = getState();

	switch (state) {
		case NavexNavigation::STATE_IDLE:
			marker.text = "Idle";
			break;
		case NavexNavigation::STATE_NAVIGATION:
			marker.text = "Navigating";
			break;
		case NavexNavigation::STATE_PLANNING:
			marker.text = "Planning";
			break;
		case NavexNavigation::STATE_RECOVERY:
			marker.text = "Recovery";
			break;
		case NavexNavigation::STATE_PLANNING_FAILED:
			marker.text = "Planning failed";
			break;
	}

	ros::param::set("/NAVIGATION_STATE", marker.text);

	statePublisher_.publish(marker);
}

void NavexNavigation::dynamicConfigCallback(
		navex_navigation::NavigationParametersConfig& config, uint32_t level) {
	dynamicConfig_ = config;

	dynamic_cast<SkidSteerController*>(robotController_)->setLinearScale(config.scale_linear);
	dynamic_cast<SkidSteerController*>(robotController_)->setAngularScale(config.scale_angular);
	dynamic_cast<FuzzyLocalPlanner*>(localPlanner_)->setNavigationGlobalFrameID(config.global_frame_id);
}
