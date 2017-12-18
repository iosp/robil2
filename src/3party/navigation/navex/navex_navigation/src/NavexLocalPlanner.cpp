/*
 * Filename: NavexLocalPlanner.cpp
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


#include <navex_navigation/NavexLocalPlanner.h>


NavexLocalPlanner::NavexLocalPlanner(CostMap* localCostmap,
		CostMap* globalCostmap,
		IController* robotController, double simulationTime,
		double granularity)
	: LocalPlannerBase(localCostmap, globalCostmap, robotController),
	  trajectorySimulator_(simulationTime, granularity),
	  localGoalFinder_(localCostmap->getFrameId()), configServer_(ros::NodeHandle("~/local_planner")) {

	configServer_.setCallback(
			boost::bind(&NavexLocalPlanner::dynamicConfigCallback, this, _1, _2));

	generateTrajectories();

	trajectoryMathcer_ = new FuzzyTrajectoryMatcher(&config_);

	ros::NodeHandle node;

	localGoalPublisher_ = node.advertise<geometry_msgs::PointStamped>(
			"local_goal", 10, true);

	localPlanPublisher_ = node.advertise<nav_msgs::Path>(
			"local_plan", 10, true);

	trajectoriesPublisher_ = node.advertise<visualization_msgs::MarkerArray>(
			"trajectories", 5, true);

}

NavexLocalPlanner::~NavexLocalPlanner() {
	delete trajectoryMathcer_;
}

TrajectoryMatch::SetPtr NavexLocalPlanner::evaluateTrajectories( const ITrajectoryMatcher::MatchStrategy strategy ) {
	TrajectoryMatch::SetPtr matches = trajectoryMathcer_->ITrajectoryMatcher::match(
			*getLocalCostmap(), *getGlobalCostmap(), trajectories_, strategy);

	return matches;
}

bool NavexLocalPlanner::updateLocalGoal() {
	const nav_msgs::Path& path = getPlan();
	const CostMap* costmap = getLocalCostmap();

	size_t localGoalIndexOnPath = localGoalFinder_.findLocalGoal();

	geometry_msgs::PoseStamped localGoalPose = path.poses[localGoalIndexOnPath];
	localGoalPose.header.stamp = ros::Time::now();

	cv::Point localGoal = costmap->poseToPixel(localGoalPose);

	trajectoryMathcer_->setGoal(localGoal);

	publishLocalGoal(localGoalPose);

	// Check if the goal is valid
	CostMapCell::CellType cell = costmap->getCellValue(localGoalPose);
	if (cell >= CostMapCell::CELL_BLOCKED) {
		// Goal is blocked
		// Replanning needed
		return false;
	}

	return true;
}

LocalPlannerBase::PlannerStatus NavexLocalPlanner::update() {
	bool validLocalGoal = updateLocalGoal();

	if (!isForceNavigation() && !validLocalGoal) {
		// Local goal (= part of the global plan) falls on a blocked
		// cells, recovery should me executed
		// note: if force navigation activated (can occur while planning is active)
		// the goal will be valid
		return LocalPlannerBase::STATUS_PATH_BLOCKED;
	}

	TrajectoryMatch::SetPtr matches = evaluateTrajectories(ITrajectoryMatcher::MS_NORMAL);
	TrajectoryMatch::Ptr bestMatch = *matches->begin();

	trajectoryMathcer_->setLastBestTrajectory(bestMatch->getTrajectory());

	publishTrajectories(matches);

	if (!isForceNavigation() && bestMatch->getTrajectory()->isInPlace()) {
		// In place best trajectory means, the other are blocked
		// This means we are in a potentially dangerous position
		// so better call a recovery behavior

		// NOTE: inPlace turn can't be fatal, because
		// GoalTrajectoryMatcher gives a fatal, in place turns a small positive score

		bool dangerousPosition = true;

		BOOST_FOREACH(const TrajectoryMatch::Ptr& match, *matches) {
			if (!match->getTrajectory()->isInPlace() && !match->isFatal())
				dangerousPosition = false;
		}

		if (dangerousPosition)
			return LocalPlannerBase::STATUS_NO_PLAN_FOUND;
	}


//	// If the local goal is behind the back, force
//	// in place turn
//	if (!isLocalGoalInFront()) {
//
//		// Choose best in place turn
//		TrajectoryMatch::SetPtr inPlaceMatches(new TrajectoryMatch::Set());
//
//		BOOST_FOREACH(const TrajectoryMatch::Ptr& match, *matches) {
//
//			if (match->getTrajectory()->isInPlace())
//				inPlaceMatches->insert(match);
//		}
//
//		bestMatch = *inPlaceMatches->begin();
//	}

	driveCommand(bestMatch->getTrajectory()->getMotionModel());


	localPlanPublisher_.publish(bestMatch->getTrajectory()->getPath(
			true, getLocalCostmap()->getFrameId()));

	return LocalPlannerBase::STATUS_OK;
}

bool NavexLocalPlanner::isLocalGoalInFront() const {
	const nav_msgs::Path& path = this->getPlan();
	size_t localGoalIndexOnPath = localGoalFinder_.getLastLocalGoal();
	geometry_msgs::PoseStamped localGoalPose = path.poses[localGoalIndexOnPath];

	localGoalPose.header.stamp = ros::Time(0);

	cv::Point goalPixel = getLocalCostmap()->poseToPixel(localGoalPose);

	localGoalPose.header.frame_id = getLocalCostmap()->getFrameId();
	localGoalPose.pose.position.x = 0;
	localGoalPose.pose.position.y = 0;

	cv::Point robotPixel = getLocalCostmap()->poseToPixel(localGoalPose);

	goalPixel = goalPixel - robotPixel;

	double angle = atan2(goalPixel.y, goalPixel.x);

//	ROS_INFO("Goal (%i, %i), angle = %f", goalPixel.x, goalPixel.y, angle);

	if (goalPixel.x > 0 && fabs(angle) < ((90.0 / 180.0) * M_PI))
		return true;

	return false;
}

void createTrajectory(double angle, double speed, double maxSpeed, Trajectory::VectorPtr& trajectories , TrajectorySimulator& trajectorySimulator_)
{
	Trajectory::Ptr trajectory = trajectorySimulator_.simulate(
			new SkidSteerModel(speed, angle));
	trajectories->push_back(trajectory);
}

void NavexLocalPlanner::generateTrajectories()
{
	double minSpeed = config_.min_speed;
	double maxSpeed = config_.max_speed;
	double minAngle = config_.min_angular_speed;
	double maxAngle = config_.max_angular_speed;
	double speedStep = config_.speed_step;

	trajectorySimulator_ = TrajectorySimulator(config_.simulation_time, config_.granularity);
	Trajectory::VectorPtr trajectories(new Trajectory::Vector());

	for (double speed = minSpeed; speed <= maxSpeed; speed += speedStep)
	{
		const double minPerc = 0.25;
		double speedPercent = ( ((maxSpeed-speed)/(maxSpeed-minSpeed))*(1-minPerc)+minPerc );
		double speedMinAngle = minAngle * speedPercent;
		double speedMaxAngle = maxAngle * speedPercent;

		double angleStep = config_.angular_speed_step;
		double f = angleStep;
		angleStep+=f;
		double lastMaxAngle(0), lastMinAngle(0);

//		ROS_INFO_STREAM("Speed Min Angle = " << speedMinAngle << "," << "Speed Max Angle = " << speedMaxAngle);
//		ROS_INFO_STREAM("F = " << f << "," << "Angle Step = " << angleStep);
		for (double angle = f; angle <= fmax(fabs(speedMaxAngle),fabs(speedMinAngle));angle+=angleStep,angleStep+=f)
		{
//			ROS_INFO_STREAM("            Speed = " << speed << ", Angle = " << angle);
			if( angle <= speedMaxAngle)
			{
				lastMaxAngle = angle;
				createTrajectory(angle, speed, maxSpeed, trajectories, trajectorySimulator_);
			}
			if( speedMinAngle <= (-angle) )
			{
				lastMinAngle = -angle;
				createTrajectory(-angle, speed, maxSpeed, trajectories, trajectorySimulator_);
			}
		}

		{
			createTrajectory(0, speed, maxSpeed, trajectories, trajectorySimulator_);
		}
		if(lastMaxAngle < (speedMaxAngle-f*0.5))
		{
			createTrajectory(speedMaxAngle, speed, maxSpeed, trajectories, trajectorySimulator_);
		}
		if(lastMinAngle > (speedMinAngle+f*0.5))
		{
			createTrajectory(speedMinAngle, speed, maxSpeed, trajectories, trajectorySimulator_);
		}
	}

	trajectories_  = trajectories;
}

void NavexLocalPlanner::generateTrajectoriesOld() {

	const double minSpeed = config_.min_speed;
	const double maxSpeed = config_.max_speed;
	const double minAngle = config_.min_angular_speed;
	const double maxAngle = config_.max_angular_speed;
	const double speedStep = config_.speed_step;
	const double angleStep = config_.angular_speed_step;

	trajectorySimulator_ = TrajectorySimulator(config_.simulation_time, config_.granularity);
	Trajectory::VectorPtr trajectories(new Trajectory::Vector());

	for (double angle = minAngle; angle <= maxAngle; angle += angleStep) {
		for (double speed = minSpeed; speed <= maxSpeed; speed += speedStep) {

			Trajectory::Ptr trajectory = trajectorySimulator_.simulate(
					new SkidSteerModel(speed, angle));

			// http://www.wolframalpha.com/input/?i=1-0.001*abs%28x%29
			trajectory->setWeight(1 - 0.001 * fabs(angle));

			trajectories->push_back(trajectory);
		}
	}

	trajectorySimulator_ = TrajectorySimulator((2 * M_PI - M_PI / 16.0) / config_.inplace_angular_speed, config_.granularity);

	Trajectory::Ptr rightTurn = trajectorySimulator_.simulate(
			new SkidSteerModel(0.005, fabs(config_.inplace_angular_speed)));

	rightTurn->overrideInPlace(true, true);

	rightTurn->setWeight(0.001);

	Trajectory::Ptr leftTurn = trajectorySimulator_.simulate(
			new SkidSteerModel(0.005, -fabs(config_.inplace_angular_speed)));

	leftTurn->overrideInPlace(true, true);

	leftTurn->setWeight(0.001);

	trajectories->push_back(rightTurn);
	trajectories->push_back(leftTurn);

	trajectories_  = trajectories;
}

void NavexLocalPlanner::publishLocalGoal(
		const geometry_msgs::PoseStamped pose) {
	geometry_msgs::PointStamped point;

	point.header.frame_id = pose.header.frame_id;
	point.header.stamp = ros::Time::now();

	point.point = pose.pose.position;

	localGoalPublisher_.publish(point);
}

void NavexLocalPlanner::publishTrajectories(const TrajectoryMatch::SetPtr& matches) {

	visualization_msgs::MarkerArray markers;

	int id = 1;

	bool scoreColorize = config_.trajectory_score_color;

	bool first = true;

	BOOST_FOREACH(const TrajectoryMatch::Ptr& match, *matches) {
		visualization_msgs::Marker marker;

		marker.id = id++;

		marker.header.frame_id = getLocalCostmap()->getFrameId();
		marker.header.stamp = ros::Time::now();

		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = 0;

		marker.scale.x = first ? 0.03 : 0.005f;

		if (first) {
			marker.color.g = 1.0;
			first = false;
		} else {

			if (scoreColorize) {
				// Color intensity reflects the score

				double colorNormalized = ((match->getScore()) + 1.0) / 2.0;

				marker.color.r = 1 - colorNormalized * 0.5;
				marker.color.g = colorNormalized * 0.5;

			} else {

				// Color reflect the danger

				if (match->isFatal()) {
					marker.color.r = 1.0;
				} else if (match->isBlocked()) {
					marker.color.r = 1.0;
					marker.color.g = 0.8;
				} else {
					marker.color.b = 1.0;
				}
			}

		}

		marker.color.a = 1.0f;

		const nav_msgs::Path::Ptr& path = match->getTrajectory()->getPath();

		BOOST_FOREACH(const geometry_msgs::PoseStamped& pose, path->poses) {
			marker.points.push_back(pose.pose.position);
		}

		marker.pose.orientation.w = 1;
		marker.lifetime = ros::Duration(1.0 / 20.0);
		marker.frame_locked = true;

		markers.markers.push_back(marker);
	}

	trajectoriesPublisher_.publish(markers);
}

void NavexLocalPlanner::dynamicConfigCallback(
		navex_navigation::NavexLocalPlannerConfig& config, uint32_t level) {
	config_ = config;

	localGoalFinder_.setGoalDistance(config.local_goal_distance);

	generateTrajectories();
}
