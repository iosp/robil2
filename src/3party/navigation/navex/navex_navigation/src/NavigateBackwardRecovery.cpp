/*
 * Filename: NavigateBackwardRecovery.cpp
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


#include <navex_navigation/NavigateBackwardRecovery.h>


NavigateBackwardRecovery::NavigateBackwardRecovery(
		CostMap* localCostmap, CostMap* globalCostmap,
		IController* robotController,
		double maxDurationSec)
	:
	  RecoveryPlannerBase(localCostmap, globalCostmap, robotController),
	  localGoalFinder_(localCostmap->getFrameId()),
	  durationSec_(maxDurationSec) {

	generateTrajectories();

	ros::NodeHandle node;

	localGoalPublisher_ = node.advertise<geometry_msgs::PointStamped>(
			"local_goal", 10, true);

	localPlanPublisher_ = node.advertise<nav_msgs::Path>(
			"local_plan", 10, true);

	startTime_ = boost::posix_time::microsec_clock::local_time();

	trajectoriesPublisher_ = node.advertise<visualization_msgs::MarkerArray>(
			"trajectories", 100, true);
}

NavigateBackwardRecovery::~NavigateBackwardRecovery() {

}

TrajectoryMatch::SetPtr NavigateBackwardRecovery::evaluateTrajectories( const ITrajectoryMatcher::MatchStrategy strategy ) {
	TrajectoryMatch::SetPtr matches =
			goalTrajectoryMatcher_.match(
					*getLocalCostmap(), *getGlobalCostmap(), trajectories_, strategy);

	return matches;
}

LocalPlannerBase::PlannerStatus NavigateBackwardRecovery::update() {
	bool validLocalGoal = updateLocalGoal();

	TrajectoryMatch::SetPtr matches = evaluateTrajectories(ITrajectoryMatcher::MS_NORMAL);
	TrajectoryMatch::Ptr bestMatch = *matches->begin();

	publishTrajectories(matches);

	driveCommand(bestMatch->getTrajectory()->getMotionModel());

	localPlanPublisher_.publish(bestMatch->getTrajectory()->getPath(
			true, getLocalCostmap()->getFrameId()));

//	boost::posix_time::ptime timeNow =
//			boost::posix_time::microsec_clock::local_time();
//
//	if ((timeNow - startTime_).total_seconds() >= durationSec_ ||
//			bestMatch->getTrajectory()->isForward()) {
//		return LocalPlannerBase::STATUS_RECOVERY_FINISHED;
//	}

	if (isLocalGoalInFront() /* && isFrontClear(matches) */) {

		return LocalPlannerBase::STATUS_RECOVERY_FINISHED;
	}

	return LocalPlannerBase::STATUS_RECOVERY_OK;
}

bool NavigateBackwardRecovery::isFrontClear(TrajectoryMatch::SetPtr matches) const {
	BOOST_FOREACH(const TrajectoryMatch::Ptr& match, *matches) {
		if (!match->isFatal() && match->getTrajectory()->isForward())
			return true;
	}

	return false;
}

bool NavigateBackwardRecovery::isLocalGoalInFront() const {
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

	if (goalPixel.x > 0 && fabs(angle) < ((65.0 / 180.0) * M_PI))
		return true;

	return false;
}

bool NavigateBackwardRecovery::updateLocalGoal() {
	const nav_msgs::Path& path = this->getPlan();
	const CostMap* costmap = getLocalCostmap();

	size_t localGoalIndexOnPath = localGoalFinder_.findLocalGoal();

	geometry_msgs::PoseStamped localGoalPose = path.poses[localGoalIndexOnPath];
	localGoalPose.header.stamp = ros::Time(0);

	cv::Point localGoal = costmap->poseToPixel(localGoalPose);

	goalTrajectoryMatcher_.setGoal(localGoal);

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

void NavigateBackwardRecovery::generateTrajectories() {

	trajectories_ = Trajectory::VectorPtr(new Trajectory::Vector());

	TrajectorySimulator trajectorySimulator(2.0, 0.1);

//	/*
//	 * Forward
//	 */
//	for (double angle = -1.0; angle <= 1.0; angle += 0.25) {
//		for (double speed = 0.3; speed <= 0.5; speed += 0.2) {
//
//			// Ignore zero speed
//			if (speed == 0.0 && angle == 0.0)
//				continue;
//
//			Trajectory::Ptr trajectory = trajectorySimulator.simulate(
//					new SkidSteerModel(speed, angle));
//
//			// http://www.wolframalpha.com/input/?i=1-0.001*abs%28x%29
//			trajectory->setWeight(1 - 0.001 * fabs(angle));
//
//			trajectories_->push_back(trajectory);
//		}
//	}

	/*
	 * Back
	 */
	for (double angle = -1.0; angle <= 1.0; angle += 0.2) {
		for (double speed = -0.4; speed <= -0.3; speed += 0.1) {

			// Ignore zero speed
			if (speed == 0.0 && angle == 0.0)
				continue;

			Trajectory::Ptr trajectory = trajectorySimulator.simulate(
					new SkidSteerModel(speed, angle));

			// http://www.wolframalpha.com/input/?i=1-0.001*abs%28x%29
			trajectory->setWeight(1 - 0.001 * fabs(angle));

			trajectories_->push_back(trajectory);
		}
	}


	trajectorySimulator = TrajectorySimulator(2.0, 0.1);

	/*
	 * Back and forth
	 */
	for (double backAngle = -1.0; backAngle <= 1.0; backAngle += 0.25) {
		for (double backSpeed = -0.4; backSpeed <= -0.3; backSpeed += 0.1) {


			for (double forthAngle = -1.0; forthAngle <= 1.0; forthAngle += 0.25) {
				for (double forthSpeed = 0.4; forthSpeed <= 0.5; forthSpeed += 0.1) {

					Trajectory::Ptr backTrajectory = trajectorySimulator.simulate(
							new SkidSteerModel(backSpeed, backAngle));

					Trajectory::Ptr frontTrajectory = trajectorySimulator.simulate(
							new SkidSteerModel(forthSpeed, forthAngle));

					backTrajectory->setWeight(1 - 0.001 * fabs(backAngle));

					backTrajectory->concatenate(*frontTrajectory);

					trajectories_->push_back(backTrajectory);
				}
			}

		}
	}

	/*
	 * In place turns
	 */
	trajectorySimulator = TrajectorySimulator((2 * M_PI - M_PI / 16.0) / 1.0, 0.1);

	Trajectory::Ptr rightTurn = trajectorySimulator.simulate(
			new SkidSteerModel(0.005, fabs(1.0)));

	rightTurn->overrideInPlace(true, true);

	rightTurn->setWeight(0.001);

	Trajectory::Ptr leftTurn = trajectorySimulator.simulate(
			new SkidSteerModel(0.005, -fabs(1.0)));

	leftTurn->overrideInPlace(true, true);

	leftTurn->setWeight(0.001);

	trajectories_->push_back(rightTurn);
	trajectories_->push_back(leftTurn);

}

void NavigateBackwardRecovery::publishLocalGoal(
		const geometry_msgs::PoseStamped pose) {
	geometry_msgs::PointStamped point;

	point.header.frame_id = pose.header.frame_id;
	point.header.stamp = ros::Time::now();

	point.point = pose.pose.position;

	localGoalPublisher_.publish(point);
}

void NavigateBackwardRecovery::publishTrajectories(const TrajectoryMatch::SetPtr& matches) {

	visualization_msgs::MarkerArray markers;

	int id = 1;

	bool scoreColorize = true;

	bool first = true;

	BOOST_FOREACH(const TrajectoryMatch::Ptr& match, *matches) {
		visualization_msgs::Marker marker;

		marker.id = id++;

		marker.header.frame_id = getLocalCostmap()->getFrameId();
		marker.header.stamp = ros::Time::now();

		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = 0;

		marker.scale.x = first ? 0.02 : 0.005f;

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
