/*
 * Filename: GoalTrajectoryMatcher.cpp
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


#include <navex/trajectory/matcher/GoalTrajectoryMatcher.h>


GoalTrajectoryMatcher::GoalTrajectoryMatcher()
	: goal_(0, 0), lastPoseDistance_(false) {
}

GoalTrajectoryMatcher::~GoalTrajectoryMatcher() {
}

double GoalTrajectoryMatcher::Pg(const tf::Vector3 & goal,
		const tf::Vector3 & robot,
		const tf::Vector3 & pose)const
{
	static const int N = 2;
	double goalPoseDistance = goal.distance(pose);
	double goalRobotDistance = goal.distance(robot);
	double pg = 1.0 - fmin(goalPoseDistance/goalRobotDistance,0.9);

	return pow(pg,N);
//	return pg;
}

double GoalTrajectoryMatcher::Po(const tf::Vector3 & goal,
		const tf::Vector3 & robot,
		const tf::Vector3 & pose)const
{
	static const int N = 2;
	double goalPoseDistance = goal.distance(pose);
	double goalRobotDistance = goal.distance(robot);

	//ROS_INFO_STREAM("DISTANCES = " << goalPoseDistance << "," << goalRobotDistance << "," << goalPoseDistance/goalRobotDistance);

	double po = fmin(goalPoseDistance/goalRobotDistance,1.0);

	return pow(po,N);
	//return po;
}

double GoalTrajectoryMatcher::Pf(const CostMap & localCostMap,
		const geometry_msgs::PoseStamped & pose)const
{
	CostMapCell::CellType cell = localCostMap.getCellValue(pose);
	if(cell == CostMapCell::CELL_BLOCKED)
		return 0.0;
	if(cell == CostMapCell::CELL_MAYBE_BLOCKED)
		return 0.5;
	if(cell == CostMapCell::CELL_UNKNOWN)
		return 1.0;
	return 1.0;
}

TrajectoryMatch::Ptr GoalTrajectoryMatcher::match(const CostMap& localCostMap,
		const CostMap& globalCostMap,
		const Trajectory::Ptr& trajectory,
		const ITrajectoryMatcher::MatchStrategy strategy ) const
{
	const nav_msgs::Path::Ptr& path = trajectory->getPath();

	tf::Vector3 goalVector;
	tf::Vector3 currentPose;
	tf::Vector3 secondToLastPose;
	tf::Vector3 robotPose;

	tf::pointMsgToTF(localCostMap.pixelToPose(goal_).pose.position, goalVector);
	tf::pointMsgToTF(path->poses[path->poses.size()-2].pose.position, currentPose);

	//double poScore = 1.0;
	double pfScore = 1.0;
	double pgScore = 0.0;
	int freeCounter=0,inflationCounter=0,blockedCounter=0,unknownCounter=0;


	for (int i = 0; i < path->poses.size(); ++i)
	{
		geometry_msgs::PoseStamped pose = path->poses[i];
		pose.header.stamp = ros::Time(0); // To get the latest tf available in the buffer
		pose.header.frame_id = localCostMap.getFrameId(); /// TODO frame id must be set by trajectory it self

		tf::pointMsgToTF(pose.pose.position, currentPose);

		CostMapCell::CellType cell = localCostMap.getCellValue(pose);
		if(cell == CostMapCell::CELL_BLOCKED)
			blockedCounter++;
		else if(cell == CostMapCell::CELL_MAYBE_BLOCKED)
			inflationCounter++;
		else if(cell == CostMapCell::CELL_FREE)
			freeCounter++;
		else
			unknownCounter++;
		//poScore *= Po(goalVector, robotPose, currentPose);
//		pfScore *= Pf(localCostMap, pose);
//		if(Pf(localCostMap, pose) < 1.0)
//		{
//			CostMapCell::CellType cell = localCostMap.getCellValue(pose);
//			ROS_INFO_STREAM("Point=" << i << "PF=" << Pf(localCostMap, pose) << "," << "PG=" << Pg(goalVector, robotPose, currentPose) << "CELL=" << (int)cell);
//		}
	}

//	double d = currentPose.distance(secondToLastPose);

//	static double w1 = 0.4;
//	static double w2 = 0.5;
//	static double w3 = 1.0;
//	static double w4 = 10000000.0;
//	static int blp = 100;

//	static double overallw = w1+w2+w3+w4;

	double overallpts = path->poses.size();
	pgScore = Pg(goalVector, robotPose, currentPose);
//	pfScore = pow(freeCounter,w1)*
//			  pow(unknownCounter,w2)*
//			  pow((overallpts-inflationCounter),w3)*
//			  pow((overallpts-blockedCounter),w4)/
//			  pow(overallpts,w1+w2+w3+w4);

//	pfScore = w1 * (freeCounter/overallpts)+
//			  w2 * (unknownCounter/overallpts) +
//			  w3 * (1-inflationCounter/overallpts) +
//			  w4 * pow(1-blockedCounter/overallpts,blp);

	double f1 = trajectory->getF1();
	double f2 = trajectory->getF2();
	pfScore = ((f2*overallpts + f1*(freeCounter + unknownCounter - inflationCounter - blockedCounter*overallpts*f2))/
			  (2.0*overallpts*f2));

//	pfScore /= overallpts;
	pfScore = fmax(fmin(pfScore,1.0),0.0);

	double score = pgScore * pfScore;

	TrajectoryMatch::Ptr match(new TrajectoryMatch(trajectory, score));

//	ROS_INFO_STREAM("PG Score = " << pgScore << "," << " PF Score = " << pfScore << " Total = " << score);
	if(score < 0.00001)
	{
		//ROS_INFO_STREAM("PG Score = " << pgScore << "," << "PF Score = " << pfScore << "Total = " << score);
		match->setFatal(true);
	}
	if(pfScore <= 0.5)
		match->setBlocked(true);

	return match;
}

TrajectoryMatch::Ptr GoalTrajectoryMatcher::matchOld(const CostMap& localCostMap,
		const CostMap& globalCostMap,
		const Trajectory::Ptr& trajectory) const {

	const nav_msgs::Path::Ptr& path = trajectory->getPath();

	double closestDistanceToGoal = numeric_limits<double>::max();

	tf::Vector3 goalVector;
	tf::Vector3 currentPose;

	tf::pointMsgToTF(localCostMap.pixelToPose(goal_).pose.position, goalVector);

	bool isFatal = false;
	bool isBlocked = false;

	for (int i = 0; i < path->poses.size(); ++i) {
		geometry_msgs::PoseStamped pose = path->poses[i];
		pose.header.stamp = ros::Time(0); // To get the latest tf available in the buffer
		pose.header.frame_id = localCostMap.getFrameId(); /// TODO frame id must be set by trajectory it self

		tf::pointMsgToTF(pose.pose.position, currentPose);

		int pointValue = fmax(localCostMap.getCellValue(pose), globalCostMap.getCellValue(pose));

		if (!lastPoseDistance_) {
			// Find closest pose to goal on the whole path
			double distance = goalVector.distance(currentPose);

			if (distance < closestDistanceToGoal) {
				closestDistanceToGoal = distance;
			}
		} else if (lastPoseDistance_ && i == path->poses.size() - 1) {
			// Calculate distance only of the last pose on path
			closestDistanceToGoal = goalVector.distance(currentPose);
		}

		if (pointValue >= CostMapCell::CELL_MAYBE_BLOCKED &&
				pointValue < CostMapCell::CELL_BLOCKED)
			isBlocked = true;

		if (pointValue >= CostMapCell::CELL_BLOCKED) {
			isFatal = true;
			isBlocked = true;

			break;
		}
	}

	double score = invsigmoid(closestDistanceToGoal) * trajectory->getWeight();

	if (isFatal) {
		score -= 1.0; // scale to range [-1, 0]
	} else 	if (isBlocked) {
		score *= 0.25; // decrease score to path that may be blocked
	}

	if (isBlocked && trajectory->isInPlace()) {
		// In place trajectories considered not fatal
		// score += 1.0;

		// A small value so that in place trajectories will win
		// only if the other trajectories are blocked
		score = 0.0005;
	}

	TrajectoryMatch::Ptr match(new TrajectoryMatch(trajectory, score));

	match->setFatal(isFatal);
	match->setBlocked(isBlocked);

	return match;
}
