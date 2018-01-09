/*
 * FuzzyLocalPlanner.cpp
 *
 *  Created on: Jan 15, 2017
 *      Author: assaf
 */

#include <navex_navigation/FuzzyLocalPlanner.h>

FuzzyLocalPlanner::FuzzyLocalPlanner(CostMap* localCostmap,
		CostMap* globalCostmap,
		IController* robotController,
		const std::string & global_navigation_frame_id,
		double simulationTime,
		double granularity)
	: LocalPlannerBase(localCostmap, globalCostmap, robotController),
	  trajectorySimulator_(simulationTime, granularity),
	  localGoalFinder_(localCostmap->getFrameId()), configServer_(ros::NodeHandle("~/local_planner")),
	  global_navigation_frame_id_(global_navigation_frame_id)
{

	configServer_.setCallback(
			boost::bind(&FuzzyLocalPlanner::dynamicConfigCallback, this, _1, _2));

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

FuzzyLocalPlanner::~FuzzyLocalPlanner()
{
	delete trajectoryMathcer_;
}

TrajectoryMatch::SetPtr FuzzyLocalPlanner::evaluateTrajectories( ITrajectoryMatcher::MatchStrategy strategy )
{
	TrajectoryMatch::SetPtr matches = trajectoryMathcer_->ITrajectoryMatcher::match(
			*getLocalCostmap(), *getGlobalCostmap(), trajectories_, strategy);

	return matches;
}

bool FuzzyLocalPlanner::updateLocalGoal()
{
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
	if (cell >= CostMapCell::CELL_BLOCKED)
	{
		// Goal is blocked
		// Replanning needed
		return false;
	}

	return true;
}

LocalPlannerBase::PlannerStatus FuzzyLocalPlanner::update()
{
	bool validLocalGoal = updateLocalGoal();

	if (!isForceNavigation() && !validLocalGoal) {
		// Local goal (= part of the global plan) falls on a blocked
		// cells, recovery should me executed
		// note: if force navigation activated (can occur while planning is active)
		// the goal will be valid
		return LocalPlannerBase::STATUS_PATH_BLOCKED;
	}

	const size_t strategies_numbers=2;
	ITrajectoryMatcher::MatchStrategy strategies[] = {
			ITrajectoryMatcher::MS_NORMAL,
			ITrajectoryMatcher::MS_RECOVERY
	};

	TrajectoryMatch::SetPtr matches;
	TrajectoryMatch::Ptr bestMatch;
	for(size_t i=0;i<strategies_numbers;i++)
	{
		matches = evaluateTrajectories(strategies[i]);
		bestMatch = *matches->begin();

		if(bestMatch->isFatal()) continue; else break;
	}

	if(not bestMatch)
	{
		std::cout<<"NAV: LocalPlannerBase::STATUS_NO_PLAN_FOUND ["<<__FILE__<<":"<<__LINE__<<"]"<<std::endl;
		return LocalPlannerBase::STATUS_NO_PLAN_FOUND;
	}

	trajectoryMathcer_->setLastBestTrajectory(bestMatch->getTrajectory());

	publishTrajectories(matches);

	if (!isForceNavigation() && bestMatch->getTrajectory()->isInPlace()) {
		std::cout<<"NAV: isInPlace"<<std::endl;
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
		{
			std::cout<<"NAV: LocalPlannerBase::STATUS_NO_PLAN_FOUND ["<<__FILE__<<":"<<__LINE__<<"]"<<std::endl;
			return LocalPlannerBase::STATUS_NO_PLAN_FOUND;
		}
	}

	driveCommand(bestMatch->getTrajectory()->getMotionModel());


	localPlanPublisher_.publish(bestMatch->getTrajectory()->getPath(
			true, getLocalCostmap()->getFrameId()));

	return LocalPlannerBase::STATUS_OK;
}

void FuzzyLocalPlanner::createTrajectory(double angle, double speed, Trajectory::VectorPtr& trajectories)
{
	Trajectory::Ptr trajectory = trajectorySimulator_.simulate(
			new SkidSteerModel(speed, angle));
	trajectories->push_back(trajectory);
}

inline double __f(double x, double mp){ return (mp-1)*x+1; }

void FuzzyLocalPlanner::generateTrajectories_angle_iterations(
		Trajectory::VectorPtr trajectories, double speed, double skip_angles_range, bool dynamic_step,
		double _minSpeed, double _maxSpeed)
{
	double minSpeed = fmin( fabs(_minSpeed), fabs(_maxSpeed) );
	double maxSpeed = fmax( fabs(_minSpeed), fabs(_maxSpeed) );

	double minAngle = config_.min_angular_speed;
	double maxAngle = config_.max_angular_speed;
	double speedStep = config_.speed_step;
	double angular_speed_step = config_.angular_speed_step;

	const double minPerc = 0.75; // greater value of the variable leads to greater angle for faster speed.
	double speed_rate =  maxSpeed==minSpeed? 0 : 1 - (maxSpeed-fabs(speed))/(maxSpeed-minSpeed);
	double speedPercent = __f(speed_rate, minPerc);
	double speedMinAngle = minAngle * speedPercent;
	double speedMaxAngle = maxAngle * speedPercent;

	double angleStep = angular_speed_step;
	double f = angleStep;
	angleStep+=f;
	double lastMaxAngle(0), lastMinAngle(0);

	for (double angle = f; angle <= fmax(fabs(speedMaxAngle),fabs(speedMinAngle));angle+=angleStep,angleStep+=(dynamic_step?f:0))
	{
		if( angle < skip_angles_range )continue;

		if( angle <= speedMaxAngle)
		{
			lastMaxAngle = angle;
			createTrajectory(angle, speed, trajectories);
		}

		if( speedMinAngle <= (-angle) )
		{
			lastMinAngle = -angle;
			createTrajectory(-angle, speed, trajectories);
		}
	}

	if(skip_angles_range<0.00001) createTrajectory(0, speed, trajectories);
	if(lastMaxAngle < (speedMaxAngle-f*0.5))
		createTrajectory(speedMaxAngle, speed, trajectories);
	if(lastMinAngle > (speedMinAngle+f*0.5))
		createTrajectory(speedMinAngle, speed, trajectories);
}

void FuzzyLocalPlanner::generateTrajectories_forward(Trajectory::VectorPtr trajectories)
{
	double minSpeed = config_.min_speed;
	double maxSpeed = config_.max_speed;
	double minAngle = config_.min_angular_speed;
	double maxAngle = config_.max_angular_speed;
	double speedStep = config_.speed_step;
	double angular_speed_step = config_.angular_speed_step;

	double speed=0;
	bool done =false;
	while(not done)
	{
		speed+=speedStep;
		if(speed>maxSpeed)
		{
			done=true;
			speed = maxSpeed;
		}

		generateTrajectories_angle_iterations(trajectories, speed, 0, false, 0, maxSpeed);

	}

}
void FuzzyLocalPlanner::generateTrajectories_backward(Trajectory::VectorPtr trajectories)
{
	double minSpeed = config_.min_speed;
	double maxSpeed = config_.max_speed;
	double minAngle = config_.min_angular_speed;
	double maxAngle = config_.max_angular_speed;
	double speedStep = config_.speed_step;

	double speed=0;
	bool done =false;
	while(not done)
	{
		speed-=speedStep;
		if(speed<minSpeed)
		{
			done=true;
			speed = minSpeed;
		}

		generateTrajectories_angle_iterations(trajectories, speed, 0, false, minSpeed, 0);

	}
}
void FuzzyLocalPlanner::generateTrajectories_inplace_turn(Trajectory::VectorPtr trajectories)
{
	double minSpeed = config_.min_speed;
	double maxSpeed = config_.max_speed;
	double minAngle = config_.min_angular_speed;
	double maxAngle = config_.max_angular_speed;
	double speedStep = config_.speed_step;

	double speed = speedStep*0.25;
	maxAngle = fabs(minAngle)*(2./3.);
	generateTrajectories_angle_iterations(trajectories, speed, maxAngle,false, speed,speed);
}

void FuzzyLocalPlanner::generateTrajectories()
{
	double minSpeed = config_.min_speed;
	double maxSpeed = config_.max_speed;
	double minAngle = config_.min_angular_speed;
	double maxAngle = config_.max_angular_speed;
	double speedStep = config_.speed_step;

	trajectorySimulator_ = TrajectorySimulator(config_.simulation_time, config_.granularity);
	Trajectory::VectorPtr trajectories(new Trajectory::Vector());

	generateTrajectories_forward(trajectories);
	generateTrajectories_backward(trajectories);
	generateTrajectories_inplace_turn(trajectories);

	trajectories_  = trajectories;
}

void FuzzyLocalPlanner::publishLocalGoal(
		const geometry_msgs::PoseStamped pose)
{
	geometry_msgs::PointStamped point;

	point.header.frame_id = pose.header.frame_id;
	point.header.stamp = ros::Time::now();

	point.point = pose.pose.position;

	localGoalPublisher_.publish(point);
}

void FuzzyLocalPlanner::publishTrajectories(const TrajectoryMatch::SetPtr& matches)
{
	visualization_msgs::MarkerArray markers;

	int id = 1;

	bool scoreColorize = config_.trajectory_score_color;

	bool first = true;

	double max_score = (*(*matches).begin())->getScore();

	ros::Time t = ros::Time::now();
	std::string err = "";

	tfListener_.getLatestCommonTime(global_navigation_frame_id_,getLocalCostmap()->getFrameId(),t, &err);

	if(err != "")
		t = ros::Time::now();

	BOOST_FOREACH(const TrajectoryMatch::Ptr& match, *matches)
	{
		visualization_msgs::Marker marker;

		marker.id = id++;

		marker.header.frame_id = getLocalCostmap()->getFrameId();
		marker.header.stamp = t;

		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = 0;

		marker.scale.x = first ? 0.05 : 0.01f;

		if (first) {
			marker.color.b = 1.0;
			first = false;
		} else {

//			if (scoreColorize) {
				// Color intensity reflects the score

				double norm_score = match->getScore() / max_score;
				marker.color.r = 1.0 - norm_score;
				marker.color.g = norm_score;

//				double colorNormalized = ((match->getScore()) + 1.0) / 2.0;
//
//				marker.color.r = 1 - colorNormalized * 0.5;
//				marker.color.g = colorNormalized * 0.5;

//			} else {

				// Color reflect the danger

				if (match->isFatal()) {
					marker.color.r = 0.0;
					marker.color.g = 0.0;
					marker.color.b = 0.0;
				}
//				else if (match->isBlocked()) {
//					marker.color.r = 1.0;
//					marker.color.g = 0.8;
//				} else {
//					marker.color.b = 1.0;
//				}
//			}

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

		if(not scoreColorize) break;
	}

	trajectoriesPublisher_.publish(markers);
}

void FuzzyLocalPlanner::dynamicConfigCallback(
		navex_navigation::NavexLocalPlannerConfig& config, uint32_t level)
{
	config_ = config;

	localGoalFinder_.setGoalDistance(config.local_goal_distance);

	generateTrajectories();
}



