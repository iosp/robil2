/*
 * FuzzyTrajectoryMatcher.cpp
 *
 *  Created on: Jan 15, 2017
 *      Author: assaf
 */

#include <navex_navigation/FuzzyTrajectoryMatcher.h>

FuzzyTrajectoryMatcher::FuzzyTrajectoryMatcher(navex_navigation::NavexLocalPlannerConfig * config)
	: goal_(0, 0),local_planner_config_(config)
{

}

FuzzyTrajectoryMatcher::~FuzzyTrajectoryMatcher()
{

}



TrajectoryMatch::Ptr FuzzyTrajectoryMatcher::match(const CostMap& localCostMap,
		const CostMap& globalCostMap,
		const Trajectory::Ptr& trajectory,
		const MatchStrategy strategy
		) const
{
	tf::Vector3 goalVector;

	tf::pointMsgToTF(localCostMap.pixelToPose(goal_).pose.position, goalVector);

	FuzzyTrajectory fuzzy(trajectory);

	tf::Vector3 robot_pose(0,0,0);

	double sim_time = local_planner_config_->simulation_time;
	double max_speed = local_planner_config_->max_speed;
	double min_speed = local_planner_config_->min_speed;

	double speed_diffrance = sim_time * (max_speed - min_speed);

	double conf_fazzy_straight_stdev = local_planner_config_->fazzy_straight_stdev;
	double conf_fazzy_straight_pow = local_planner_config_->fazzy_straight_pow;
	double conf_fazzy_straight_min = local_planner_config_->fazzy_straight_min;

	//FUZZIFICATION

	FuzzyValue safe = fuzzy.is_safe(localCostMap, min_speed, max_speed);
	FuzzyValue fast = fuzzy.is_fast(max_speed);
	FuzzyValue on_goal = fuzzy.is_on_goal(localCostMap, goalVector, robot_pose, max_speed, sim_time);
	FuzzyValue straight = fuzzy.is_straight(localCostMap, conf_fazzy_straight_stdev, conf_fazzy_straight_pow, conf_fazzy_straight_min);
	FuzzyValue backward = fuzzy.is_backward();
	FuzzyValue stable = last_best_.trajectory() == NULL ? FuzzyValue(1) : fuzzy.is_stable(localCostMap, last_best_, speed_diffrance);
	FuzzyValue recovery = fuzzy.is_recovery(localCostMap);

	//DECISION

	FuzzyValue valid(0);

		valid =
		//FOR NORMAL DRIVING
				priority(
					very(safe,					local_planner_config_->fazzy_safe			) and
					very(fast,					local_planner_config_->fazzy_fast			) and
					very(on_goal,				local_planner_config_->fazzy_goal_oriented	) and
					very(straight,				local_planner_config_->fazzy_straight		) and
					very(stable,				local_planner_config_->fazzy_stable			) and
					very(not backward,			local_planner_config_->fazzy_forward		) and
					very(recovery,				local_planner_config_->fazzy_recovery		)
				,1.0)
				or
		//FOR RECOVERY
				priority(
						very(not fast,			1./4.	) and
						very(on_goal,			1./4.	) and
						very(straight,			0.0		) and
						very(stable,			1./2.	) and
						very(not backward,		1./4	) and
						very(recovery,			8.		)
				,1E-10)

				;

	//DEFUZZIFICATION

	FSet_ValidTrajectory vt;
	double valid_score = vt(valid);

	TrajectoryMatch::Ptr match(new TrajectoryMatch(trajectory, valid_score));

	if(valid_score < 1e-60)
		match->setFatal(true);

	return match;
}

