/*
 * FuzzyTrajectoryMatcher.h
 *
 *  Created on: Jan 15, 2017
 *      Author: assaf
 */

#ifndef NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZYTRAJECTORYMATCHER_H_
#define NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZYTRAJECTORYMATCHER_H_

#include <cmath>
#include <navex/trajectory/matcher/ITrajectoryMatcher.h>
#include <navex_navigation/Fuzzy.h>
#include <navex_navigation/NavexLocalPlannerConfig.h>

class FuzzyTrajectoryMatcher : public ITrajectoryMatcher
{
private:
	cv::Point goal_;
	FuzzyTrajectory last_best_;
	navex_navigation::NavexLocalPlannerConfig * local_planner_config_;

public:
	inline void setGoal(const cv::Point& goal)
	{
		goal_ = goal;
	}

	inline void setLastBestTrajectory(const Trajectory::Ptr & trajectory)
	{
		last_best_ = FuzzyTrajectory(trajectory);
	}

	inline Trajectory::Ptr getLastBestTrajectory()const
	{
		return last_best_.trajectory();
	}

	inline cv::Point getGoal() const
	{
		return goal_;
	}


public:
	FuzzyTrajectoryMatcher(navex_navigation::NavexLocalPlannerConfig * config);
	virtual ~FuzzyTrajectoryMatcher();

	virtual TrajectoryMatch::Ptr match(const CostMap& localCostMap,
			const CostMap& globalCostMap,
			const Trajectory::Ptr& trajectory,
			const MatchStrategy strategy) const;
};



#endif /* NAVEX_NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_FUZZYTRAJECTORYMATCHER_H_ */
