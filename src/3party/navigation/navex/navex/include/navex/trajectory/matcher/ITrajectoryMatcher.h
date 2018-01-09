/*
 * Filename: ITrajectoryMatcher.h
 *   Author: Igor Makhtes
 *     Date: Nov 29, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014
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


#ifndef INCLUDE_NAVEX_ITRAJECTORYMATCHER_H_
#define INCLUDE_NAVEX_ITRAJECTORYMATCHER_H_


#include <navex/costmap/CostMap.h>
#include <navex/trajectory/matcher/TrajectoryMatch.h>


/**
 * Trajectory matcher interface, used to evaluate trajectories against a cost map
 */
class ITrajectoryMatcher {

public:

	virtual ~ITrajectoryMatcher() { }

public:

	enum MatchStrategy
	{
		MS_NORMAL,
		MS_RECOVERY
	};

	/**
	 * Evaluate one trajectory
	 * @param costMap
	 * @param trajectory
	 * @return
	 */
	virtual TrajectoryMatch::Ptr match(const CostMap& localCostMap, const CostMap& globalCostMap,
			const Trajectory::Ptr& trajectory, const MatchStrategy strategy) const = 0;

	/**
	 * Evaluate number of trajectories and returns a sorted set of matches by score
	 * @param costMap
	 * @param trajectories
	 * @return
	 */
	inline TrajectoryMatch::SetPtr match(const CostMap& localCostMap, const CostMap& globalCostMap,
			const Trajectory::VectorPtr& trajectories, const MatchStrategy strategy) const {
		TrajectoryMatch::SetPtr trajectoryMatchSet(new TrajectoryMatch::Set());

		for (int i = 0; i < trajectories->size(); ++i) {
			trajectoryMatchSet->insert(
					match(localCostMap, globalCostMap, (*trajectories)[i], strategy));
		}

		return trajectoryMatchSet;
	}

};

#endif /* INCLUDE_NAVEX_ITRAJECTORYMATCHER_H_ */
