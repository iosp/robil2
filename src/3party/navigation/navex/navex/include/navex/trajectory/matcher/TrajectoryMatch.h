/*
 * Filename: TrajectoryMatch.h
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


#ifndef INCLUDE_NAVEX_TRAJECTORYMATCH_H_
#define INCLUDE_NAVEX_TRAJECTORYMATCH_H_


#include <set>

#include <navex/trajectory/Trajectory.h>


using namespace std;


/**
 * Holds trajectory match result
 */
class TrajectoryMatch {

public:

	typedef boost::shared_ptr<TrajectoryMatch> Ptr;

public:

	struct TrajectoryMatchCostComparator {
		bool operator() (const TrajectoryMatch& first,
				const TrajectoryMatch& second) const {
			return first.getScore() > second.getScore();
		}

		bool operator() (const TrajectoryMatch::Ptr& first,
				const TrajectoryMatch::Ptr& second) const {
			return first->getScore() > second->getScore();
		}
	};

public:

	typedef multiset<TrajectoryMatch::Ptr, TrajectoryMatch::TrajectoryMatchCostComparator> Set;
	typedef boost::shared_ptr<Set> SetPtr;

public:

	/**
	 * Constructs trajectory match
	 * @param trajectory
	 * @param score
	 */
	TrajectoryMatch(const Trajectory::Ptr& trajectory, double score);

public:

	/**
	 * Gets score of this trajectory
	 * @return
	 */
	inline double getScore() const {
		return score_;
	}

	/**
	 * Gets trajectory
	 * @return
	 */
	inline Trajectory::Ptr getTrajectory() const {
		return trajectory_;
	}

	/**
	 * Returns true if path goes through inflated area
	 * @return
	 */
	inline bool isBlocked() const {
		return isBlocked_;
	}

	/**
	 * Returns true if path leads to collision
	 * @return
	 */
	inline bool isFatal() const {
		return isFatal_;
	}

	inline void setFatal(bool isFatal) {
		isFatal_ = isFatal;
	}

	/**
	 * Sets a flag that this trajectory may be blocked
	 * @param isBlocked
	 */
	inline void setBlocked(bool isBlocked) {
		isBlocked_ = isBlocked;
	}

	/**
	 * Returns true if the path goes out of blocked area (not fatal area)
	 * @return
	 */
	inline bool leadsToFree() const {
		return leadsToFree_;
	}

	inline void setLeadsToFree(bool leadsToFree) {
		leadsToFree_ = leadsToFree;
	}

private:

	const double score_;

	const Trajectory::Ptr trajectory_;

	bool isBlocked_;
	bool isFatal_;

	bool leadsToFree_;

};


#endif /* INCLUDE_NAVEX_TRAJECTORYMATCH_H_ */
