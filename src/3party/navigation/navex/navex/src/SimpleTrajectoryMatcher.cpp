/*
 * Filename: SimpleTrajectoryMatcher.cpp
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


#include <navex/trajectory/matcher/SimpleTrajectoryMatcher.h>


SimpleTrajectoryMatcher::SimpleTrajectoryMatcher() {
}

SimpleTrajectoryMatcher::~SimpleTrajectoryMatcher() {
}

TrajectoryMatch::Ptr SimpleTrajectoryMatcher::match(const CostMap& localCostMap,
		const CostMap& globalCostMap,
		const Trajectory::Ptr& trajectory
		, const ITrajectoryMatcher::MatchStrategy strategy ) const {

	const nav_msgs::Path::Ptr& path = trajectory->getPath();

	/**
	 * Score ranges from -1 (Unknown) to 100 (definitely occupied)
	 */
	const int pointScoreRange = CostMapCell::CELL_MAX - CostMapCell::CELL_MIN;

	/**
	 * Maximum possible trajectory weight
	 */
	const double maxTrajectoryWeight = 1;

	/**
	 * Maximum score, used to normalize the value to range 0..1
	 */
	double maxPathScore = path->poses.size() * pointScoreRange * maxTrajectoryWeight;

	bool maybeBlocked = false;
	bool fatalPath = false;
	bool leadsToFree = false;

	double scoreSum = 0;

	for (int i = 0; i < path->poses.size(); ++i) {
		const geometry_msgs::Pose& pose = path->poses[i].pose;
		const double pointValue = 1.0 + (double)localCostMap.getCellValue(pose);

		if (pointValue >= CostMapCell::CELL_MAYBE_BLOCKED) {
			maybeBlocked = true;
		}

		/**
		 * Fatal path check
		 */
		if (pointValue >= CostMapCell::CELL_BLOCKED) {
			/**
			 * The path is definitely blocked, mark it as fatal
			 */
			fatalPath = true;
		}

		/**
		 * Check if the path leads to free cell (not through blocked cell)
		 */
		if (!fatalPath && !leadsToFree) {
			if (pointValue <= CostMapCell::CELL_FREE)
				leadsToFree = true;
		}

		scoreSum += pointValue;
	}

	/**
	 * Empty path
	 */
	if (maxPathScore == 0)
		return TrajectoryMatch::Ptr(new TrajectoryMatch(trajectory, 0));

	/**
	 * Normalize and invert
	 */
	double finalScore = (1 - scoreSum / maxPathScore) * trajectory->getWeight();

	/**
	 * Not free path, adjust the score to be in range [-1,0]
	 */
	if (maybeBlocked)
		finalScore -= 1;

	TrajectoryMatch::Ptr trajectoryMatch(new TrajectoryMatch(trajectory, finalScore));

	trajectoryMatch->setFatal(fatalPath);
	trajectoryMatch->setBlocked(maybeBlocked);
	trajectoryMatch->setLeadsToFree(leadsToFree);

	return trajectoryMatch;
}
