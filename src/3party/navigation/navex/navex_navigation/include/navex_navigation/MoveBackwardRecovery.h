/*
 * Filename: MoveBackwardRecovery.h
 *   Author: Igor Makhtes
 *     Date: Jun 29, 2015
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

#ifndef INCLUDE_NAVEX_NAVIGATION_MOVEBACKWARDRECOVERY_H_
#define INCLUDE_NAVEX_NAVIGATION_MOVEBACKWARDRECOVERY_H_


#include <navex/costmap/CostMap.h>
#include <navex/controller/IController.h>

#include <navex/planner/local/RecoveryPlannerBase.h>
#include <navex/trajectory/simulator/TrajectorySimulator.h>
#include <navex/trajectory/simulator/models/SkidSteerModel.h>
#include <navex/trajectory/matcher/SimpleTrajectoryMatcher.h>


/**
 * Trying to move the robot backward
 */
class MoveBackwardRecovery : public RecoveryPlannerBase {

public:

	MoveBackwardRecovery(CostMap* localCostmap, CostMap* globalCostmap,
			IController* robotController, double speed = -0.1);

	virtual ~MoveBackwardRecovery();

public:

	/**
	 * Main control loop
	 * @return
	 */
	virtual PlannerStatus update();

	/**
	 * Gets planner name
	 * @return
	 */
	virtual inline string getName() const {
		return "move_backward_recovery";
	}

private:

	double speed_;

	Trajectory::VectorPtr frontTrajectories_;
	Trajectory::Ptr backTrajectory_;

	ITrajectoryMatcher* trajectoryMatcher_;

private:

	/**
	 * Checks if the front is clear and safe to move
	 * @return
	 */
	bool checkFrontTrajectories() const;

	/**
	 * Checks if the robot can move backward without collision
	 * @return
	 */
	bool checkBackIsSafeToMove() const;

	void createFrontTrajectories();

};

#endif /* INCLUDE_NAVEX_NAVIGATION_MOVEBACKWARDRECOVERY_H_ */
