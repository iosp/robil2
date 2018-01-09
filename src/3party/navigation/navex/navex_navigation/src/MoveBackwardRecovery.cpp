/*
 * Filename: MoveBackwardRecovery.cpp
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

#include <navex_navigation/MoveBackwardRecovery.h>


MoveBackwardRecovery::MoveBackwardRecovery(CostMap* localCostmap,
		CostMap* globalCostmap,
		IController* robotController, double speed)
	: RecoveryPlannerBase(localCostmap, globalCostmap, robotController),
	  speed_(speed) {

	trajectoryMatcher_ = new SimpleTrajectoryMatcher();

	speed = -fabs(speed);

	createFrontTrajectories();
}

MoveBackwardRecovery::~MoveBackwardRecovery() {
	delete trajectoryMatcher_;
}

LocalPlannerBase::PlannerStatus MoveBackwardRecovery::update() {

	if (checkBackIsSafeToMove())
		robotController_->driveLinear(speed_);
	else {
		robotController_->stop();

		/*
		 * Can't move backward, recovery failed
		 */
		return LocalPlannerBase::STATUS_RECOVERY_FAILED;
	}

	if (checkFrontTrajectories()) {
		robotController_->stop();

		/*
		 * We done
		 */
		return LocalPlannerBase::STATUS_RECOVERY_FINISHED;
	}

	return LocalPlannerBase::STATUS_RECOVERY_OK;
}

bool MoveBackwardRecovery::checkFrontTrajectories() const {
	TrajectoryMatch::SetPtr matches =
			trajectoryMatcher_->match(
					*getLocalCostmap(), *getGlobalCostmap(), frontTrajectories_, ITrajectoryMatcher::MS_NORMAL);

	if ((*matches->begin())->isBlocked())
		return false;

	return true;
}

bool MoveBackwardRecovery::checkBackIsSafeToMove() const {
	TrajectoryMatch::Ptr match = trajectoryMatcher_->match(
			*getLocalCostmap(), *getLocalCostmap(), backTrajectory_, ITrajectoryMatcher::MS_NORMAL);

	return match->leadsToFree() || true; // TODO fix
}

void MoveBackwardRecovery::createFrontTrajectories() {
	frontTrajectories_ = Trajectory::VectorPtr(new Trajectory::Vector());

	/**
	 * Front trajectories
	 */
	TrajectorySimulator simulator(1.5, 0.05);

	frontTrajectories_->push_back(simulator.simulate(new SkidSteerModel(0.3, -0.4)));
	frontTrajectories_->push_back(simulator.simulate(new SkidSteerModel(0.3, 0.0)));
	frontTrajectories_->push_back(simulator.simulate(new SkidSteerModel(0.3, 0.4)));

	/**
	 * Back trajectories
	 */
	simulator = TrajectorySimulator(2.0, 0.05);
	backTrajectory_ = simulator.simulate(new SkidSteerModel(-0.3, 0.0));

}
