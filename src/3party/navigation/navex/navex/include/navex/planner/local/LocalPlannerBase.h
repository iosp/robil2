/*
 * Filename: LocalPlannerBase.h
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

#ifndef INCLUDE_NAVEX_PLANNER_LOCALPLANNERBASE_H_
#define INCLUDE_NAVEX_PLANNER_LOCALPLANNERBASE_H_


#include <navex/costmap/CostMap.h>
#include <navex/controller/IController.h>
#include <navex/trajectory/simulator/models/IMotionModel.h>


/**
 * Base class for local planner
 */
class LocalPlannerBase {

public:

	LocalPlannerBase(CostMap* localCostmap, CostMap* globalCostmap,
			IController* robotController)
		: localCostmap_(localCostmap), globalCostmap_(globalCostmap),
		  robotController_(robotController),
		  forceNavigation_(false) { }

	virtual ~LocalPlannerBase() { }

public:

	enum PlannerStatus {
		STATUS_OK,
		STATUS_NO_PLAN_FOUND,
		STATUS_PATH_BLOCKED,
		STATUS_IN_OBSTACLE,
		STATUS_UNKNOWN_ERROR,

		STATUS_RECOVERY_OK,
		STATUS_RECOVERY_FAILED,
		STATUS_RECOVERY_FINISHED
	};

public:

	/**
	 * Sets new plan
	 * @param path
	 */
	virtual inline void setPlan(const nav_msgs::Path& path) {
		path_ = path;
		planUpdated();
	}

	/**
	 * Gets path
	 * @return
	 */
	virtual inline nav_msgs::Path getPlan() const {
		return path_;
	}

	/**
	 * Gets the local costmap
	 * @return
	 */
	inline const CostMap* getLocalCostmap() const {
		return localCostmap_;
	}

	/**
	 * Gets the global costmap
	 * @return
	 */
	inline const CostMap* getGlobalCostmap() const {
		return globalCostmap_;
	}

	virtual inline void setForceNavigation(bool forceNavigation) {
		forceNavigation_ = forceNavigation;
	}

	virtual inline bool isForceNavigation() const {
		return forceNavigation_;
	}

	/**
	 * Main control loop
	 * @return
	 */
	virtual PlannerStatus update() = 0;

	/**
	 * Gets planner name
	 * @return
	 */
	virtual string getName() const = 0;

protected:

	const CostMap* localCostmap_;
	const CostMap* globalCostmap_;

	const IController* robotController_;

	nav_msgs::Path path_;

	/**
	 * If true, the planner must continue navigation despite
	 * possible problems with global path
	 */
	bool forceNavigation_;

protected:

	/**
	 * Plan updated event
	 */
	virtual void planUpdated() {	}

	virtual void driveCommand(const IMotionModel* motionModel) const {
		robotController_->drive(motionModel);
	}

};

#endif /* INCLUDE_NAVEX_PLANNER_LOCALPLANNERBASE_H_ */
