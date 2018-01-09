/*
 * Filename: TrajectorySimulator.h
 *   Author: Igor Makhtes
 *     Date: Nov 25, 2014
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


#ifndef INCLUDE_NAVEX_TRAJECTORYSIMULATOR_H_
#define INCLUDE_NAVEX_TRAJECTORYSIMULATOR_H_


#include <navex/trajectory/simulator/models/IMotionModel.h>
#include <navex/trajectory/Trajectory.h>
#include <tf/tf.h>


/**
 * Simulates trajectories using motion models
 */
class TrajectorySimulator {

public:

	TrajectorySimulator(double simulationTime, double granularity);

public:

	/**
	 * Simulates trajectory using specified motion model
	 * @param model Motion model pointer
	 * @warning DO NOT delete the model pointer manually, it will be automatically deleted with trajectories
	 * @return
	 */
	Trajectory::Ptr simulate(const IMotionModel* model) const;

	/**
	 * Sets simulation time in seconds
	 * @param simulationTime
	 */
	inline void setSimulationTime(double simulationTime) {
		simulationTime_ = simulationTime;
	}

	/**
	 * Gets simulation time in seconds
	 * @return
	 */
	inline double getSimulationTime() const {
		return simulationTime_;
	}

	/**
	 * Sets simulation time step in seconds
	 * @param granularity
	 */
	inline void setGranularity(double granularity) {
		granularity_ = granularity;
	}

	/**
	 * Gets simulation time step in seconds
	 * @return
	 */
	inline double getGranularity() const {
		return granularity_;
	}

private:

	/**
	 * Simulation time in seconds
	 */
	double simulationTime_;

	/**
	 * Simulation time step in seconds
	 */
	double granularity_;

};

#endif /* INCLUDE_NAVEX_TRAJECTORYSIMULATOR_H_ */
