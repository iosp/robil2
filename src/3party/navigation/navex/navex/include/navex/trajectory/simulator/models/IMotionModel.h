/*
 * Filename: IMotionModel.h
 *   Author: Igor Makhtes
 *     Date: Dec 1, 2014
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


#ifndef INCLUDE_NAVEX_TRAJECTORY_SIMULATOR_MODELS_IMOTIONMODEL_H_
#define INCLUDE_NAVEX_TRAJECTORY_SIMULATOR_MODELS_IMOTIONMODEL_H_


#include <boost/noncopyable.hpp>

#include <tf/tf.h>


/**
 * Motion model interface used by @see TrajectorySimulator
 */
class IMotionModel : public boost::noncopyable {

public:

	/**
	 * Constructs motion model
	 * @param constantMotion If true, assumes that the motion steps are always identical,
	 * so the simulationStep method will be invoked only once, otherwise,
	 * simulateStep will be invoked for each simulation step.
	 */
	IMotionModel(bool constantMotion)
		: constantMotion_(constantMotion) { }

	virtual ~IMotionModel() { }

public:

	/**
	 * Simulate the transformation of a single step
	 * @param timeDelta
	 * @return
	 */
	virtual tf::Transform simulateStep(double timeDelta) const = 0;

	/**
	 * If true, assumes that the motion steps are always identical,
	 * so the simulationStep method will be invoked only once, otherwise,
	 * simulateStep will be invoked for each simulation step.
	 * @return
	 */
	inline bool isConstantMotion() const {
		return constantMotion_;
	}

private:

	bool constantMotion_;

};

#endif /* INCLUDE_NAVEX_TRAJECTORY_SIMULATOR_MODELS_IMOTIONMODEL_H_ */
