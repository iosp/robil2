/*
 * Filename: IDriver.h
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

#ifndef INCLUDE_NAVEX_CONTROLLER_ICONTROLLER_H_
#define INCLUDE_NAVEX_CONTROLLER_ICONTROLLER_H_


#include <navex/trajectory/Trajectory.h>


/**
 * Robot controller interface, used to set drive commands to robots
 * with different motion models
 */
class IController {

public:

	virtual ~IController() { }

public:

	/**
	 * Set drive command
	 * @param motion
	 */
	virtual void drive(const IMotionModel* motion) const = 0;

	/**
	 * Drives forward or backward
	 * @param speed Speed in m/s
	 */
	virtual void driveLinear(double speed) const = 0;

	/**
	 * Stops the robot
	 */
	virtual void stop() const = 0;

};

#endif /* INCLUDE_NAVEX_CONTROLLER_ICONTROLLER_H_ */
