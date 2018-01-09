/*
 * Filename: IParametersProvider.h
 *   Author: Igor Makhtes
 *     Date: Dec 3, 2014
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


#ifndef IPARAMETERSPROVIDER_H_
#define IPARAMETERSPROVIDER_H_


#include <string>


using namespace std;


/**
 * Cost map parameters provider interface
 */
class IParametersProvider {

public:

	virtual ~IParametersProvider() { }

public:

	/**
	 * Parameters updated event callback
	 */
	boost::function<void(void)> updateParametersCallback;

public:

	/**
	 * Gets map width in meters
	 * @return
	 */
	virtual double getMapWidth() const = 0;

	/**
	 * Gets map height in meters
	 * @return
	 */
	virtual double getMapHeight() const = 0;

	/**
	 * Gets map resolution in p/m
	 * @return
	 */
	virtual double getMapResolution() const = 0;

	/**
	 * Gets obstacle inflation radius in meters
	 * @return
	 */
	virtual double getInflationRadius() const = 0;

	virtual double getInflationSigmma() const = 0;
	virtual double getInflationPow() const = 0;

	/**
	 * Gets robot radius in meters
	 * @return
	 */
	virtual double getRobotRadius() const = 0;

	/**
	 * Gets the frame id of the cost map
	 * @return
	 */
	virtual string getMapFrameId() const = 0;

	/**
	 * Sets map width in meters
	 * @param mapWidth
	 */
	virtual void setMapWidth(double mapWidth) = 0;

	/**
	 * Sets map height in meters
	 * @param mapHeight
	 */
	virtual void setMapHeight(double mapHeight) = 0;

	/**
	 * Sets map resolution in px/m
	 * @param resolution
	 */
	virtual void setMapResolution(double resolution) = 0;

	/**
	 * Sets frame id of the map
	 * @param frameId
	 */
	virtual void setFrameId(const string& frameId) = 0;

protected:

	/**
	 * Raises parameters updated event
	 */
	void raiseUpdateParameters() {
		if (!updateParametersCallback)
			return;

		updateParametersCallback();
	}

};

#endif /* IPARAMETERSPROVIDER_H_ */
