/*
 * Filename: PathSearchBase.h
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


#ifndef INCLUDE_NAVEX_PATH_SEARCH_PATHSEARCHBASE_H_
#define INCLUDE_NAVEX_PATH_SEARCH_PATHSEARCHBASE_H_


#include <navex/costmap/CostMap.h>

#include <nav_msgs/Path.h>


/**
 * Abstract base class for path search algorithms
 */
class PathSearchBase {

public:

	PathSearchBase() { }

	virtual ~PathSearchBase() { }

public:

	/**
	 * Finds a path from start to goal
	 * @param costMap The map
	 * @param start The start pose
	 * @param goal The goal pose
	 * @param [out] path The found plan
	 * @return True if path found, false otherwise
	 */
	virtual bool findPath(const CostMap& costMap, const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path) const = 0;


};

#endif /* INCLUDE_NAVEX_PATH_SEARCH_PATHSEARCHBASE_H_ */
