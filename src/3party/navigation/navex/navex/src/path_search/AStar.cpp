/*
 * Filename: AStar.cpp
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

#include <navex/path_search/AStar.h>


AStar::AStar() {
}

AStar::~AStar() {
}

bool AStar::findPath(const CostMap& costMap, const geometry_msgs::PoseStamped& start,
		 const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path) const {


	Point startPoint = costMap.poseToPixel(start);
	Point goalPoint = costMap.poseToPixel(goal);
	Path result_path;

	SetOfUsedPoints closedset = cv::Mat::zeros(
			costMap.getHeight(), costMap.getWidth(), CV_8UC1);

	SetOfUsedPoints openset_contains = cv::Mat::zeros(
			costMap.getHeight(), costMap.getWidth(), CV_8UC1);

	SortedSetOfPoints openset;			// The set of tentative nodes to be evaluated,
	PointsMap points(costMap.getWidth(), costMap.getHeight());

	double tentative_g_score;

	points.get(startPoint.xy()) = startPoint;
	startPoint.score = heuristicCostEstimate(startPoint, goalPoint);
	points.get(goalPoint) = goalPoint;
	insert(openset, startPoint);				//initially containing the start node
	insert(openset_contains, startPoint);

	while( not openset.empty() ){
		Point& current = points.get( top(openset) );	//the node in openset having the lowest f_score[] value
		if( current.xy() == goalPoint.xy() ){
			reconstructPath(points, goalPoint.xy(), result_path);

			path.poses.clear();
			path.header.frame_id = costMap.getFrameId();
			path.header.stamp = ros::Time::now();

			BOOST_FOREACH(const Point::XY& p, result_path) {
				geometry_msgs::PoseStamped pose = costMap.pixelToPose(p.x, p.y);
				path.poses.push_back(pose);
			}

			return true;
		}

		pop(openset);					//remove current from openset
		remove(openset_contains,current);

		insert(closedset,current);		//add current to closedset

		static vector<Point::XY> neighbor_list(20);

		neighborNodes(costMap, current, closedset, neighbor_list);
		BOOST_FOREACH( Point::XY& neighbor_ , neighbor_list){

			Point& neighbor = points.get(neighbor_);
			double g_score = neighbor.g_score;
			if(g_score==0){
				neighbor.x = neighbor_.x;
				neighbor.y = neighbor_.y;
			}

			double d = dist(current,neighbor);
			double tentative_g_score = current.g_score + d;

			if( not in(neighbor, openset_contains) or (tentative_g_score < neighbor.g_score) ){
				neighbor.parent = &current;
				neighbor.g_score = tentative_g_score;
				double score = neighbor.g_score + heuristicCostEstimate(neighbor, goalPoint);
				updateScore(openset, openset_contains, neighbor, score);
			}
		}

	}

	return false;
}

void AStar::neighborNodes(const CostMap& costMap, Point current,
		const SetOfUsedPoints& closed, vector<Point::XY>& neighbor) const {
	neighbor.resize(0);
	Point n;
	n = Point(current.x-1,current.y-0); if( valide(costMap, n,closed) ) neighbor.push_back(n.xy());
	n = Point(current.x-1,current.y-1); if( valide(costMap, n,closed) ) neighbor.push_back(n.xy());
	n = Point(current.x-1,current.y+1); if( valide(costMap, n,closed) ) neighbor.push_back(n.xy());
	n = Point(current.x-0,current.y-1); if( valide(costMap, n,closed) ) neighbor.push_back(n.xy());
	n = Point(current.x-0,current.y+1); if( valide(costMap, n,closed) ) neighbor.push_back(n.xy());
	n = Point(current.x+1,current.y-1); if( valide(costMap, n,closed) ) neighbor.push_back(n.xy());
	n = Point(current.x+1,current.y-0); if( valide(costMap, n,closed) ) neighbor.push_back(n.xy());
	n = Point(current.x+1,current.y+1); if( valide(costMap, n,closed) ) neighbor.push_back(n.xy());
}

void AStar::reconstructPath(const PointsMap& points,
		const Point::XY& _current_node, Path& result_path) const {

	const Point* current_node = &(points.get(_current_node));
	bool has_parent = current_node->parent;

	while(has_parent) {
		result_path.push_front(current_node->xy());
		current_node = current_node->parent;
		has_parent = current_node->parent;
	}

	result_path.push_front(current_node->xy());
}
