/*
 * Filename: AStar.h
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

#ifndef INCLUDE_NAVEX_PATH_SEARCH_ASTAR_H_
#define INCLUDE_NAVEX_PATH_SEARCH_ASTAR_H_


#include <map>
#include <queue>
#include <vector>
#include <set>
#include <algorithm>
#include <boost/foreach.hpp>

#include <navex/path_search/PathSearchBase.h>


using namespace std;


/**
 * A* graph search algorithm implementation
 */
class AStar : public PathSearchBase {

public:

	AStar();

	virtual ~AStar();

public:

	/**
	 * Find path using A* algorithm
	 * @param costMap The map
	 * @param start The start pose
	 * @param goal The goal pose
	 * @param [out] path The found plan
	 * @return True if path found, false otherwise
	 */
	virtual bool findPath(const CostMap& costMap, const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path) const;

private:

	struct Point{


		Point(const cv::Point& p)
			: x(p.x), y(p.y), score(0), g_score(0), parent(0) {
		}

		Point(int x = 0, int y = 0)
			: x(x), y(y), score(0), g_score(0), parent(0){}

		bool operator<(const Point& p)const{
			return
					score < p.score	? true  :
					score > p.score	? false :
					x < p.x			? true  :
					x > p.x			? false :
					y < p.y;
		}

		bool operator==(const Point& p)const{
			return x == p.x && y == p.y;
		}

		struct XY {

			int x; int y;

			bool operator==(const XY& p)const{
				return x == p.x && y == p.y;
			}

			bool operator<(const XY& v)const{
				return	x < v.x?true:
						x > v.x?false:
						y < v.y;
			};

		};

		XY xy()const{ XY v={x,y}; return v; }

		int x,y;
		double score, g_score;
		Point* parent;

	};

	struct PointsMap{

		PointsMap(int w, int h) :
			w(w), h(h), points(new Point[w*h]) {

		}

		~PointsMap() {
			delete[] points;
		}

		Point& get(int x, int y) {
			return points[y * w + x];
		}

		const Point& get(int x, int y) const {
			return points[y*w+x];
		}

		Point& get(const Point& p){
			return get(p.x,p.y);
		}

		const Point& get(const Point& p) const {
			return get(p.x, p.y);
		}

		Point& get(const Point::XY& p) {
			return get(p.x, p.y);
		}

		const Point& get(const Point::XY& p) const {
			return get(p.x, p.y);
		}

		int w,h;
		Point* points;
	};

private:

	typedef cv::Mat SetOfUsedPoints;
	typedef std::set<AStar::Point> SortedSetOfPoints;
	typedef std::map<AStar::Point::XY,AStar::Point::XY> MapPointToPoint;
	typedef std::list<AStar::Point::XY> Path;

private:

	double dist(const Point& a, const Point& b) const {
			return hypot( b.x-a.x, b.y-a.y );
	}

	bool occupedPoint(const CostMap& costMap, Point p) const {
		uchar v = costMap.getCellValue(p.x, p.y);
		/// TODO May change in the future
		return v >= CostMapCell::CELL_BLOCKED;
	}

	double heuristicCostEstimate(Point start, Point goal) const {
		return dist(start,goal);
	}

	bool valide(const CostMap& costMap, Point p, const SetOfUsedPoints& closed) const {
		bool in_range = 0<=p.x and p.x<costMap.getWidth() and 0<=p.y and p.y<costMap.getHeight();
		bool not_closed = not in(p,closed);
		in_range = in_range and not_closed;
		return in_range and not occupedPoint(costMap, p);
	}

	void neighborNodes(const CostMap& costMap, Point current,
			const SetOfUsedPoints& closed, vector<Point::XY>& neighbor) const;

	Point top(const SortedSetOfPoints& s)const{
		return *s.begin();
	}

	void pop(SortedSetOfPoints& s)const{
		s.erase(s.begin());
	}

	bool in(const Point& p, const SetOfUsedPoints& s)const{
		return s.at<uchar>(cv::Point(p.x, p.y)) == 255;
	}

	template<class S,class T>
	bool in(const T& t, const S& s)const{
		return s.find(t)!=s.end();
	}

	void remove(SortedSetOfPoints& s, const Point& p)const{
		s.erase(p);
	}

	void remove(SetOfUsedPoints& s, const Point& p)const{
		s.at<uchar>(cv::Point(p.x, p.y)) = 0;
	}

	void insert(SortedSetOfPoints& s, const Point& p)const{
		s.insert(p);
	}

	void insert(SetOfUsedPoints& s, const Point& p)const{
		s.at<uchar>(cv::Point(p.x, p.y)) = 255;
	}

	void updateScore(SortedSetOfPoints& openset, SetOfUsedPoints& contains, Point& point, double score)const{
		if( in(point, contains) ){
			remove(openset, point);
			remove(contains, point);
		}
		point.score =  score;
		insert(openset,point);
		insert(contains, point);
	}

	void reconstructPath(const PointsMap& points,
			const Point::XY& _current_node, Path& result_path) const;

};

#endif /* INCLUDE_NAVEX_PATH_SEARCH_ASTAR_H_ */
