/*
 * GoalCalculatorMath.h
 *
 *  Created on: May 7, 2017
 *      Author: assaf
 */

#ifndef NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_GOALCALCULATORMATH_H_
#define NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_GOALCALCULATORMATH_H_

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <queue>
#include <opencv2/opencv.hpp>

using namespace std;

namespace goal_calculator
{

	inline bool in_range(double a, double mi, double ma)
	{
		return mi <= a and a <= ma;
	}

	inline bool in_range(double a, double mi, double ma, bool miw, bool maw)
	{
		bool _mi, _ma;
		if(miw) _mi = mi <= a; else _mi = mi < a;
		if(maw) _ma = ma <= a; else _ma = ma < a;
		return _mi and _ma;
	}

	class Point_2d
	{
	public:
		double x, y;

		Point_2d(double x = 0, double y = 0):x(x), y(y)
		{}

		explicit Point_2d(const cv::Point2d &p):x(p.x), y(p.y)
		{}

		static Point_2d polar(double ang, double len)
		{ return Point_2d(len * cos(ang), len * sin(ang)); }

		double ang() const
		{ return atan2(y, x); }
		double len() const
		{ return hypot(x, y); }

		bool operator==(const Point_2d &p) const
		{ return ((*this) - p).len() < 1E-5; }
		bool operator!=(const Point_2d &p) const
		{ return not((*this) == p); }

#	define DEF_OPERATOR(OPNAME) \
        Point_2d operator OPNAME(const Point_2d& p) const { return Point_2d(x OPNAME p.x,y OPNAME p.y); }\
        Point_2d operator OPNAME(const double& p) const { return Point_2d(x OPNAME p,y OPNAME p); }

		DEF_OPERATOR(+)
		DEF_OPERATOR(-)
		DEF_OPERATOR(/)
		DEF_OPERATOR(*)

#	undef DEF_OPERATOR

#	define DEF_OPERATOR(OPNAME) \
        void operator OPNAME(const Point_2d& p){ *this = Point_2d(x OPNAME p.x,y OPNAME p.y); }\
        void operator OPNAME(const double& p) { *this = Point_2d(x OPNAME p,y OPNAME p); }

		DEF_OPERATOR(+=)
		DEF_OPERATOR(-=)
		DEF_OPERATOR(/=)
		DEF_OPERATOR(*=)

#	undef DEF_OPERATOR

		Point_2d rotated(double a) const
		{ return polar(a + ang(), len()); }
		void rotate(double a)
		{ *this = rotated(a); }
		Point_2d norm() const
		{ return *this / len(); }

		friend ostream& operator<<(ostream& out, const Point_2d& p)
		{
			out << "(" << p.x << "," << p.y << ")";
			return out;
		}
	};

	typedef int Index;

	class Map
	{
	public:
		typedef unsigned char cell_t;
		vector<cell_t> cells;
		size_t w, h;
		Point_2d offset;
		double heading;
		double resolution;

		static bool is_free_value(cell_t c)
		{ return c & 0x01; }
		static bool is_occupied_value(cell_t c)
		{ return c & 0x02; }
		static bool is_accessible_value(cell_t c)
		{ return c & 0x10; }

		static void set_free_value(cell_t &c)
		{ c = (c & 0xf0) | 0x01; }
		static void set_occupied_value(cell_t &c)
		{ c = (c & 0xf0) | 0x02; }
		static void set_accessible_value(cell_t &c)
		{ c = (c & 0x0f) | 0x10; }
		static void set_inaccessible_value(cell_t &c)
		{ c = (c & 0x0f) | 0x00; }

		Map(size_t w, size_t h, Point_2d offset, double heading, double resolution)
				:cells(w * h, 0), w(w), h(h), offset(offset), heading(heading), resolution(resolution)
		{}

		Point_2d to_cell_coordinates(const Point_2d &world) const
		{
			Point_2d c = world - offset;
			c.rotate(-heading);
			c /= resolution;
			return c;
		}

		Point_2d to_world_coordinates(const Point_2d &c) const
		{
			Point_2d world = c * resolution;
			world.rotate(heading);
			world += offset;
			return world;
		}

		bool in_range_cell(const Point_2d &cell) const
		{
			if(round(cell.x) < 0) return false;
			if(round(cell.y) < 0) return false;
			if(round(cell.x) >= w) return false;
			if(round(cell.y) >= h) return false;
			return true;
		}

		bool in_range(const Point_2d &world) const
		{ return in_range_cell(to_cell_coordinates(world)); }

		Index index(double x, double y) const
		{ return Index(w * round(y) + round(x)); }
		Index index(const Point_2d &cell) const
		{ return index(cell.x, cell.y); }
		Index point_id(const Point_2d &p) const
		{
			Point_2d c = to_cell_coordinates(p);
			return index(c.x, c.y);
		}

		const cell_t &operator[](Index i) const
		{
			assert(0 <= i and i < (Index) cells.size());
			return cells[i];
		}

		cell_t &operator[](Index i)
		{
			assert(0 <= i and i < (Index) cells.size());
			return cells[i];
		}

		const cell_t &operator()(double x, double y) const
		{ return (*this)(Point_2d(x, y)); }
		cell_t &operator()(double x, double y)
		{ return (*this)(Point_2d(x, y)); }
		const cell_t &operator()(const Point_2d &p) const
		{
			Point_2d c = to_cell_coordinates(p);
			//std::cout<<"[is_Accesible]            cell index = "<< c << " : "<< (index(c.x, c.y)) << " cells.size() = "<<cells.size() <<endl;
			assert(in_range_cell(c));
			return cells[index(c.x, c.y)];
		}

		cell_t &operator()(const Point_2d &p)
		{
			Point_2d c = to_cell_coordinates(p);
			assert(in_range_cell(c));
			return cells[index(c.x, c.y)];
		}

		bool is_accessible(const Point_2d &world_point) const
		{
			//cout << "[is_Accesible] start    point=" << world_point<<endl;
			if(not in_range(world_point)) {
				//cout << "[is_Accesible]            point not in range ==> accessible" <<endl;
				return true;
			}
			cell_t cell = (*this)(world_point);
			//cout <<std::hex<< "[is_Accesible]            cell = " << (int)cell <<std::dec<<endl;
			return is_accessible_value(cell);
		}

		void redraw_map(bool updated, const Point_2d & robot)
		{
			/* If map is unchanged and the robot is in an accessible cell, no need to remap */
			//if((not updated) and is_accessible(robot))
			cout << "[REDRAW]\tUpdated = " << updated << ", robot = " << robot << endl;
			if(not updated)
			{
				cout << "Is accessible: " << is_accessible(robot);
				if(is_accessible(robot))
					return;
			}

			/* Else, redraw the map */
			select_accessible_points(robot);
		}

		void select_accessible_points(const Point_2d &point)
		{
			Map &map = *this;
			for(size_t i = 0; i < cells.size(); i++) set_inaccessible_value(cells[i]);

			Point_2d cell = map.to_cell_coordinates(point);
			if(not map.in_range_cell(cell))return;

			queue<Point_2d> openlist;
			set<Index> closedlist;

			openlist.push(cell);
			closedlist.insert(map.index(cell));

			while(openlist.empty() == false) {
				Point_2d frontier = openlist.front();
				openlist.pop();

				set_accessible_value(map[index(frontier)]);

//			int dx[]={-1,0,1, 1, 1,0,-1, -1};
//			int dy[]={-1,-1,-1, 0, 1,1,1, 0};
//			int dn = 8;
				int dx[] = {-1, 0, 1, 0};
				int dy[] = {0, -1, 0, 1};
				int dn = 4;
				for(int i = 0; i < dn; i++) {
					Point_2d n(frontier.x + dx[i], frontier.y + dy[i]);

					if(not map.in_range_cell(n))continue;
					if(closedlist.find(map.index(n)) != closedlist.end()) continue;
					if(is_occupied_value(map[index(n)])) continue;

					openlist.push(n);
					closedlist.insert(map.index(n));
				}
			}
		}
	};
}

#endif /* NAVEX_NAVIGATION_INCLUDE_NAVEX_NAVIGATION_GOALCALCULATORMATH_H_ */
