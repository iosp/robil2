/*
 * GoalCalculator.cpp
 *
 *  Created on: Jul 19, 2017
 *      Author: assaf
 */

#include "GoalCalculator.h"

using namespace std;

bool FStreamer::active = true;

namespace RobilGC
{
	Config config;

	//=========================== LOGGER ====================================

	void Logger::init(int log_files)
	{
		_num_of_logs = log_files;

		/* Create directory */
		string dir_sys = "rm -rf " + GC_LOG_DIR + " || true";
		system(dir_sys.c_str());
		system(("mkdir " + GC_LOG_DIR).c_str());

		_current_log_file = -1;
	}

	void Logger::advance()
	{
		_current_log_file = (_current_log_file + 1) % _num_of_logs;
		stringstream curr_log_name;
		curr_log_name << GC_LOG_DIR << "/" << GC_LOG_PREFIX << dec << _current_log_file;
		_log = curr_log_name.str();
	}

	void Logger::log(const Map & map, const Point_2d & robot, Path & path)
	{
		if(_num_of_logs)
		{
			advance();
			ofstream curr_log;
			curr_log.open(_log.c_str(), ofstream::out);
			curr_log << ros::Time::now() << endl;
			curr_log << robot << endl;
			curr_log << path.size() << endl;
			for(size_t i = 0; i < path.size(); i++)
				curr_log << path[i] << endl;
			curr_log << map.w << " " << map.h << " " << map.offset << " " << map.heading << " " << map.resolution << endl;
			for(int i = 0; i < map.cells.size(); i++)
				curr_log << (int)map.cells[i] << endl;
			curr_log.close();
		}
	}

	//=========================== GOAL CALCULATOR GLOBAL FUNCTIONS ====================================

	bool is_last_point(const Path &path, const Point_2d &p)
	{
		return (path.back() - p).len() < config.same_point_eps;
	}

	bool is_last_point(const Path &path, const Index &p)
	{
		return Index(path.size() - 1) == p;
	}

	bool is_passed(
		INPUT
		const Path& path,
		const Point_2d& robot,
		const Point_2d* prev,
		const Point_2d& point,
		const Point_2d* post
	)
	{
		//cout<<"[["<<endl;
		if(not prev and not post)
		{
			//cout<<"  single point : "<<((point-robot).len())<<" < "<<(config.robot_is_near_distance)<<endl;
			return (point-robot).len() < config.robot_is_near_distance;
		}
		if(prev)
		{
			double a = (point- *prev).len();
			double b = (point- robot).len();
			double c = (robot- *prev).len();
			//cout<<"  prev point : a = "<<(a)<<", b = "<<(b)<<", c = "<<(c)<<" :: cc = "<<(c*c)<<" >=  aa+bb = "<<(a*a+b*b)<<endl;
			return c*c >= a*a+b*b;
		}
		if(post)
		{
			double a = (point- *post).len();
			double b = (point- robot).len();
			double c = (robot- *post).len();
			//cout<<"  post point : a = "<<(a)<<", b = "<<(b)<<", c = "<<(c)<<" :: cc = "<<(c*c)<<" <=  aa+bb = "<<(a*a+b*b)<<endl;
			return c*c <= a*a+b*b;
		}
		return true;
	}

	bool is_passed(
		INPUT
		const Path& path,
		const Point_2d& robot,
		const Index prev,
		const Point_2d& point,
		const Index post
	)
	{
		const Point_2d* _prev=  prev>=0?path.data()+prev:0;
		const Point_2d& _point= point;
		const Point_2d* _post=  post<(Index)path.size()?path.data()+post:0;
		return
				is_passed(path, robot, 0    , _point, _post) or
				is_passed(path, robot, _prev, _point, _post)
				;
	}

	bool is_passed(
		INPUT
		const Path& path,
		const Point_2d& robot,
		const Index prev,
		const Index point,
		const Index post
	)
	{
		return is_passed(path, robot, prev, path[point], post);
	}

	Index search_nearest(const Path &path, const Point_2d &robot)
	{
		assert(path.size() > 0);

		Index best = 0;
		Index best_d = (path[0] - robot).len();
		for(Index i = 0; i < (Index) path.size(); i++) {
			/*const Point2d* _prev = (i != 0) ? &(path[i-1]) : nullptr;
			const Point2d* _post = (i != path.size() - 1) ? &(path[i+1]) : nullptr;
			if(is_passed(path, robot, _prev, path[i], _post))
				continue;*/
			Index d = (path[i] - robot).len();
			if(d < best_d) {
				best = i;
				best_d = d;
			}
		}
		return best;
	}

	bool search_accessible_point(INPUT const Map &map, const Point_2d &b, const Point_2d &e,
								 OUTPUT    Point_2d &free_point)
	{
		Point_2d d = (e - b).norm();
		for(Point_2d c = b; (b - c).len() < (b - e).len(); c += d) {
			if(map.is_accessible(c)) {
				free_point = c;
				return true;
			}
		}
		return false;
	}

	bool search_accessible_point(INPUT const Map &map, const Path &path, const Index b, const Index e,
								 OUTPUT Point_2d &free_point, Index &waypoint)
	{
		assert (e - b >= 2 and in_range(b, 0, path.size() - 2) and in_range(e, 2, path.size()));
		for(Index i = b; i < e - 1; i++) {
			Index next = i + 1;
			if(map.is_accessible(path[i])) {
				free_point = path[i];
				waypoint = i;
				return true;
			}
			if(search_accessible_point(map, path[i], path[next], free_point)) {
				waypoint = i;
				return true;
			}
		}
		return false;
	}

	bool search_accessible_point(INPUT const Map &map, const Point_2d &point,
								 OUTPUT    Point_2d &free_point)
	{
		Point_2d cell = map.to_cell_coordinates(point);

		if(map.is_accessible_value(map[map.index(cell)])) {
			free_point = point;
			return true;
		}

		queue<Point_2d> openlist;
		set<Index> closedlist;
		Point_2d first_outrange_point = point;

		openlist.push(cell);
		closedlist.insert(map.index(cell));

		while(openlist.empty() == false) {
			Point_2d frontier = openlist.front();
			openlist.pop();

			if(map.is_accessible_value(map[map.index(frontier)])) {
				free_point = map.to_world_coordinates(frontier);
				return true;
			}

			int dx[] = {-1, 0, 1, 1, 1, 0, -1, -1};
			int dy[] = {-1, -1, -1, 0, 1, 1, 1, 0};
			int dn = 8;
			for(int i = 0; i < dn; i++) {
				Point_2d n(frontier.x + dx[i], frontier.y + dy[i]);

				if(not map.in_range_cell(n)) {
					if(first_outrange_point == point) first_outrange_point = map.to_world_coordinates(n);
					continue;
				}
				if(closedlist.find(map.index(n)) != closedlist.end()) continue;

				openlist.push(n);
				closedlist.insert(map.index(n));
			}
		}

		if(first_outrange_point != point) {
			free_point = first_outrange_point;
			return true;
		}
		return false;
	}

	/*
	 * Test definition of "is passed":
	 * Given the robot location, a point P is passed if the robot lies within P's bounding circle.
	 * Created to fix issues with paths of unordered waypoints.
	 */
	bool is_passed(const Point_2d & point, const Point_2d & robot)
	{
		// Radius of bounding circle
		static const double R = 2.0;

		return (robot - point).len() < R;
	}

	bool get_goal(INPUT    const Map &map, const Path &path, const Point_2d &robot,
				  OUTPUT Point_2d &goal, Index &waypoint)
	{
		//THE PATH IS EMPTY
		if(path.empty()) return false;

		//THE PATH IS A SINGLE POINT
		if(path.size() == 1) {
			waypoint = 0;
			//SEARCH FREE POINT NEAR THE SINGLE POINT OF THE PATH
			return search_accessible_point(map, path[0], goal);
		}

		//THE PATH HAS MORE THEN ONE POINT

//		SEARCH NEAREST POINT OF THE PATH TO THE ROBOT
//		Index nearest_point = search_nearest(path, robot);
		/* @assaf:
		 * This causes back-and-forth navigation if waypoints are not in order!
		 * That's why I take the maximum between the nearest index and the next waypoint in path.
		 */
		Index closest_point = search_nearest(path, robot);
		if(closest_point > waypoint)
			F_WARN("get_goal") << "Skipping waypoint #" << (waypoint + 1)
			<< " since there's a closer waypoint ahead." << endl;
		Index nearest_point = max(waypoint, closest_point);

		//IF THE NEAREST POINT IS THE LAST POINT, THEN SEARCH FREE POINT NEAR IT.
		if(is_last_point(path, nearest_point)) {
			waypoint = nearest_point;
			return search_accessible_point(map, path[nearest_point], goal);
		}

		//IF THE NEAREST POINT IS NOT A LAST POINT
		//SEARCH FREE POINT FROM THE NEAREST TO THE PATH END
		Point_2d free_of_nearest_point;
		bool found = search_accessible_point(map, path, nearest_point, path.size(), free_of_nearest_point,
											 waypoint);

		//IF THE PATH IS BLOCKED UP TO THE END, SEARCH A NEAREST FREE POINT NEAR THE LAST POINT OF THE PATH
		if(not found) {
			waypoint = path.size() - 1;
			return search_accessible_point(map, path[waypoint], goal);
		}

		//IF SOME FREE POINT IS FOUND
//		F_DEBUG("get_goal") << "Found free point @ " << free_of_nearest_point << endl;
		assert(not is_last_point(path, waypoint));
		assert(nearest_point <= waypoint);

		Index prev_point = waypoint;
		if(path[prev_point] == free_of_nearest_point) prev_point -= 1;
		Index post_point = waypoint + 1;

		// IF THE FREE POINT IS PASSED, THEN GET NEXT WAYPOINT
//		if(is_passed(path, robot, prev_point, free_of_nearest_point, post_point)) {
		if(is_passed(free_of_nearest_point, robot))
		{
			//cout << "Free point is passed!" << endl;
			Index next_waypoint = waypoint + 1;

			//IF THE NEXT WAYPOINT IS THE LAST POINT OF THE PATH, JUST SEARCH FREE POINT NEAR IT
			if(is_last_point(path, next_waypoint)) {
				waypoint = next_waypoint;
				return search_accessible_point(map, path[next_waypoint], goal);
			}

			//IF THE NEXT WAYPOINT IS NOT THE LAST POINT OF THE PATH, SEARCH FREE POINT TO BE A GOAL, FROM IT TO THE END OF THE PATH
			bool found = search_accessible_point(map, path, next_waypoint, path.size(), goal, waypoint);
//			F_DEBUG("get_goal") << "Free point is passed, assigning new goal @ " << goal << "  (Index = " << waypoint << ")" << endl;
			//cout << "Assigning new goal: " << goal << "(WPI = " << waypoint << ")" << endl;

			//IF NEAREST ON PATH IS NOT FOUND, SEARCH A GENERAL NEAREST POINT
			if(not found) {
				bool found = search_accessible_point(map, path[path.size() - 1], goal);
				return found;
			}
			return true;
		}

		goal = free_of_nearest_point;
		return true;
	}

	bool translate_goal(const Path & path, Point_2d & goal, const Index wpi, const Point_2d & robot)
	{
		static const double TRANSLATION_FACTOR = 1.4;

		/* If goal is an original waypoint, translate it */
		if(std::find(path.begin(), path.end(), goal) != path.end())
		{
			Point_2d diff_vector = path[wpi] - robot;
			Point_2d diff_norm = diff_vector.norm() * TRANSLATION_FACTOR;
			goal += diff_norm;
			return true;
		}

		// TODO Once IBEO topic is solved, check if translation is needed for obstacle cases as well.
		return false;
	}

	//=========================== INTERACTION WITH MOVEBASE ====================================

	void GoalCalculator::updateCellMap()
	{
		Point_2d origin(_world._map.info.origin.position.x,_world._map.info.origin.position.y);
		tf::Quaternion q;
		q.setW(_world._map.info.origin.orientation.w);
		q.setX(_world._map.info.origin.orientation.x);
		q.setY(_world._map.info.origin.orientation.y);
		q.setZ(_world._map.info.origin.orientation.z);
		double resolution = _world._map.info.resolution, heading = tf::getYaw(q);

		_cell_map.reset(new Map(_world._map.info.width, _world._map.info.height, origin, heading, resolution));
		Map & map = *_cell_map;

		for(int x = 0; x <map.w; x++)
		{
			for(int y = 0; y < map.h; y++)
			{
				size_t index = x + (map.w * y);
				double coor_x = map.offset.x + (x * map.resolution);
				double coor_y = map.offset.y + (y * map.resolution);

				// any value more than 50 is  for occupied cell
				if(_world._map.data[index] < 50)
					map.set_free_value(map(coor_x,coor_y));
				else
					map.set_occupied_value(map(coor_x,coor_y));
			}
		}
	}

	bool GoalCalculator::updateMap(nav_msgs::OccupancyGrid & new_map, geometry_msgs::PoseWithCovarianceStamped & new_pose)
	{
		_world._robot = Point_2d(new_pose.pose.pose.position.x, new_pose.pose.pose.position.y);
		size_t map_size = new_map.data.size();
		bool map_changed = (map_size != _world._map.data.size()) or (memcmp((void*)new_map.data.data(), (void*)_world._map.data.data(), map_size));

		if(map_changed or (not _cell_map->is_accessible(_world._robot)))
		{
			_world._map = new_map;
			updateCellMap();
			_cell_map->select_accessible_points(_world._robot);
			_logger.log(*_cell_map, _world._robot, _path);
		}

		return (map_size != 0);
	}

	void GoalCalculator::updatePath(nav_msgs::Path & gotten_path)
	{
		size_t path_size = gotten_path.poses.size();
		bool path_changed = false;

		if(path_size < _path.size())
			_path.clear();

		for(size_t i = 0; i < path_size; i++)
		{
			Point_2d p = Point_2d(gotten_path.poses[i].pose.position.x, gotten_path.poses[i].pose.position.y);
			if(i >= _path.size())
			{
				path_changed = true;
				_path.push_back(p);
			}
			else if(p != _path[i])
			{
				path_changed = true;
				_path[i] = p;
			}
		}

		if(path_changed)
		{
			F_WARN("GoalCalculator::updatePath") << "Path is changed!" << endl;
			F_WARN("GoalCalculator::updatePath") << "New path contains " << path_size << " waypoint(s)." << endl;
			_wpi = 0;
		}
	}

	bool GoalCalculator::get(OUTPUT geometry_msgs::PoseStamped & goal, Index & wpi)
	{
		Map& map = *_cell_map;

//		cout << "Original start: " << _world._robot << ", goal: " << _path[_wpi] << endl;
		Point_2d _goal = _path[_wpi];
		Index last_wp = _wpi;
		bool res = RobilGC::get_goal(map, _path, _world._robot, _goal, _wpi);
		if(_wpi > last_wp)
			F_DEBUG("GoalCalculator::get") << "Moving on to waypoint #" << (_wpi + 1) << " which is located @ " << _path[_wpi] << endl;
		wpi = _wpi;

		/* If goal is the last waypoint, send end signal */
		if(_wpi >= _path.size() - 1)
		{
			F_INFO("GoalCalculator::get") << "End of path!" << endl;
			_wpi = 0;
			return false;
		}

		/* Else, perform translation if needed */
//		cout << "Goal: " << _goal << "    [Index = " << wpi << "]";
		if(translate_goal(_path, _goal, _wpi, _world._robot))
			;//cout << "    --->    Translated to " << _goal << endl;

		/* Apply changes to original goal */
		goal.pose.position.x = _goal.x;
		goal.pose.position.y = _goal.y;

		return true;
	}
}
