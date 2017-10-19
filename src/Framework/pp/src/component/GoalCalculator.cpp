/*
 * GoalCalculator.cpp
 *
 *  Created on: Jul 19, 2017
 *      Author: assaf
 */

#include "GoalCalculator.h"

//std::ofstream log_goal_calc_decision("/tmp/log_calc_decision.txt");
#define LCD(...) do{}while(0);//log_goal_calc_decision << __VA_ARGS__ << std::endl;
#define SHOW_CURRENT_CELLMAP 0

#if SHOW_CURRENT_CELLMAP
#	include <opencv2/opencv.hpp>
#	include <opencv2/highgui/highgui.hpp>
#	include <opencv2/imgproc/imgproc.hpp>
#endif

using namespace std;

bool FStreamer::active = true;

namespace RobilGC
{
    

#	if SHOW_CURRENT_CELLMAP
    cv::Mat create_cv_image( const Map& map )
    {
		cv::Vec3b color_free, color_occupied, color_inaccessable_free;
		color_inaccessable_free[0]=color_inaccessable_free[1]=color_inaccessable_free[2]= 100;
		color_free[0]=color_free[1]=color_free[2]= 255;
		color_occupied[0]=color_occupied[1]=color_occupied[2]= 0;

		cv::Mat img( map.h, map.w, CV_8UC3, cv::Scalar::all(150));

		for(int y=0, index=0; y<img.rows; y++)
		for(int x=0; x<img.cols; x++, index++)
		{

			Map::cell_t cell = map.cells[index];

			if( Map::is_free_value(cell) and Map::is_accessible_value(cell)  )
			{
				img.at<cv::Vec3b>(y,x) = color_free;
			}
			else if( Map::is_free_value(cell) and not Map::is_accessible_value(cell) )
			{
				img.at<cv::Vec3b>(y,x) = color_inaccessable_free;
			}
			else
			{
				img.at<cv::Vec3b>(y,x) = color_occupied;
			}
		}

		return img;
    }

    void draw_point( cv::Mat& img, const Map& map, const Point_2d& point, cv::Scalar point_color, int point_size )
    {
    	Point_2d cell_coordinates = map.to_cell_coordinates(point);
    	cv::circle(img, cv::Point(cell_coordinates.x, cell_coordinates.y), point_size, point_color, -1);
    }
    
    void draw_line( cv::Mat& img, const Map& map, const Point_2d& point1, const Point_2d& point2, cv::Scalar point_color, int point_size, int line_size )
    {
    	Point_2d cell_coordinates1 = map.to_cell_coordinates(point1),cell_coordinates2 = map.to_cell_coordinates(point2) ;

    	cv::line(img, cv::Point(cell_coordinates1.x, cell_coordinates1.y), cv::Point(cell_coordinates2.x, cell_coordinates2.y), point_color, line_size);
    	cv::circle(img, cv::Point(cell_coordinates2.x, cell_coordinates2.y), point_size, point_color, -1);

    }
#	endif
    
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
			curr_log << map.w << " " << map.h << " " << map.origin.location << " " << map.origin.heading << " " << map.origin.resolution << std::endl;
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
            
            LCD("TEST: path[i="<<i<<"]="<<path[i]<<" is "<<(map.is_accessible(path[i])?"accessible":"not accessible"))
            
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
        LCD("------")
        LCD("Start get_goal ( robot="<<robot<<", goal="<<goal<<", waypoint="<<waypoint);
        LCD("   path is ")
        for(size_t i=0;i<path.size();i++){ LCD("       "<<i<<". "<<path[i]) }
        
		//THE PATH IS EMPTY
		if(path.empty())
        {
            LCD("path is empty => return false")
            LCD("return false")
            return false;
        }

		//THE PATH IS A SINGLE POINT
		if(path.size() == 1) {
            LCD("path is a single point => search nearest accessible point to path[0]="<<path[0])
			waypoint = 0;
			//SEARCH FREE POINT NEAR THE SINGLE POINT OF THE PATH
			bool res = search_accessible_point(map, path[0], goal);
            LCD("   res = "<<(res?"found":"not found")<<", new gola = "<<goal)
            
            LCD("return "<<(res?"true":"false"))
            return res;
		}

		//THE PATH HAS MORE THEN ONE POINT
        
        LCD("path has more then one point ("<<path.size()<<")")

//		SEARCH NEAREST POINT OF THE PATH TO THE ROBOT
//		Index nearest_point = search_nearest(path, robot);
		/* @assaf:
		 * This causes back-and-forth navigation if waypoints are not in order!
		 * That's why I take the maximum between the nearest index and the next waypoint in path.
		 */
		Index closest_point = search_nearest(path, robot);
        LCD("search_nearest(path,robot="<<robot<<") -> closest_point = "<<closest_point)
		if(closest_point > waypoint)
        {
			F_WARN("get_goal") << "Skipping waypoint #" << (waypoint + 1)
			<< " since there's a closer waypoint ahead." << endl;
        }
		Index nearest_point = max(waypoint, closest_point);
        LCD("max(waypoint="<<waypoint<<", closest_point="<<closest_point<<") -> nearest_point="<<nearest_point)

		//IF THE NEAREST POINT IS THE LAST POINT, THEN SEARCH FREE POINT NEAR IT.
		if(is_last_point(path, nearest_point)) 
        {
            LCD("the nearest point is the last point, so search free point near it (path[nearest_point]="<<path[nearest_point]<<") -> waypoint=nearest_point="<<nearest_point);
			waypoint = nearest_point;
			bool res = search_accessible_point(map, path[nearest_point], goal);
            LCD("   res = "<<(res?"found":"not found")<<", new gola = "<<goal)
            
            LCD("return "<<(res?"true":"false"))
            return res;
		}

		//IF THE NEAREST POINT IS NOT A LAST POINT
		//SEARCH FREE POINT FROM THE NEAREST TO THE PATH END
        
        LCD("nearest point is not the last point, so search free point from the nearest to the end of the path")
        LCD("   search_accessible_point(map, path, nearest_point="<<nearest_point<<", path.size()="<<path.size()<<", free_of_nearest_point=?, waypoint="<<waypoint<<")")
        
		Point_2d free_of_nearest_point;
		bool found = search_accessible_point(map, path, nearest_point, path.size(), free_of_nearest_point,
											 waypoint);
		F_DEBUG("get_goal") << "s_a_p returned: WayPoint " << (waypoint+1) << " @ " << free_of_nearest_point << endl;
        LCD("   result = "<<(found?"found":"not found")<<", free_of_nearest_point="<<free_of_nearest_point<<", waypoint="<<waypoint)

		//IF THE PATH IS BLOCKED UP TO THE END, SEARCH A NEAREST FREE POINT NEAR THE LAST POINT OF THE PATH
		if(not found) {
            
            LCD("if the path is blocked up to the end (accessible_point is not found), search a nearest free point near the last point of the path")
            
			F_WARN("get_goal") << "Path is blocked up to the end." << endl
				<< "Am I occupied? " << (map.is_occupied_value(map(robot)) ? "Yes!" : "No!") << endl;
                
			waypoint = path.size() - 1;
            
            LCD("waypoint=path.size()-1 -> waypoint="<<waypoint)
            
			bool bres = search_accessible_point(map, path[waypoint], goal);
			F_INFO("get_goal") << "Nearest free point near last point of path: " << goal << endl;
            
            LCD("search_accessible_point(map, path[waypoint]="<<path[waypoint]<<", goal="<<goal<<") -> "<<(bres?"found":"not found"))
            
            LCD("return "<<(bres?"true":"false"))
			return bres;
		}

		//IF SOME FREE POINT IS FOUND
        LCD("if some free point is found. free_of_nearest_point = "<<free_of_nearest_point)
		F_DEBUG("get_goal") << "Found free point @ " << free_of_nearest_point << endl;
        LCD("assert that not is_last_point(path=(size:"<<path.size()<<", waypoint="<<waypoint<<") and nearest_point="<<nearest_point<<" <= waypoint="<<waypoint)

        assert(not is_last_point(path, waypoint));
		assert(nearest_point <= waypoint);

		Index prev_point = waypoint;
        LCD("prev_point=waypoint="<<prev_point)
		if(path[prev_point] == free_of_nearest_point){ prev_point -= 1; LCD("path[prev_point] == free_of_nearest_point -> prev_point -= 1 -> prev_point="<<prev_point) }
		Index post_point = waypoint + 1;
        LCD("post_point = waypoint + 1 -> post_point="<<post_point)

		// IF THE FREE POINT IS PASSED, THEN GET NEXT WAYPOINT
        
        LCD("check if is_passed(path, robot="<<robot<<", prev_point="<<prev_point<<", free_of_nearest_point="<<free_of_nearest_point<<", post_point="<<post_point<<")")
        
		if(is_passed(path, robot, prev_point, free_of_nearest_point, post_point)) 
//		if(is_passed(free_of_nearest_point, robot))
		{
            LCD("   is_passed")
            
			F_DEBUG("get_goal") << "Point " << free_of_nearest_point << " is passed." << endl;
			//cout << "Free point is passed!" << endl;
			Index next_waypoint = waypoint + 1;

			//IF THE NEXT WAYPOINT IS THE LAST POINT OF THE PATH, JUST SEARCH FREE POINT NEAR IT
			if(is_last_point(path, next_waypoint)) {
                
                LCD("the next waypoint ("<<next_waypoint<<") is the last point of the path (size:"<<path.size()<<"), just search free point near it")
                
				waypoint = next_waypoint;
                LCD("waypoint = next_waypoint -> waypoint ="<<waypoint)
				bool res = search_accessible_point(map, path[next_waypoint], goal);
                LCD("search_accessible_point(map, path[next_waypoint]="<<path[next_waypoint]<<", goal="<<goal<<") -> "<<(res?"found":"not found")<<", gola="<<goal)

                LCD("return "<<(res?"true":"false"))
                return res;
			}

			//IF THE NEXT WAYPOINT IS NOT THE LAST POINT OF THE PATH, SEARCH FREE POINT TO BE A GOAL, FROM IT TO THE END OF THE PATH
            LCD("the next waypoint ("<<next_waypoint<<") is not the last point of the path (size:"<<path.size()<<"), search free point to be a goal, from it to the end of the path")
			
            bool found = search_accessible_point(map, path, next_waypoint, path.size(), goal, waypoint);
            
            LCD("search_accessible_point(map, path, next_waypoint, path.size(), goal="<<goal<<", waypoint="<<waypoint<<")"
                                                                                        <<(found?"found":"not found")<<", gola="<<goal<<", waypoint="<<waypoint)
			
            F_DEBUG("get_goal") << "Free point is passed, assigning new goal @ " << goal << "  (Index = " << waypoint << ")" << endl;
			//cout << "Assigning new goal: " << goal << "(WPI = " << waypoint << ")" << endl;

			//IF NEAREST ON PATH IS NOT FOUND, SEARCH A GENERAL NEAREST POINT
			if(not found) {
				bool found = search_accessible_point(map, path[path.size() - 1], goal);
                LCD("nearest on path is not found, search a general nearest point -> "
                        <<"search_accessible_point(map, path[path.size() - 1 = "<<path.size() - 1<<"]="<<path[path.size() - 1]<<", goal="<<goal<<") -> "
                                <<(found?"found":"not found")<<", gola="<<goal)
                                
                LCD("return "<<(found?"true":"false"))
				return found;
			}
            LCD("return true")
			return true;
		}
        
        LCD("   is not passed")

		goal = free_of_nearest_point;
        LCD("goal = free_of_nearest_point -> goal="<<goal)
        LCD("return true")
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

	void GoalCalculator::updateCellMap(bool include_data_update)
	{
		Point_2d origin(_world._map.info.origin.position.x,_world._map.info.origin.position.y);
		double resolution = _world._map.info.resolution, heading = quaternion_to_heading(_world._map.info.origin.orientation);

        if(include_data_update or not _cell_map)
        {
            _cell_map.reset(new Map(_world._map.info.width, _world._map.info.height, origin, heading, resolution, _world._robot, _world._heading));
        }
        else
        {
            _cell_map->origin = Pose_2d(origin, heading, resolution);
            _cell_map->world_pose = Pose_2d(_world._robot, _world._heading);
        }
        
        LCD("update: origin="<<origin<<", heading="<<heading<<", resolution="<<resolution<<", world_location="<<_world._robot<<", world_heading="<<_world._heading)

		Map & map = *_cell_map;
        
        if(include_data_update)
            for(size_t index=0; index<_world._map.data.size();index++)
            {
                if(_world._map.data[index] < _occupied_level)
                    map.set_free_value(map.cells[index]);
                else
                    map.set_occupied_value(map.cells[index]);
            }
        
	}
    
        bool Map::is_accessible(const Point_2d &world_point) const
		{
			LCD( "[is_Accesible] start    point=" << world_point )
			if(not in_range(world_point)) {
				LCD( "[is_Accesible]            point not in range ==> accessible" )
				return true;
			}
			cell_t cell = (*this)(world_point);
			LCD( std::hex<< "[is_Accesible]            cell = " << (int)cell <<std::dec )
			bool res = is_accessible_value(cell);
            LCD( "[is_Accesible]            is_accessible_value = " << (res?"yes":"no") )
            return res;
		}
        
        const Map::cell_t &Map::operator()(const Point_2d &p) const
		{
			Point_2d c = to_cell_coordinates(p);
			LCD( "[is_Accesible]            cell_coordinates = "<<c );
			assert(in_range_cell(c));
			return cells[index(c.x, c.y)];
		}
        

	bool GoalCalculator::updateMap(nav_msgs::OccupancyGrid & new_map, geometry_msgs::PoseWithCovarianceStamped & new_pose)
	{
		_world._robot = Point_2d(new_pose.pose.pose.position.x, new_pose.pose.pose.position.y);
		_world._heading = quaternion_to_heading(new_pose.pose.pose.orientation);
        
        _world._map.info = new_map.info;
        updateCellMap(false);
        
		size_t map_size = new_map.data.size();
		bool map_changed = (map_size != _world._map.data.size()) or (memcmp((void*)new_map.data.data(), (void*)_world._map.data.data(), map_size));
        
        LCD("updateMap : map_changed is "<<(map_changed?"changed":"not changed"))
        if(_cell_map)
            LCD("updateMap : cell of robot is "<<(_cell_map->is_accessible(_world._robot)?"accessible":"not accessible"))
        else
            LCD("updateMap : _cell_map is not created yet")

		if(map_changed or (not _cell_map->is_accessible(_world._robot)))
		{
            LCD("    update map")
			_world._map = new_map;
            
			updateCellMap(true);
			_cell_map->select_accessible_points(_world._robot);

			//_logger.log(*_cell_map, _world._robot, _path);
		}
        else
        {
            LCD("    skip map update")
        }

#		if SHOW_CURRENT_CELLMAP
        if(_cell_map)
        {
        	struct win_init_t{ win_init_t(){
        		cv::namedWindow("current_map", CV_WINDOW_NORMAL);
        	}};
        	static win_init_t _win_init;

			cv::Mat img_current_map = create_cv_image( *_cell_map );

			draw_point( img_current_map, *_cell_map, _world._robot, cv::Scalar(255,0,0), 5 );
			Point_2d ph(5, 0); ph.rotate(_world._heading); ph+=_world._robot;
			draw_line ( img_current_map, *_cell_map, _world._robot, ph, cv::Scalar(255,0,0), 1, 1 );

			if(_path.size()>0)
			{
				draw_point( img_current_map, *_cell_map, Point_2d(_path[0].x, _path[0].y), cv::Scalar(0,255,0), 5 );
			}
			if(_path.size()>1)
			{
				draw_line( img_current_map, *_cell_map, Point_2d(_path[0].x, _path[0].y), Point_2d(_path[1].x, _path[1].y), cv::Scalar(0,255,0), 3, 1 );
			}

			cv::flip(img_current_map, img_current_map, 0);

			cv::imshow("current_map", img_current_map);

			cv::waitKey(30);
        }
#		endif

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

		/* If goal is the last waypoint and is within close enough reach, send end signal */
		if(_wpi >= _path.size() - 1 and is_passed(_goal, _world._robot))
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
