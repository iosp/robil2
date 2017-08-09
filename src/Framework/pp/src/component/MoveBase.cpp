/*
 * MoveBase.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: dan
 */

#include "MoveBase.h"
#include <Geometry.h>

#include <move_base_msgs/MoveBaseActionGoal.h>
#include "ComponentMain.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <tf_geometry/tf_geometry.h>

#include <navex_msgs/VersionService.h>

#include <boost/date_time/posix_time/posix_time.hpp>

long f_counter=0;

#define CREATE_MAP_FOR_NAV 0
#define CREATE_POINTCLOUD2_FOR_NAV 0
#define CREATE_POINTCLOUD_FOR_NAV 1

#define TH_NEARBY 2.5 //m
//define duration of "finished" message for path, in order to consider it as truly finished.
#define SECONDS_FOR_FINISHING_AFTER_PATH_FINISHED 60

#include <opencv2/opencv.hpp>


#define SYNCH 	boost::recursive_mutex::scoped_lock locker(mtx);
#define REMOVE_Z(vA) vA.setZ(0)

#include <fstream>
#include <time.h>
ofstream dbg_cout("/tmp/pp_log");

#define DBG_INFO_ONCE( X ) ROS_INFO_STREAM_ONCE(X); dbg_cout <<"[I] "<< X << std::endl;
#define DBG_INFO( X ) ROS_INFO_STREAM(X); dbg_cout <<"[I] "<< X << std::endl;
#define DBG_ERROR( X ) ROS_ERROR_STREAM(X); dbg_cout <<"[E] "<< X << std::endl;
#define DBG_WARN( X ) ROS_WARN_STREAM(X); dbg_cout <<"[W] "<< X << std::endl;



namespace{

	//=========================== DEBUG FREQUENCY TIMER ====================================
	class FTimer
	{
		clock_t		_this_start;
		clock_t		_this_end;
		clock_t		_prev_start;
		clock_t		_prev_end;
		string		_func;
		double		_avg_rate;
		bool		_active;
		int			_freq;

		static double doublize(clock_t clock)
		{
			return ((double)clock) / CLOCKS_PER_SEC;
		}

		void print(clock_t Ta, clock_t Tb, clock_t Tc)
		{
			cout << "\033[1;33m" << endl;

			cout << " ⌛ " << "FTimer for \"" << _func << "\":\033[0;33m" << endl;
			cout << "   " << "Ta = " << doublize(Ta) << "    Tc = " << doublize(Tc) << "    (Rate = " <<
					100.0 * doublize(Ta) / doublize(Tc) << "%)" << endl;
			//cout << "   " << "Rate = " << doublize(Ta) / doublize(Tc) << "  (Avg. " << _avg_rate << ")" << endl;

			cout << "\033[0m" << endl;
		}

		void update_timer()
		{
			clock_t Ta = _this_end - _this_start;
			clock_t Tb = _this_start - _prev_end;
			clock_t Tc = _this_end - _prev_end;

			static int n = 1;
			double xn1 = (doublize(Ta) / doublize(Tc)) - _avg_rate;

			if(_active)
			{
				n++;
				_avg_rate += (xn1 / n);
				if(n % _freq == 0)
					print(Ta, Tb, Tc);
			}

			_prev_start = _this_start;
			_prev_end = _this_end;
			if(not _active)
				_active = true;
		}

	public:
		FTimer(const string & funcId, int frequency = 1):_func(funcId),_active(false),_avg_rate(0.0), _freq(frequency){}

		void start()
		{
			_this_start = clock();
		}

		void stop()
		{
			_this_end = clock();
			update_timer();
		}
	};

	long goal_counter =0;

	template<int N> double fround(double x){
		static const double k = ::pow10(N);
		return round(x*k)/k;
	}


	//=========================== SEARCH NEXT WAIPOINT : BGN ====================================

	void remove_orientation(geometry_msgs::PoseStamped& goal){
		goal.pose.orientation.x = 0;
		goal.pose.orientation.y = 0;
		goal.pose.orientation.z = 0;
		goal.pose.orientation.w = 1;
	}
	double calcTriangle_deg(
			const geometry_msgs::Pose& A,
			const geometry_msgs::Pose& C,
			const geometry_msgs::Pose& B
	){
		btVector3 vA = toVector(A); REMOVE_Z(vA);
		btVector3 vB = toVector(B); REMOVE_Z(vB);
		btVector3 vC = toVector(C); REMOVE_Z(vC);
		double a = vC.distance(vB);//CB
		double c = vA.distance(vB);//AB
		double b = vA.distance(vC);//AC
		double gamma;// \_ ACB
		gamma = acos( (a*a + b*b - c*c)/(2*a*b) );
		return angles::to_degrees( angles::normalize_angle_positive( gamma ) );
	}
	size_t search_nearest_waypoint_index(const nav_msgs::Path& path, const geometry_msgs::PoseWithCovarianceStamped& pos, size_t start_index){
		btVector3 vPose = toVector(getPose(pos)); REMOVE_Z(vPose);
		double min_idx=start_index;
		btVector3 path_poses_0 = toVector(getPose(path.poses[min_idx])); REMOVE_Z(path_poses_0);
		double min_distance=path_poses_0.distance(vPose);
		for(size_t i=min_idx;i<path.poses.size();i++){
			btVector3 path_poses_i = toVector(getPose(path.poses[i])); REMOVE_Z(path_poses_i);
			double c_distance=path_poses_i.distance(vPose);
			if(c_distance<min_distance){
				min_distance = c_distance;
				min_idx = i;
			}
		}
		return min_idx;
	}

	std::string ex(long n, int c)
	{
		std::stringstream s; s<<n;
		std::string ss=s.str();
		while(ss.size()<c) ss = std::string("0")+ss;
		return ss;
	}

	geometry_msgs::PoseStamped search_next_waypoint(const nav_msgs::Path& path, size_t start_index, const geometry_msgs::PoseWithCovarianceStamped& pos, bool& path_is_finished, int& goal_res_index){

		static std::ofstream position_log("/tmp/pp_position.log");

		//cout<<"search_next_waypoint for: "<<pos.pose.pose.position.x<<","<<pos.pose.pose.position.y<<endl;
		if(path.poses.size()==0){
			path_is_finished = true;
			goal_res_index = -1;
			geometry_msgs::PoseStamped p = getPoseStamped( pos ); p.header = pos.header;
			return p;
		}
		if(path.poses.size()==1){
			geometry_msgs::PoseStamped my_pose = getPoseStamped( pos ); my_pose.header = pos.header;
			geometry_msgs::PoseStamped path_pose = getPoseStamped( path.poses[0] ); path_pose.header = path.header;
			btVector3 vpath_pose_pose = toVector(path_pose.pose); REMOVE_Z(vpath_pose_pose);
			btVector3 vmy_pose_pose = toVector(my_pose.pose); REMOVE_Z(vmy_pose_pose);
			path_is_finished = vpath_pose_pose.distance(vmy_pose_pose) <= TH_NEARBY;
			goal_res_index = 0;
			return path_pose;
		}
		path_is_finished = false;
		size_t ni = search_nearest_waypoint_index(path , pos, start_index);

		position_log<<"NEAREST POINT["<<ni<<"]  TH_NEARBY="<<TH_NEARBY<<" robot frame="<<pos.header.frame_id<<", path frame="<<path.header.frame_id<<std::endl;

//		static cv::Mat www; double f=10;
//		www = cv::Mat(1000,1000,CV_8UC3,cv::Scalar::all(200));
//		cv::namedWindow("WWW", CV_WINDOW_NORMAL);
//		cv::startWindowThread();
//		static long img_n=0; img_n++;
//		std::stringstream img_name; img_name<<"/tmp/IMG/img_"<<ex(img_n,5)<<".png";





		//cout<<"[i] nearest index = "<<ni<<endl;
		const geometry_msgs::Pose& c = getPose( pos );
		static cv::Mat www = cv::Mat(1000,1000,CV_8UC3,cv::Scalar::all(200));
		//FIRST POINT
		if(ni==0){
			const geometry_msgs::Pose& b = getPose( path.poses[0] );
			const geometry_msgs::Pose& d = getPose( path.poses[1] );
			double angle_deg = calcTriangle_deg(d,b,c);
			bool angle_ok = angle_deg<=90;
			bool distance_ok = (toVector(b)-toVector(c)).length() <= TH_NEARBY;

			position_log<<"POINT["<<0<<"]"
					<<": robot="<<c.position.x<<","<<c.position.y
					<<", goal="<<b.position.x<<","<<b.position.y
					<<", distance="<<((toVector(b)-toVector(c)).length())<<"("<<(distance_ok?"ok":"bad")<<")"
					<<", angle[deg]="<<angle_deg<<"("<<(angle_ok?"ok":"bad")<<")"
			<<std::endl;

			double f=10;
//			cv::line(www, cv::Point(b.position.x*f+www.cols/2,b.position.y*f+www.rows/2), cv::Point(d.position.x*f+www.cols/2,d.position.y*f+www.rows/2), cv::Scalar(0,100,100), 1);
//			cv::line(www, cv::Point(b.position.x*f+www.cols/2,b.position.y*f+www.rows/2), cv::Point(c.position.x*f+www.cols/2,c.position.y*f+www.rows/2), cv::Scalar(0,100,100), 1);
//			cv::circle(www, cv::Point(c.position.x*f+www.cols/2,c.position.y*f+www.rows/2), 5, cv::Scalar(0,0,255), 2);
//			cv::circle(www, cv::Point(b.position.x*f+www.cols/2,b.position.y*f+www.rows/2), 5, cv::Scalar(0,255,0), 2);
//			cv::circle(www, cv::Point(d.position.x*f+www.cols/2,d.position.y*f+www.rows/2), 5, cv::Scalar(0,255,255), 2);+9
//////			cv::imshow("WWW",www);
//////			cv::waitKey(1);
//			cv::imwrite(img_name.str(), www);

			if(not (angle_ok or distance_ok) ){
				//cout<<"[i] return index = 0"<<endl;
				tf_geometry::Position p1(path.poses[0].pose.position);
				tf_geometry::Position p2(path.poses[1].pose.position);
				tf_geometry::Position res = p1+((p2-p1).normalized()*0.5);
				geometry_msgs::PoseStamped pose_res=path.poses[0]; pose_res.header = path.header;
				pose_res.pose.position = res.to_msg_Point();
				goal_res_index = 0;
				geometry_msgs::PoseStamped p = getPoseStamped( pose_res ); p.header = pose_res.header;
				return p;
//				return getPoseStamped( path.poses[0] );
			}
			ni+=1;
		}
		//OTHERS POINTS
		for(size_t i=ni;i<path.poses.size();i++){
			const geometry_msgs::Pose& a = getPose( path.poses[i-1] );
			const geometry_msgs::Pose& b = getPose( path.poses[i] );
			double angle_deg = calcTriangle_deg(a,b,c);
			bool angle_ok = angle_deg>=90;
			bool distance_ok = (toVector(b)-toVector(c)).length() <= TH_NEARBY;

			position_log<<"POINT["<<i<<"]"
					<<": robot="<<c.position.x<<","<<c.position.y
					<<", goal="<<b.position.x<<","<<b.position.y
					<<", distance="<<((toVector(b)-toVector(c)).length())<<"("<<(distance_ok?"ok":"bad")<<")"
					<<", angle[deg]="<<angle_deg<<"("<<(angle_ok?"ok":"bad")<<")"
			<<std::endl;

//			cv::line(www, cv::Point(b.position.x*f+www.cols/2,b.position.y*f+www.rows/2), cv::Point(a.position.x*f+www.cols/2,a.position.y*f+www.rows/2), cv::Scalar(100,100,2), 1);
//			cv::line(www, cv::Point(b.position.x*f+www.cols/2,b.position.y*f+www.rows/2), cv::Point(c.position.x*f+www.cols/2,c.position.y*f+www.rows/2), cv::Scalar(100,100,2), 1);
//			cv::circle(www, cv::Point(c.position.x*f+www.cols/2,c.position.y*f+www.rows/2), 5, cv::Scalar(0,0,255), 2);
//			cv::circle(www, cv::Point(b.position.x*f+www.cols/2,b.position.y*f+www.rows/2), 5, cv::Scalar(0,255,0), 2);
//			cv::circle(www, cv::Point(a.position.x*f+www.cols/2,a.position.y*f+www.rows/2), 5, cv::Scalar(0,255,255), 2);
//////			cv::imshow("WWW",www);
//////			cv::waitKey(1);
//			cv::imwrite(img_name.str(), www);

			if(not (angle_ok or distance_ok) ){
				//cout<<"[i] return index = "<< i <<endl;
				if(i==path.poses.size()-1){
					geometry_msgs::PoseStamped my_pose = getPoseStamped( pos ); my_pose.header = pos.header;
					geometry_msgs::PoseStamped path_pose = getPoseStamped( path.poses[i] ); path_pose.header = path.header;
					path_is_finished = toVector(path_pose.pose).distance(toVector(my_pose.pose)) <= TH_NEARBY;
				}
				if(i==path.poses.size()-1){
					goal_res_index = i;
					geometry_msgs::PoseStamped p = getPoseStamped( path.poses[i] ); p.header = path.header;
					return p;
				}
				tf_geometry::Position p1(path.poses[i-1].pose.position);
				tf_geometry::Position p2(path.poses[i].pose.position);
				tf_geometry::Position res = p2+((p2-p1).normalized()*0.5);
				geometry_msgs::PoseStamped pose_res=path.poses[i];pose_res.header = path.header;
				pose_res.pose.position = res.to_msg_Point();
				goal_res_index = i;
				geometry_msgs::PoseStamped p = getPoseStamped( pose_res );p.header = pose_res.header;
				return p;
			}
		}

		position_log << "PATH IS FINISHED" <<std::endl;

		//PATH IS FINISHED
		path_is_finished = true;
		goal_res_index = -2;
		geometry_msgs::PoseStamped p =  getPoseStamped( pos ); p.header = pos.header;
		return p;
		//return getPoseStamped( path.poses[path.poses.size()-1] );
	}

	//=========================== SEARCH NEXT WAIPOINT : END ====================================

	//=========================== MAKE GOAL REACHABLE : BGN =====================================

	struct point_t{
		int x,y;
		point_t():x(0),y(0){}
		point_t(int x,int y):x(x),y(y){}
		int index(int w,int h)const{return y*w+x;}
		bool inside(int w,int h)const{ return 0<=x and x<w and 0<=y and y<h; }
		std::string str()const{
		  std::stringstream s; s<<"("<<x<<","<<y<<")";
		  return s.str();
		}
	};

	template<class T, class H>
	inline std::string str_position(const T& p, const H& h)
	{
		std::stringstream s; s<<"("<<p.x<<", "<<p.y<<")["<<h.header.frame_id<<":"<<h.header.stamp<<"]";
		return s.str();
	}

	/*[OLD VERSION]*/
//	inline
//	point_t get_point_on_grid(const geometry_msgs::Point& p, double resolution, const point_t& center, const point_t& robot)
//	{
//		return point_t(p.x/resolution - robot.x + center.x, p.y/resolution - robot.y + center.y );
//	}

	/*[NEW VERSION]{*/
	inline
	tf::TransformListener& tfListener(){ static tf::TransformListener l; return l; }
	inline std::string str_dbg_location(std::string f, int l){ std::stringstream s; s<<f<<":"<<l; return s.str(); }

	geometry_msgs::PointStamped transform_to_frame(const geometry_msgs::PointStamped& point, const std::string& target_frame, const std::string& dbg_location="")
	{
		if(target_frame == point.header.frame_id) return point;
		geometry_msgs::PointStamped res;

		try {
			tfListener().waitForTransform(target_frame, point.header.frame_id, point.header.stamp, ros::Duration(5.0));
			tfListener().transformPoint(target_frame, point, res);
			res.header.frame_id = target_frame ;
			return res;
		}
		catch (tf::TransformException& exception) {
			std::string dbgl = dbg_location;
			if(dbg_location == ""){ dbgl = str_dbg_location(__FILE__,__LINE__); }
			ROS_ERROR("Navigation: Failed to transform point: \n%s [%s]", exception.what(), dbgl.c_str());
			throw;
		}
	}
	geometry_msgs::PoseStamped transform_to_frame(const geometry_msgs::PoseStamped& point, const std::string& target_frame, const std::string& dbg_location="")
	{
		if(target_frame == point.header.frame_id) return point;
		geometry_msgs::PoseStamped res;

		try {
			tfListener().waitForTransform(target_frame, point.header.frame_id, point.header.stamp, ros::Duration(5.0));
			tfListener().transformPose(target_frame, point, res);
			res.header.frame_id = target_frame ;
			return res;
		}
		catch (tf::TransformException& exception) {
			std::string dbgl = dbg_location;
			if(dbg_location == ""){ dbgl = str_dbg_location(__FILE__,__LINE__); }
			ROS_ERROR("Navigation: Failed to transform pose: \n%s [%s]", exception.what(), dbgl.c_str());
			throw;
		}
	}
	geometry_msgs::PointStamped transform_to_map_frame(const geometry_msgs::PointStamped& point, const nav_msgs::OccupancyGrid& map)
	{
		return transform_to_frame(point, map.header.frame_id, str_dbg_location(__FILE__,__LINE__));
	}

	inline
	point_t get_point_on_grid(const geometry_msgs::PointStamped& p, const nav_msgs::OccupancyGrid& map)
	{
		geometry_msgs::PointStamped res;
		res = transform_to_map_frame(p, map);
		point_t r (
				(res.point.x-map.info.origin.position.x)/map.info.resolution,
				(res.point.y-map.info.origin.position.y)/map.info.resolution
				);
		//DBG_INFO("TRANSFORM: "<<str_position(p.point, p)<<" -> "<<str_position(res.point,res)<<" -> "<<r.str()<<"[GRID]")
		return r;
	}
	inline
	point_t get_point_on_grid(const geometry_msgs::Point& p, const nav_msgs::OccupancyGrid& map)
	{
		geometry_msgs::PointStamped ps;
		ps.header.frame_id = "WORLD";
		ps.header.stamp = map.header.stamp;
		ps.point = p;
		return get_point_on_grid(ps, map);
	}
	inline
	point_t get_point_on_grid(const geometry_msgs::Point& p, const std_msgs::Header& point_header, const nav_msgs::OccupancyGrid& map)
	{
		geometry_msgs::PointStamped ps;
		ps.header = point_header;
		ps.point = p;
		return get_point_on_grid(ps, map);
	}

	inline
	geometry_msgs::PointStamped get_point_on_grid(const point_t& p, const nav_msgs::OccupancyGrid& map)
	{
		geometry_msgs::PointStamped res;
		res.point.x = p.x*map.info.resolution+map.info.origin.position.x;
		res.point.y = p.y*map.info.resolution+map.info.origin.position.y;
		res.header.frame_id = map.header.frame_id;
		res.header.stamp = map.header.stamp;
		geometry_msgs::PointStamped r = transform_to_frame(res, "WORLD");
		DBG_INFO("TRANSFORM: "<<p.str()<<"[GRID] -> "<<str_position(res.point,res)<<" -> "<<str_position(r.point,r));
		return r;
	}
	inline
	geometry_msgs::PointStamped get_point_on_grid(const point_t& p, const nav_msgs::OccupancyGrid& map, const std_msgs::Header& point_header)
	{
		geometry_msgs::PointStamped res;
		res.point.x = p.x*map.info.resolution+map.info.origin.position.x;
		res.point.y = p.y*map.info.resolution+map.info.origin.position.y;
		res.header.frame_id = map.header.frame_id;
		res.header.stamp = map.header.stamp;
		geometry_msgs::PointStamped r = transform_to_frame(res, point_header.frame_id);
		DBG_INFO("TRANSFORM: "<<p.str()<<"[GRID] -> "<<str_position(res.point,res)<<" -> "<<str_position(r.point,r));
		return r;
	}
	/*}[NEW VERSION]*/

	inline
	/*[OLD VERSION]*/
	//bool on_way(const nav_msgs::Path& path, int goal_index, const point_t& point, double dist, double resolution, const point_t& center, const point_t& robot)
	/*[NEW VERSION]*/
	bool on_way(const nav_msgs::Path& path, int goal_index, const point_t& point, double dist, const nav_msgs::OccupancyGrid& map, const point_t& center, const point_t& robot)
	{
		point_t p1(0,0);
		point_t p2(0,0);
		while(goal_index < path.poses.size()){
			if(goal_index < path.poses.size()-1){
/*[OLD VERSION]{*/
//				p1 = get_point_on_grid(path.poses[goal_index + 0].pose.position, resolution, center, robot );
//				p2 = get_point_on_grid(path.poses[goal_index + 1].pose.position, resolution, center, robot );
//			}else{
//				p1 = get_point_on_grid(path.poses[goal_index - 1].pose.position, resolution, center, robot );
//				p2 = get_point_on_grid(path.poses[goal_index + 0].pose.position, resolution, center, robot );
/*}[OLD VERSION]*/
/*[NEW VERSION]{*/
				p1 = get_point_on_grid(path.poses[goal_index + 0].pose.position, map );
				p2 = get_point_on_grid(path.poses[goal_index + 1].pose.position, map );
			}else{
				p1 = get_point_on_grid(path.poses[goal_index - 1].pose.position, map );
				p2 = get_point_on_grid(path.poses[goal_index + 0].pose.position, map );
/*}[NEW VERSION]*/
			}
			double c = hypot(p1.x - point.x, p1.y - point.y);
			double b = hypot(p2.x - point.x, p2.y - point.y);
			double a = hypot(p1.x - p2.x, p1.y - p2.y);
			double p = (a + b + c)/2.0;
			double h = 2.0*sqrt(p * (p-a) * (p-b) * (p-c))/a;
			double a1 = sqrt(b*b - h*h);
			double a2 = sqrt(c*c - h*h);
			double A = a1 + a2;
			bool on_line = h <= dist;
			bool in_segment = fabs(A-a) < 0.001;

			if( on_line and in_segment ) return true;

			goal_index ++;
		}
		return false;
	}

	template<class T>
	struct array_manager{
		T* &p;
		array_manager(T*&p, size_t data_size):p(p){
			p = new T[data_size];
			memset(p,0,data_size*sizeof(T));
		}
		~array_manager(){ delete[] p; p=0; }
	};

	struct points_pool{
		point_t* next;
		point_t* r;
		point_t* w;

		static point_t* & points(){ static point_t* v(0); return v; }
		static size_t& points_size(){ static size_t s(0); return s; }

		points_pool(size_t size)
			: next( 0 ) //(point_t*)(new char[sizeof(point_t)*size])
			, r(0)
			, w(0)
		{
			if(points_size()<size)
			{
				if(points_size()>0){ delete[] points(); }
				//points() = (point_t*)(new char[sizeof(point_t)*size]); //Note: the allocation in such way prevent execution of constructor.
				points() = new point_t[size];
				points_size()=size;
			}
			r=w=next = points();
			memset(next,0,size*sizeof(point_t));
		}
		~points_pool()
		{
			//delete[] next; next=0;
		}
		void push_back(const point_t& p){ *w = p; w++; }
		const point_t& front()const{ return *r; }
		void pop_front(){ r++; }
		bool empty()const{ return not( r<w ); }
	};

	inline
	bool search_nearest_reachable_point(
			const size_t data_size,
			const size_t w, const size_t h,
			const point_t& goal_cell,
			const nav_msgs::Path& path,
			const size_t& goal_index,
			const nav_msgs::OccupancyGrid& global_map,
			const point_t& center, const point_t& _robot,
			const char* reachable,
			const bool check_on_way,
			point_t& best
	)
	{
		bool* search_visited;
		array_manager<bool> sfc(search_visited, data_size);
		points_pool next_for_search(w*h);
		point_t current(0,0);

		next_for_search.push_back(goal_cell);
		while( not next_for_search.empty() ){

			const char REACHABLE=1;
			const char UNREACHABLE=2;
			const char UNKNOWN=0;

			current = next_for_search.front();
			next_for_search.pop_front();

			//FOR ALL NAIGHBORS
			for(int iy=-1;iy<=1;iy++)for(int ix=-1;ix<=1;ix++){
				if( ix==0 and iy==0 ) 						continue;
				point_t nei(current.x+ix,current.y+iy);

				if( nei.inside(w, h) == false ) 			continue;
				if( search_visited[ nei.index(w, h) ] ) 	continue;

				if( check_on_way ){
					bool _on_way =
							on_way( path, goal_index, nei, 1, global_map, center, _robot  );
					if( not _on_way )						continue;
				}

				if( reachable[ nei.index(w, h) ] == REACHABLE ){
					best = nei;
					return true;
				}

				search_visited[ nei.index(w, h) ] = true;
				next_for_search.push_back(nei);
			}
		}
		return false;
	}

	bool make_goal_reachable(
		geometry_msgs::PoseStamped& goal, int& goal_index,
		const nav_msgs::Path& path,
		const geometry_msgs::PoseWithCovarianceStamped& pos,
		const nav_msgs::OccupancyGrid& global_map,
		bool& path_is_finished
	)
	{
#		define SHOW_CV_RESULTS 0
		bool result(false);
		int data_size =global_map.data.size();
		int w = global_map.info.width;
		int h = global_map.info.height;

		//NOTE: don't use vector<T> or list<T> for "reachable" and "next" arrays;
		//      them implementation is too slow.

		char* reachable;
		array_manager<char> rc(reachable, data_size);
		points_pool next(w*h);

		/*[PREV_VERSION]*/
		//point_t _robot(pos.pose.pose.position.x/global_map.info.resolution, pos.pose.pose.position.y/global_map.info.resolution);

		point_t center(w/2,h/2);

		/*[NEW VERISON]*/
		point_t _robot = center;

		/*[PREV_VERSION]*/
		//point_t goal_cell = get_point_on_grid(goal.pose.position, global_map.info.resolution, center, _robot);

		/*[NEW VERISON]*/
		point_t goal_cell = get_point_on_grid(goal.pose.position,goal.header, global_map);
		
		DBG_INFO_ONCE("Navigation: cells: robot="<<_robot.str()<<", center="<<center.str()<<", goal="<<goal_cell.str() << "[GRID]: pos="<<str_position(pos.pose.pose.position,pos)<<"; goal="<<str_position(goal.pose.position,goal)<<" r="<<global_map.info.resolution);

		try{
			{
				const char REACHABLE=1;
				const char UNREACHABLE=2;
				const char UNKNOWN=0;
				reachable[center.index(w,h)] = REACHABLE;																					// select all reachable cells
				next.push_back(center);
				while(next.empty()==false){
					point_t current = next.front(); next.pop_front();
					//FOR ALL NAIGHBORS
					for(int iy=-1;iy<=1;iy++)for(int ix=-1;ix<=1;ix++){
						if( ix==0 and iy==0 ) 										continue;

						point_t nei(current.x+ix,current.y+iy);

						if/*outside of map*/	(nei.inside(w,h)==false) 			continue;
						if/*already checked*/	(reachable[nei.index(w,h)]>UNKNOWN)	continue;

						reachable[nei.index(w,h)] = UNREACHABLE;
						if/*occupied*/		(global_map.data[nei.index(w,h)]>70) 	continue;
						/*else*/
						reachable[nei.index(w,h)] = REACHABLE;
						next.push_back(nei);
					}
				}
				for(size_t i=0; i<data_size; i++) if(reachable[i]==UNKNOWN) reachable[i]=UNREACHABLE;
			}
			for(int y=0;y<h;y++)for(int x=0;x<w;x++){
				point_t current = point_t(x,y);
				bool v = global_map.data[current.index(w,h)]>70;/*is occupied*/
				if(v){
					for(int iy=-1;iy<=1;iy++)for(int ix=-1;ix<=1;ix++){
						point_t nei(current.x+ix,current.y+iy);
						if/*outside of map*/	(nei.inside(w,h)==false) 			continue;
						reachable[nei.index(w,h)] = false;
					}
				}
			}
#			if SHOW_CV_RESULTS==1
				static cv::Mat show; 
				static ros::NodeHandle pnode("~");
				static int param_show_cv_results = 0;
				pnode.getParamCached("show_cv_results",param_show_cv_results);
				if(param_show_cv_results)
				{

				    DBG_INFO("Navigation Visualization: update show matrix");
				    show = cv::Mat(h,w, CV_8UC3);
				    for(int y=0;y<h;y++)for(int x=0;x<w;x++){
					    uchar v = reachable[point_t(x,y).index(w,h)] ? 255 : 150;
					    show.at<cv::Vec3b>(h-y-1,x) = cv::Vec3b(v,v,v);
				    }
				    show.at<cv::Vec3b>(h-center.y-1,center.x) = cv::Vec3b(255,0,0);
				    show.at<cv::Vec3b>(h-goal_cell.y-1,goal_cell.x) = cv::Vec3b(0,0,255);
				    DBG_INFO("Navigation Visualization: update show matrix. done");
				}
#			endif

			bool is_reachable = goal_cell.inside(w,h) and reachable[goal_cell.index(w,h)];

			//DBG_INFO("Navigation: is_reachable = "<<(is_reachable?"true":"false"));

			if( not is_reachable ){																									// if the cell is not reachable then
				point_t best(0,0);

				bool best_found =
						search_nearest_reachable_point(
								data_size,
								w, h,
								goal_cell,
								path,
								goal_index,
								global_map,
								center, _robot,
								reachable,
								true,

								best
						);

				//DBG_INFO("Navigation: best_found = "<<(best_found?"true":"false"));
				//If all path is blocked, select just available and nearest to the last goal point
				if( best_found == false ){

					/*[PREV VERSION]*/
					//goal_cell = get_point_on_grid( path.poses.back().pose.position, global_map.info.resolution, center, _robot);
					/*[NEW VERSION]*/
					goal_cell = get_point_on_grid( path.poses.back().pose.position, path.header, global_map);
					best_found =
						search_nearest_reachable_point(
							data_size,
							w, h,
							goal_cell,
							path,
							goal_index,
							global_map,
							center, _robot,
							reachable,
							false,

							best
					);

				}

#				if SHOW_CV_RESULTS==1
				if(param_show_cv_results)
				{
					show.at<cv::Vec3b>(h-goal_cell.y-1,goal_cell.x) = cv::Vec3b(100,255,200);
				}
#				endif

				if( best_found == false ){
					//DBG_INFO( "[E] Navigation: [e] Cann't find best ");
					return false;
				}

				goal.pose.position = get_point_on_grid(best, global_map, goal.header).point;
				DBG_INFO_ONCE("Best: " << best.str() << "[GRID].     Goal: " << str_position(goal.pose.position,goal));

#				if SHOW_CV_RESULTS==1
				if(param_show_cv_results)
				{
					//show.at<cv::Vec3b>(h-best.y-1,best.x) = cv::Vec3b(255,0,0);
					cv::circle(show, cv::Point(best.x,h-best.y-1), 5, cv::Scalar(255,0,0), 1);
				}
#				endif

				result = true;
			}
			else { result = false; }																								// else return false

#			if SHOW_CV_RESULTS==1
			if(param_show_cv_results)
			{
				DBG_INFO("Navigation Visualization: rendering");
				//if(show.empty()) cv::namedWindow("REACHABLE", CV_WINDOW_NORMAL);
				//cv::imshow("REACHABLE",show);
				//cv::waitKey(10);
				static long img_n=0; img_n++;
				std::stringstream img_name; img_name<<"/tmp/IMG/img_"<<ex(img_n,5)<<"_"<<f_counter<<".png";
				cv::imwrite(img_name.str(), show);
				DBG_INFO("Navigation Visualization: rendering. done");
			}
#			endif

		}catch (...) {
			DBG_INFO_ONCE( "[E] Navigation: [e] exception during "<<__FUNCTION__ );
		}
		return result;
	}

	//=========================== MAKE GOAL REACHABLE : END =====================================


	boost::thread_group threads;
	actionlib_msgs::GoalID last_nav_goal_id;
	move_base_msgs::MoveBaseActionGoal last_nav_goal;
	nav_msgs::Path curr_nav_path;
	bool clear_path_on_activate=true;

	boost::mutex global_map_mutex;
	nav_msgs::OccupancyGrid global_cost_map;
}


void on_GlobalCostMap( const nav_msgs::OccupancyGrid::ConstPtr& cost_map){
	boost::mutex::scoped_lock l(global_map_mutex);
	global_cost_map = *cost_map;

	global_cost_map.data[rand() % 255] = rand() % 255;
	DBG_INFO("Navigation: Global occupancy cost grid gotten");
}
void on_speed(const geometry_msgs::Twist::ConstPtr& msg){
	bool moving = msg->linear.x + msg->linear.y + msg->angular.z;
	if(not moving){
		//DBG_INFO( "Navigation: ROBOTE IS STOPPED" );
	}

	struct SurrealSpeed
	{
		double	_x, _z;
		double	_eps;
		SurrealSpeed(double x, double z, double eps = 0.01):_x(x),_z(z),_eps(eps){}
		operator bool() const
		{
			if(_x != 0 && fabs(_x) < _eps)
				return true;

			if(_z != 0 && fabs(_z) < _eps)
				return true;

			return false;
		}
		string str() const
		{
			stringstream ss;
			ss << "X_linear = " << _x << ", Z_angular = " << _z;
			return ss.str();
		}
	};

	SurrealSpeed s(msg->linear.x, msg->angular.z);
	if(s) F_ERROR("PP - on_speed") << "Surrealistic speed alert: " << s.str() << endl;
}

/**
 * Checks the version of move_base by using the service.
 * (for standard move_base it won't work - we need Cogniteam's move_base).
 */
bool checkMoveBaseVersion(){
//	return true;
	 navex_msgs::VersionService version;
	 if(ros::service::call("move_base/version" , version)){
	 	std::string wantedPrefix = "Cogniteam";
	 	std::string versionPrefix = version.response.version.substr(0, wantedPrefix.size());
	 	if(versionPrefix == wantedPrefix){
	 		return true;
	 	} else {
	 		ROS_ERROR("Wrong version for move_base detected: expected prefix %s, found %s" , wantedPrefix.c_str() , versionPrefix.c_str());
	 		return false;
	 	}
	 }
	 else {
	 	// no service found - this is not cogniteam's version.
	 	ROS_ERROR("No version for move_base detected");
	 	return false;
	 }
}

void showMoveBaseInstallationInstructions() {
	std::string instructions = "ERROR: WRONG VERSION OF move_base DETECTED! "
			"\n Make sure that Cogniteam's version of move_base is in the workspace. That version can be obtained from iosp's robil2 repository."
			"\n For using Cogniteam's version, please make sure move_base is installed by running:"
			"\n    sudo apt-get install ros-indigo-move-base"
			"\n Also, before recompiling, it is advised to delete build/ and devel/  ";
	ROS_ERROR("%s" , instructions.c_str());
}


MoveBase::MoveBase(ComponentMain* comp)
	:is_active(true), gp_defined(false),gnp_defined(false), gl_defined(false), comp(comp), is_canceled(true), is_path_calculated(false), goal_calculator(0)
	//, last_move_base_vel_ok(false), last_real_vel_ok(false), pub_frequancy(30), resend_thread(0)
{
	ros::NodeHandle node;

	goalPublisher = node.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 5, false);
	originalGoalPublisher = node.advertise<geometry_msgs::PoseStamped>("/move_base/original_goal", 5, false);
	goalCancelPublisher = node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 5, false);
	pathSubscriber = node.subscribe("/move_base/NavfnROS/plan", 10, &MoveBase::on_nav_path, this);
	globalCostmapSubscriber = node.subscribe("/move_base/global_costmap/costmap", 1, &on_GlobalCostMap);
	speedSubscriber = node.subscribe("/cmd_vel", 1, &on_speed);
	globalPathPublisher = node.advertise<nav_msgs::Path>("/pp/global_path",1);
	selectedPathPublisher = node.advertise<nav_msgs::Path>("/pp/selected_path",1);
	pathVisualizationPublisher = node.advertise<sensor_msgs::PointCloud>("/path_visualization", 5, false);
	//moveBaseStatusSubscriber = node.subscribe("/move_base/status", 1, &MoveBase::on_move_base_status, this);
	diagnosticPublisher = node.advertise<diagnostic_msgs::DiagnosticArray>(DIAGNOSTIC_TOPIC_NAME, 100);

#if CREATE_POINTCLOUD_FOR_NAV == 1
	mapPublisher = node.advertise<sensor_msgs::PointCloud>("/map_cloud", 5, false);
	fakeLaserPublisher = node.advertise<sensor_msgs::LaserScan>("/costmap_clear_fake_scan",1,false);
#endif

	sub_log = node.subscribe("/rosout", 10, &MoveBase::on_log_message, this);

	// GOAL CALCULATOR
	ros::NodeHandle nh("~");
	goal_calculator = new RobilGC::GoalCalculator(nh.param<int>("gc_files", 0));

//	//FOR TEST
	sub_location = node.subscribe("/test/location", 10, &MoveBase::on_sub_loc, this);
	sub_location_cov = node.subscribe("/test/location_cov", 10, &MoveBase::on_sub_loc_cov, this);
	sub_map = node.subscribe("/test/map", 10, &MoveBase::on_sub_map, this);
	sub_path = node.subscribe("/test/path", 10, &MoveBase::on_sub_path, this);
	sub_commands = node.subscribe("/test/command", 10, &MoveBase::on_sub_commands, this);

}

void MoveBase::on_move_base_status(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
	if( not is_active ) return;
	size_t n = msg->status_list.size();
	for(size_t i=0;i<n;i++){
		const actionlib_msgs::GoalStatus& status = msg->status_list[i];
		if( status.text == "" or status.text == "''" ){
			//DBG_WARN("Navigation: move_base status is empty => aborted?");
			//on_error_from_move_base();
		}
	}
}

void MoveBase::on_log_message(const LogMessage::ConstPtr& msg){
	if(msg->name == "/move_base" and msg->level > LogMessage::DEBUG){
		on_log_message(msg->level, msg->msg);
	}
}
void MoveBase::on_log_message(int type, std::string message){
	std::stringstream pose_info;
	pose_info <<"; POSE["
	    <<"goal=("<<last_nav_goal.goal.target_pose.pose.position.x<<","<<last_nav_goal.goal.target_pose.pose.position.y<<"), "
	    <<"loc=(" <<gotten_location.pose.pose.position.x<<","<<gotten_location.pose.pose.position.y<<")"
	    <<"]"
	;
	if(type == LogMessage::WARN){
		bool rate_of = message.find("rate of")!=std::string::npos;
		
		DBG_INFO_ONCE("Navigation: move base: warning: "<<message<<(rate_of?string(""):pose_info.str()) );
		
	}else
	if(type == LogMessage::ERROR){
		bool skip = message.find("Aborting because a valid plan could not be found")!=string::npos;
		
		DBG_INFO_ONCE("Navigation: move base: error: "<<message<<(skip?" : skip this error":"")<< pose_info.str() );
		
		if(not skip) on_error_from_move_base();
	}else
	if(type == LogMessage::FATAL){
		DBG_INFO_ONCE("Navigation: move base: fatal: "<<message<< pose_info.str() );
		on_error_from_move_base();
	}
}

void MoveBase::on_error_from_move_base(){
	DBG_ERROR("Navigation: path is aborted. send event and clear current path.");
	stop_navigation(false);
}
void MoveBase::stop_navigation(bool success){
	notify_path_is_finished(success);
	gp_defined=gnp_defined=false;
	this->is_path_calculated = false;
	remove_memory_about_path(gotten_path.id);
	goalCancelPublisher.publish(last_nav_goal_id);
}

//================ TEST =================
void MoveBase::on_sub_map(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	this->on_map(*msg);
}
void MoveBase::on_sub_path(const nav_msgs::Path::ConstPtr& msg){
	this->on_path(*msg);
}
void MoveBase::on_sub_loc(const geometry_msgs::PoseStamped::ConstPtr& msg){
	geometry_msgs::PoseWithCovarianceStamped p;
	p.header.frame_id = "/WORLD";//NOTE: check if frame_id not has to be from robil_map.header
	p.header.stamp = ros::Time::now();
	p.pose.pose = msg->pose;
	this->on_position_update(p);
}
void MoveBase::on_sub_loc_cov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	//std::cout<<"path.poses.push_back(createPose("<< msg->pose.pose.position.x<<","<< msg->pose.pose.position.y<<"));"<<std::endl;
	geometry_msgs::PoseWithCovarianceStamped p = *msg;
	p.header.frame_id = "/WORLD";//NOTE: check if frame_id not has to be from robil_map.header
	this->on_position_update(p);
}
void MoveBase::on_sub_commands(const std_msgs::String::ConstPtr& msg){
	SYNCH
	std::string command = msg->data;
	if(command == "clear"){
		DBG_INFO("Navigation: on command CLEAR");
		this->gl_defined = false;
		this->gnp_defined= false;
		this->gp_defined = false;
	}
	if(command == "cancel"){
		DBG_INFO("Navigation: on command CANCEL");
		this->cancel();
	}
}
//=======================================

MoveBase::~MoveBase() {
	delete goal_calculator;
}

bool MoveBase::all_data_defined()const{
	return gl_defined and (gp_defined or gnp_defined) and is_active and not is_canceled;
}

void MoveBase::notify_path_is_finished(bool success)const{
	if(success) comp->rise_taskFinished();
	else comp->rise_taskAborted();
}

void MoveBase::on_position_update(const geometry_msgs::PoseWithCovarianceStamped& _location){
SYNCH

	int failts_counter=0;
	geometry_msgs::PoseWithCovarianceStamped location = _location;

on_position_update_START:
	bool exception_is_catched=false;
	try{
//		DBG_INFO("\t [1]on_position_update( "<<str_position(location.pose.pose.position, location)<<" )");
		gotten_location = location;
//		geometry_msgs::PoseStamped tmp; tmp.pose=location.pose.pose; tmp.header = location.header;
		tfListener().waitForTransform("WORLD", gotten_location.header.frame_id, gotten_location.header.stamp, ros::Duration(5.0));
		gotten_location.header.frame_id = "WORLD";
//		tmp = transform_to_frame(tmp, "WORLD", str_dbg_location(__FILE__,__LINE__));
//		gotten_location.pose.pose = tmp.pose;
//		gotten_location.header = tmp.header;
//		gotten_location.pose.pose = location.pose.pose;
//
//		DBG_INFO("\t [2]on_position_update( "<<str_position(gotten_location.pose.pose.position, gotten_location)<<" )");
	}
	catch(tf2::ExtrapolationException& t1){exception_is_catched=true;DBG_ERROR("Navigation: [e] tf2::ExtrapolationException");}
	catch(tf2::InvalidArgumentException& t2){
		exception_is_catched=true;DBG_ERROR("Navigation: [e] tf2::InvalidArgumentException\n "<<t2.what());
		std::string msg(t2.what());
		bool retry = false;
		if(msg.find("Quaternion malformed, magnitude")!=std::string::npos)
		{
			DBG_INFO("Q: "<<location.pose.pose.orientation.x<<", "<<location.pose.pose.orientation.y<<", "<<location.pose.pose.orientation.z<<", "<<location.pose.pose.orientation.w);
			double s = sqrt(pow(location.pose.pose.orientation.x,2) + pow(location.pose.pose.orientation.y,2) +pow(location.pose.pose.orientation.z,2) +pow(location.pose.pose.orientation.w,2) );
			location.pose.pose.orientation.x /= s;
			location.pose.pose.orientation.y /= s;
			location.pose.pose.orientation.z /= s;
			location.pose.pose.orientation.w /= s;
			double s1 = sqrt(pow(location.pose.pose.orientation.x,2) + pow(location.pose.pose.orientation.y,2) +pow(location.pose.pose.orientation.z,2) +pow(location.pose.pose.orientation.w,2) );
			DBG_INFO("Q: "<<location.pose.pose.orientation.x<<", "<<location.pose.pose.orientation.y<<", "<<location.pose.pose.orientation.z<<", "<<location.pose.pose.orientation.w);
			DBG_ERROR("Navigation: [e] quaternion magnitude has been changed "<<s<<" -> "<<s1);
			retry = true;
		}
		if(retry and failts_counter<10){
			failts_counter++;
			DBG_INFO("Navigation: [e] RETRY #"<<failts_counter);
			goto on_position_update_START;
		}
	}
	catch(...){exception_is_catched = true; DBG_ERROR("Navigation: [e] Unknown type of exception");}
	if(exception_is_catched){
		DBG_ERROR("Navigation: update of position is rejected. The reason is problems with transformation to WORLD frame.");
		return;
	}
	//DBG_INFO("Navigation: position is updated.");
	gl_defined=true;
	if(all_data_defined()) calculate_goal();
}

#define STR(P) P.x<<","<<P.y
template <class T>
int byte_compare(const T& a, const T& b)
{
	return memcmp( &a, &b, sizeof(b));
}

void MoveBase::extend_path()
{
	geometry_msgs::Pose lp;
	size_t waypoints = gotten_path.waypoints.poses.size();

	/* If path is empty, do nothing */
	if(waypoints < 1)
		return;

	/* If path consists of one waypoint,
	 * extend it by adding another waypoint with orientation of (waypoint - robot_pose)
	 * and distance of 0.5m from the actual waypoint.
	 *
	 * Else (two or more waypoints),
	 * extend the path by adding a waypoint with orientation of (waypoint N - waypoint N-1)
	 * and distance of 0.5m from last waypoint.
	 */

	if(waypoints == 1)
		lp = gotten_location.pose.pose;
	else
		lp = gotten_path.waypoints.poses[waypoints - 2].pose;

	geometry_msgs::Pose last = gotten_path.waypoints.poses[waypoints - 1].pose;

    geometry_msgs::Point p;
    p.x = last.position.x-lp.position.x;
    p.y = last.position.y-lp.position.y;
    double len = hypot(p.x,p.y);
    if(len>0.001){
	p.x /= len;
	p.y /= len;
	p.x *= 0.5;
	p.y *= 0.5;
	p.x += last.position.x;
	p.y += last.position.y;
	geometry_msgs::PoseStamped npose = gotten_path.waypoints.poses[waypoints-1];
	npose.pose.position = p;
	gotten_path.waypoints.poses.push_back( npose );
	DBG_INFO("Navigation: path extended by adding additional tail point")
    }
}

void MoveBase::on_path(const robil_msgs::Path& input_goal_path){
SYNCH
	robil_msgs::Path goal_path = input_goal_path;

	DBG_INFO("Navigation: Global path gotten. Number of way points is "<<goal_path.waypoints.poses.size()<<" ");

	if(goal_path.waypoints.poses.size()==0) return;

	robil_msgs::Path goal_path_tmp = goal_path;
	goal_path_tmp.waypoints.poses.clear();
	goal_path_tmp.waypoints.poses.push_back(goal_path.waypoints.poses[0]);
	for(size_t i=1;i<goal_path.waypoints.poses.size();i++)
	{
		if(byte_compare( goal_path_tmp.waypoints.poses.back().pose.position, goal_path.waypoints.poses[i].pose.position)==0)
		{
			DBG_INFO("Navigation: remove point "<<i<<" from path because it's same as point "<<i-1<<" : "<<goal_path_tmp.waypoints.poses.back().pose.position.x<<","<<goal_path_tmp.waypoints.poses.back().pose.position.y);
			continue;
		}
		goal_path_tmp.waypoints.poses.push_back(goal_path.waypoints.poses[i]);
	}
	goal_path = goal_path_tmp;

	gotten_path = goal_path;
	gp_defined=true;

	extend_path();
//	if(gotten_path.waypoints.poses.size()>=1){
//	    geometry_msgs::Pose last = gotten_path.waypoints.poses[gotten_path.waypoints.poses.size()-1].pose;
//	    geometry_msgs::Pose llast = gotten_path.waypoints.poses[gotten_path.waypoints.poses.size()-2].pose;
//	    geometry_msgs::Point p;
//	    p.x = last.position.x-llast.position.x;
//	    p.y = last.position.y-llast.position.y;
//	    double len = hypot(p.x,p.y);
//	    if(len>0.001){
//		p.x /= len;
//		p.y /= len;
//		p.x *= 0.5;
//		p.y *= 0.5;
//		p.x += last.position.x;
//		p.y += last.position.y;
//		geometry_msgs::PoseStamped npose = gotten_path.waypoints.poses[gotten_path.waypoints.poses.size()-1];
//		npose.pose.position = p;
//		gotten_path.waypoints.poses.push_back( npose );
//		DBG_INFO("Navigation: path extended by adding additional tail point");
//	    }
//	}
	BOOST_FOREACH( const geometry_msgs::PoseStamped& p , gotten_path.waypoints.poses )
	{
	    DBG_INFO("Navigation: ...... "<<STR(p.pose.position));
	}

	if(not is_active){
		DBG_WARN("Navigation: New Global Path is rejected, because navigation is deactivated");
		return;
	}
	if(all_data_defined()) calculate_goal();
}
void MoveBase::on_path(const nav_msgs::Path& input_goal_path){
SYNCH

	nav_msgs::Path goal_path = input_goal_path;

	DBG_INFO("Navigation: Global path gotten. Number of way points is "<<goal_path.poses.size()<<" ");

	if(goal_path.poses.size()==0) return;

	nav_msgs::Path goal_path_tmp = goal_path;
	goal_path_tmp.poses.clear();
	goal_path_tmp.poses.push_back(goal_path.poses[0]);
	for(size_t i=1;i<goal_path.poses.size();i++)
	{
		if(byte_compare( goal_path_tmp.poses.back().pose.position, goal_path.poses[i].pose.position)==0)
		{
			DBG_INFO("Navigation: remove point "<<i<<" from path because it's same as point "<<i-1<<" : "<<goal_path_tmp.poses.back().pose.position.x<<","<<goal_path_tmp.poses.back().pose.position.y);
			continue;
		}
		goal_path_tmp.poses.push_back(goal_path.poses[i]);
	}
	goal_path = goal_path_tmp;
	
	gotten_nav_path = goal_path;
	gnp_defined=true;

	extend_path();
//	if(gotten_nav_path.poses.size()>=1){
//	    geometry_msgs::Pose last = gotten_nav_path.poses[gotten_nav_path.poses.size()-1].pose;
//	    geometry_msgs::Pose llast = gotten_nav_path.poses[gotten_nav_path.poses.size()-2].pose;
//	    geometry_msgs::Point p;
//	    p.x = last.position.x-llast.position.x;
//	    p.y = last.position.y-llast.position.y;
//	    double len = hypot(p.x,p.y);
//	    if(len>0.001){
//		p.x /= len;
//		p.y /= len;
//		p.x *= 0.5;
//		p.y *= 0.5;
//		p.x += last.position.x;
//		p.y += last.position.y;
//		geometry_msgs::PoseStamped npose = gotten_nav_path.poses[gotten_nav_path.poses.size()-1];
//		npose.pose.position = p;
//		gotten_nav_path.poses.push_back( npose );
//		DBG_INFO("Navigation: path extended by adding additional tail point");
//	    }
//	}
	BOOST_FOREACH( const geometry_msgs::PoseStamped& p , gotten_nav_path.poses )
	{
	    DBG_INFO("Navigation: ...... "<<p.pose.position.x<<","<<p.pose.position.y);
	}
	if(not is_active){
		DBG_WARN("Navigation: New Global Path is rejected, because navigation is deactivated");
		return;
	}
	if(all_data_defined()) calculate_goal();
}

void MoveBase::publish_global_gotten_path_visualization(nav_msgs::Path global_gotten_path){
	//publish the gotten goal as PointCloud message, for better visualization.
	sensor_msgs::PointCloud path_visualization;

	for(int i=0; i<global_gotten_path.poses.size(); ++i){
		geometry_msgs::Point32 vis_point;
		geometry_msgs::PoseStamped_<std::allocator<void> > path_point = global_gotten_path.poses.at(i);

		vis_point.x = path_point.pose.position.x;
		vis_point.y = path_point.pose.position.y;
		vis_point.z = path_point.pose.position.z;

		path_visualization.points.push_back(vis_point);
	}

	path_visualization.header.frame_id = global_gotten_path.header.frame_id;
	path_visualization.header.stamp =ros::Time::now();

	pathVisualizationPublisher.publish(path_visualization);
}

void MoveBase::cancel(bool clear_last_goals){
	SYNCH
	is_canceled = true;
	this->gl_defined = false;
	if(clear_last_goals){
		this->gnp_defined= false;
		this->gp_defined = false;
	}
	this->is_path_calculated = false;
	last_nav_goal = move_base_msgs::MoveBaseActionGoal();
	goalCancelPublisher.publish(last_nav_goal_id);
	goal_counter++;
}
void MoveBase::activate(){
	SYNCH
	is_active=true;
	is_canceled=false;
	last_nav_goal = move_base_msgs::MoveBaseActionGoal();
	DBG_INFO("Navigation: navigation is active. you can send Global Path.");
	if(clear_path_on_activate){
		this->gnp_defined= false;
		this->gp_defined = false;
		DBG_INFO("Navigation:   the previous path is deleted");
	}else{
		DBG_INFO("Navigation:   the previous path is restored.");
	}
}
void MoveBase::deactivate(bool clear_last_goals){
	SYNCH
	is_active=false;
	clear_path_on_activate = clear_last_goals;
	cancel();
	DBG_INFO("Navigation: navigation is deactivated. The driver is stopped and each new Global Path will rejected up to activation.");
}

void path_publishing(ComponentMain* comp, boost::recursive_mutex* mtx, bool* is_canceled, bool* is_path_calculated){
	struct Sleeper
	{
		ros::Rate& r;
		Sleeper(ros::Rate& r):r(r){}
		~Sleeper(){r.sleep();}
	};
	ros::Rate r(1);
	while(not boost::this_thread::interruption_requested() and ros::ok())
	{
		Sleeper s(r);
		robil_msgs::Path lpath;
		lpath.is_heading_defined=false;
		lpath.is_ip_defined=false;
		{
			boost::recursive_mutex::scoped_lock locker(*mtx);
			if(*is_canceled or not *is_path_calculated) continue;

			lpath.waypoints = curr_nav_path;
		}
		comp->publishLocalPath(lpath);
	}
}

void MoveBase::on_nav_path(const nav_msgs::Path& nav_path){
	SYNCH
	if(not is_active) return;

	is_path_calculated = true;
	curr_nav_path = nav_path;
	if(threads.size()==0) threads.add_thread(new boost::thread(boost::bind(path_publishing, comp, &mtx, &is_canceled, &is_path_calculated)));
}

//template<class T, class H>
//inline std::string str_position(const T& p, const H& h)
//{
//	std::stringstream s; s<<"("<<p.x<<", "<<p.y<<")["<<h.header.frame_id<<"]";
//	return s.str();
//}

std::string MoveBase::init_path()
{
	nav_msgs::Path gotten_global_path;
	std::string path_id = " ";
	if(gp_defined)
	{
		gotten_global_path = gotten_path.waypoints;
		path_id = gotten_path.id;
	}
	else
		gotten_global_path = gotten_nav_path;

	size_t waypoints = gotten_global_path.poses.size();
	gotten_global_path.header.frame_id = "/WORLD";
	gotten_global_path.header.stamp = ros::Time::now() ;
	globalPathPublisher.publish(gotten_global_path);
	selectedPathPublisher.publish(gotten_global_path);
	publish_global_gotten_path_visualization(gotten_global_path);

	goal_calculator->updatePath(gotten_global_path);

	return path_id;
}

void MoveBase::calculate_goal()
{
	f_counter++;
	int goal_index = 0;
	bool is_path_finished = false;
	geometry_msgs::PoseStamped goal;

	/* Obtain global path */
	string path_id = init_path();

	/* Obtain global costmap */
	global_map_mutex.lock();
	nav_msgs::OccupancyGrid map = global_cost_map;
	global_map_mutex.unlock();
	if(not goal_calculator->updateMap(map, gotten_location))
		DBG_WARN("Navigation: Global occupancy cost map is not defined");

	/* Get goal */
	bool gc_res = goal_calculator->get(goal, goal_index);

	if(!gc_res)
		is_path_finished = true;

	if (is_path_finished)
	{
		DBG_INFO("Navigation: path is finished. send event and clear current path.");
		stop_navigation(true);
		return;
	}

	static boost::posix_time::ptime time_since_path_not_finished_last_time; //hold timestamp for last path that was not finished
	if (is_path_finished)
	{
		//consider duration of "finished" message for path, in order to consider it as truly finished.
		//if not enough time has passed since considering this path as finished, do nothing.
		if (boost::get_system_time() - time_since_path_not_finished_last_time >= boost::posix_time::seconds(SECONDS_FOR_FINISHING_AFTER_PATH_FINISHED))
		{
			DBG_INFO("Navigation: path is finished (after tries to make it reachable). send event and clear current path.");
			stop_navigation(true);
			return;
		}
	}
	else
	{
		time_since_path_not_finished_last_time = boost::get_system_time();
	}

	diagnostic_publish_new_goal(path_id, goal, goal_index, gotten_location);
	update_unvisited_index(path_id, goal_index);
	goal.header = gp_defined ? gotten_path.waypoints.header : gotten_nav_path.header;
	on_goal(goal);
}

namespace{
	typedef diagnostic_msgs::DiagnosticStatus::_values_type::value_type KeyValue;

	KeyValue diag_value_str(const std::string& key, const std::string& value){
		KeyValue v;
		v.key = key; v.value = value;
		return v;
	}
	template<class T>
	KeyValue diag_value(const std::string& key, const T& value){
		std::stringstream s; s<<value;
		return diag_value_str(key, s.str());
	}
	template<>
	KeyValue diag_value(const std::string& key, const std::string& value){
		return diag_value_str(key, value);
	}

}
void MoveBase::diagnostic_publish_new_goal(const string& path_id, const geometry_msgs::PoseStamped& goal, size_t goal_index, const geometry_msgs::PoseWithCovarianceStamped& gotten_location){
	using namespace diagnostic_msgs;
	DiagnosticArray array;
	DiagnosticStatus status;
	status.hardware_id ="";
	status.level = DiagnosticStatus::OK;
	status.name = "path planner";
	stringstream sid; sid<<"goal: task("<<path_id<<")["<<goal_index<<"]=("<<goal.pose.position.x<<","<<goal.pose.position.y<<")";
	status.message = sid.str();
	status.values.push_back(diag_value("task_id",path_id));
	status.values.push_back(diag_value("goal",goal));
	status.values.push_back(diag_value("index",goal_index));
	status.values.push_back(diag_value("pose",gotten_location));
	array.status.push_back(status);
	array.header.stamp = ros::Time::now();
	if(last_diagnostic_message_id!=status.message){
		diagnosticPublisher.publish(array);
		last_diagnostic_message_id = status.message;
	}
}

void MoveBase::on_goal(const geometry_msgs::PoseStamped& robil_goal){

	double robil_goal_x = fround<1>(robil_goal.pose.position.x);
	double robil_goal_y = fround<1>(robil_goal.pose.position.y);
	double distance_prev_and_new_goal = hypot( last_nav_goal.goal.target_pose.pose.position.x-robil_goal_x , last_nav_goal.goal.target_pose.pose.position.y-robil_goal_y);
	double distance_to_gaol = hypot( gotten_location.pose.pose.position.x-robil_goal_x , gotten_location.pose.pose.position.y-robil_goal_y);
	static int rejection_counter=0;
	if(

//		true

//		last_nav_goal.goal.target_pose.pose.position.x == robil_goal_x and
//		last_nav_goal.goal.target_pose.pose.position.y == robil_goal_y
	  
		( rejection_counter < 50 and distance_prev_and_new_goal < 1.5 )
		or
		distance_to_gaol > 90

	){
	    if( distance_prev_and_new_goal > 0.5 )
	    {
	    	DBG_INFO("Navigation: goal is rejected. the goal is "<<(distance_prev_and_new_goal<1.5?" same one.":"")<<(distance_to_gaol > 90?"too far":"")<<" goal="<<robil_goal_x<<","<<robil_goal_y);
	    }
	    rejection_counter++;
	    return;
	}else{
	   DBG_INFO("Navigation: goal is accepted. "
			   <<"prev="<<str_position(last_nav_goal.goal.target_pose.pose.position,last_nav_goal)
			   <<", new="<<robil_goal_x<<","<<robil_goal_y<<"["<<robil_goal.header.frame_id<<":"<<robil_goal.header.stamp<<"]");
	   rejection_counter=0;
	}

	//check version, since it is likely that move_base is running in this point.
	if (!checkMoveBaseVersion()) {
		showMoveBaseInstallationInstructions();
		exit(1);
	}


	move_base_msgs::MoveBaseActionGoal goal;
	geometry_msgs::PoseStamped ps_goal;

	ps_goal = robil_goal;
	remove_orientation(ps_goal);
	ps_goal.pose.position.x = robil_goal_x;
	ps_goal.pose.position.y = robil_goal_y;

	goal.header.frame_id = "/WORLD";
	goal.header.stamp = ros::Time::now();
	ps_goal.header = goal.header;

	std::stringstream sid ;
	static long send_counter=0; send_counter++;
	sid<<"[i] goal #"<< boost::lexical_cast<std::string>(goal_counter) <<": "
					 << ps_goal.pose.position.x<<","<<ps_goal.pose.position.y<<";"<<send_counter;
	goal.goal_id.id = sid.str();
	goal.goal_id.stamp = goal.header.stamp;
	
	goal.goal.target_pose = ps_goal;
	
	//DBG_INFO("Navigation: set new goal : "<<ps_goal.pose.position.x<<","<<ps_goal.pose.position.y);
	last_nav_goal_id = goal.goal_id;
	last_nav_goal = goal;

//	goalCancelPublisher.publish(last_nav_goal_id);
	goalPublisher.publish(goal);
}

void MoveBase::on_map(const nav_msgs::OccupancyGrid& nav_map){
SYNCH

	nav_msgs::OccupancyGrid map = nav_map;
	mapPublisher.publish(map);
}


void MoveBase::on_map(const robil_msgs::Map& robil_map){
SYNCH

#if CREATE_POINTCLOUD_FOR_NAV == 1

	sensor_msgs::PointCloud map;
	static ros::NodeHandle node("~");
	int rad_clear =2;
	if(not node.getParamCached("clear_map_radius",rad_clear) ) node.setParam("clear_map_radius",rad_clear);

	double x_offset = robil_map.info.width*robil_map.info.resolution * 2.0/3.0;
	double y_offset = robil_map.info.height*robil_map.info.resolution * 1.0/2.0;

	for(
			size_t y=0;
			y<robil_map.info.height;
			y++
	){
		float fy = y*robil_map.info.resolution;
		for(
				size_t x=0;
				x<robil_map.info.width;
				x++
		){
			size_t i = y*robil_map.info.width+x;
			float fx = x*robil_map.info.resolution;


			float fz = 0;

			if(robil_map.data[i].type==robil_msgs::MapCell::type_obstacle /*or rand()%10>8*/){
				fz = 1 - gotten_location.pose.pose.position.z;
			}else
			if(robil_map.data[i].type==robil_msgs::MapCell::type_clear){
				continue;
				fz = -1; //0
			}else
			if(robil_map.data[i].type==robil_msgs::MapCell::type_unscanned){
				continue;
				fz = -1;
			}
			geometry_msgs::Point32 point;
			point.x = -fx + x_offset;
			point.y = -fy + y_offset;
			point.z = fz;

			if( hypot( point.x, point.y ) < rad_clear ) continue;
			map.points.push_back(point);
		}
	}

	if (map.points.size() == 0) return;

	map.header.frame_id = "/ODOM"; 
	map.header.stamp = ros::Time::now();//robil_map.header.stamp;//


	static sensor_msgs::LaserScan fake_scan; static bool fsd=false; static long fk_iter=0;fk_iter++;
	fake_scan.header = map.header;
	if(!fsd){fsd=true;
		const int ranges_count=359*1;
		fake_scan.angle_min = 0;
		fake_scan.angle_max = (359.0/360.0)*2*M_PI;
		fake_scan.angle_increment = fake_scan.angle_max/ranges_count;
		fake_scan.range_min=0.1;
		fake_scan.range_max=31;
		fake_scan.ranges.resize(ranges_count);
		for(int i=0;i<ranges_count;i++) fake_scan.ranges[i]=30;
	}
	if(fk_iter%100==0) fakeLaserPublisher.publish(fake_scan);
	mapPublisher.publish(map);

#endif
}



size_t MoveBase::get_unvisited_index(string path_id){
	if(path_id=="") return 0;
	boost::recursive_mutex::scoped_lock l(mtx);
	size_t inx=0;
	if(unvisited_index.find(path_id)==unvisited_index.end()){
		unvisited_index[path_id]=0;
	}
	inx = unvisited_index.at(path_id);
	return inx;
}
void MoveBase::remove_memory_about_path(string path_id){
	last_diagnostic_message_id = "";
	if(path_id=="") return;
	boost::recursive_mutex::scoped_lock l(mtx);
	if(unvisited_index.find(path_id)==unvisited_index.end()) return;
	unvisited_index.erase(path_id);
}

void MoveBase::update_unvisited_index(string path_id, size_t new_index){
	if(path_id=="") return;
	boost::recursive_mutex::scoped_lock l(mtx);
	if(unvisited_index[path_id] != new_index)
	{
	    DBG_INFO_ONCE("Navigation: new waypoint index is "<<new_index);
	}
	unvisited_index[path_id] = new_index;
}

