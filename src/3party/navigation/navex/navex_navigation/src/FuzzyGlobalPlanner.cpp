/*
 * FuzzyGlobalPlanner.cpp
 *
 *  Created on: Jan 15, 2017
 *      Author: assaf
 */

#include <navex_navigation/FuzzyGlobalPlanner.h>

#if USE_DAMPING == 1
	#include "path_damping/PathDamping.h"
	#include <fstream>
#endif


FuzzyGlobalPlanner::FuzzyGlobalPlanner()
	: pathFinder_(new NavexPathFinder())
{
}

FuzzyGlobalPlanner::~FuzzyGlobalPlanner()
{
	delete pathFinder_;
}

void FuzzyGlobalPlanner::initialize()
{

}

bool FuzzyGlobalPlanner::makePlan(CostMap& costmap,
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path) const
{

	geometry_msgs::PoseStamped actualStart = start;
	geometry_msgs::PoseStamped actualGoal = goal;

	if (costmap.getCellValue(start) >= CostMapCell::CELL_BLOCKED) {
		// Starting position is block, try clearing
		ROS_INFO_STREAM("FuzzyGlobalPlanner: Starting position ("<<actualStart.pose.position.x<<","<<actualStart.pose.position.y<<") is blocked, searching for a nearest not blocked point...");
		//costmap.clearCell(start, costmap.getRobotRadius());
		if( findNonBlockedStart(costmap, start, actualStart) )
			ROS_INFO_STREAM("FuzzyGlobalPlanner:    actual start is "<<actualStart.pose.position.x<<","<<actualStart.pose.position.y);
		else
			ROS_WARN_STREAM("FuzzyGlobalPlanner:    free point is not found. actual start is "<<actualStart.pose.position.x<<","<<actualStart.pose.position.y);
	}

	if (costmap.getCellValue(actualGoal) >= CostMapCell::CELL_BLOCKED) {
		ROS_INFO_STREAM("FuzzyGlobalPlanner: Goal ("<<actualGoal.pose.position.x<<","<<actualGoal.pose.position.y<<") is blocked, searching for a nearest not blocked point ...");
		if( findNonBlockedGoal(costmap, actualStart, goal, actualGoal) )
			ROS_INFO_STREAM("FuzzyGlobalPlanner:    actual goal is "<<actualGoal.pose.position.x<<","<<actualGoal.pose.position.y);
		else
			ROS_WARN_STREAM("FuzzyGlobalPlanner:    free point is not found. actual goal is "<<actualGoal.pose.position.x<<","<<actualGoal.pose.position.y);
	}

	bool pathFound = false;

	try {
		pathFound = pathFinder_->findPath(costmap, actualStart, actualGoal, path);
	} catch (...) {
		ROS_ERROR("Unexpected error occurred while planning");
		return false;
	}

#if USE_DAMPING == 1

#	define LOG_PATH_TO_FILE

	const double min_distance_for_near_objects = 1.0; // 1m
	const bool is_last_required_goal_near_current_goal =
			cast_pose(goal				).getOrigin().distance(
			cast_pose(lastRequiredGoal_	).getOrigin()
			) < min_distance_for_near_objects;

#	ifdef LOG_PATH_TO_FILE
	static std::ofstream damping_log("/tmp/damping.log");

	damping_log << "Planning request. New goal from previous goal is far as "<<(
			cast_pose(goal				).getOrigin().distance(
			cast_pose(lastRequiredGoal_	).getOrigin())
			)<<" meteres" <<endl;
#	endif

	if( is_last_required_goal_near_current_goal )
	{
		static PathDamping path_damping;

#		ifdef LOG_PATH_TO_FILE
		damping_log <<"Original path (damping) : "<<endl<< path<<endl;
#		endif

		path_damping.filter(
				lastCalucaltedPath_,
				path,
				start,
				goal,

				path
		);

#		ifdef LOG_PATH_TO_FILE
		damping_log <<"Damped path : "<<endl<< path<<endl;
#		endif

	}

#	ifdef LOG_PATH_TO_FILE
	else
	{
		damping_log <<"Original path (no damping) : "<<endl<< path<<endl;
	}
#	endif

	lastRequiredGoal_ = goal;
	lastCalucaltedPath_ = path;

#endif // USE_DAMPING == 1

	return pathFound;
}

bool FuzzyGlobalPlanner::findNonBlockedGoal(
		const CostMap& costmap,
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,

		geometry_msgs::PoseStamped& result
		) const
{
	bool res = findNonBlockedNearestPointOnViewline(costmap, start, goal, result);
	if( res ) return res;
	res = findNonBlockedNearestPoint(costmap, goal, result);
	return res;
}

bool FuzzyGlobalPlanner::findNonBlockedNearestPointOnViewline(
		const CostMap& costmap,
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,

		geometry_msgs::PoseStamped& result
		) const
{
	geometry_msgs::PoseStamped actualGoal = goal;

	cv::Point startPoint, goalPoint;

	startPoint = costmap.poseToPixel(start);
	goalPoint = costmap.poseToPixel(goal);

	tf::Vector3 startVec(startPoint.x, startPoint.y, 0);
	tf::Vector3 goalVec(goalPoint.x, goalPoint.y, 0);

	double goalDistance = startVec.distance(goalVec);
	tf::Vector3 stepVector = (startVec - goalVec) / goalDistance; // 1px step

	tf::Vector3 currentPoint = goalVec;

	for (size_t i = 0; i < (int)goalDistance; ++i) {
		currentPoint += stepVector;

		CostMapCell::CellType cellValue =
				costmap.getCellValue(
						(uint32_t)currentPoint.x(), (uint32_t)currentPoint.y());

		if (cellValue < CostMapCell::CELL_BLOCKED) {
			ROS_INFO("findNonBlockedGoal: Free point found!");

			if( startPoint == goalPoint)
			{
				ROS_INFO("findNonBlockedGoal: ... but the point is same as start");
				result = start;
				return false;
			}

			result = costmap.pixelToPose(currentPoint.x(), currentPoint.y());
			return false;
		}
	}

	// Failed to find
	result = start;
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
	tf::Vector3* next;
	tf::Vector3* r;
	tf::Vector3* w;

	static tf::Vector3* & points(){ static tf::Vector3* v(0); return v; }
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
			points() = new tf::Vector3[size];
			points_size()=size;
		}
		r=w=next = points();
		memset(next,0,size*sizeof(tf::Vector3));
	}
	~points_pool()
	{
		//delete[] next; next=0;
	}
	void push_back(const tf::Vector3& p){ *w = p; w++; }
	const tf::Vector3& front()const{ return *r; }
	void pop_front(){ r++; }
	bool empty()const{ return not( r<w ); }
};

inline int index(double x, double y, double w, double h){ return y*w+x; }
inline int index(const tf::Vector3& v, double w, double h){ return index(v.x(), v.y(), w, h); }
inline bool inside(double x, double y, double w, double h){ return 0<=x and x<w and 0<=y and y<h; }
inline bool inside(const tf::Vector3& v, double w, double h){ return inside(v.x(), v.y(), w, h); }

bool FuzzyGlobalPlanner::findNonBlockedStart(
		const CostMap& costmap,
		const geometry_msgs::PoseStamped& start,

		geometry_msgs::PoseStamped& result

		) const
{
	return findNonBlockedNearestPoint(costmap, start, result);
}

bool FuzzyGlobalPlanner::findNonBlockedNearestPoint(
		const CostMap& costmap,
		const geometry_msgs::PoseStamped& start,

		geometry_msgs::PoseStamped& result

		) const
{

	cv::Point startPoint;
	startPoint = costmap.poseToPixel(start);

	tf::Vector3 startVec(startPoint.x, startPoint.y, 0);

	{
		int w = costmap.getWidth();
		int h = costmap.getHeight();
		char* reachable;
		array_manager<char> rc(reachable, w*h);
		points_pool next(w*h);

		const char REACHABLE=1;
		const char UNREACHABLE=2;
		const char UNKNOWN=0;
		reachable[index(startVec, w,h)] = REACHABLE;																					// select all reachable cells
		next.push_back(startVec);
		while(next.empty()==false){
			tf::Vector3 current = next.front(); next.pop_front();
			//FOR ALL NAIGHBORS
			for(int iy=-1;iy<=1;iy++)for(int ix=-1;ix<=1;ix++){
				if( ix==0 and iy==0 ) 										continue;

				tf::Vector3 nei(current.x()+ix,current.y()+iy, 0);

				if/*outside of map*/	(inside(nei, w,h)==false) 			continue;
				if/*already checked*/	(reachable[index(nei, w,h)]>UNKNOWN)	continue;

				reachable[index(nei, w,h)] = REACHABLE;
				CostMapCell::CellType cellValue = costmap.getCellValue((uint32_t)nei.x(), (uint32_t)nei.y());
				if/*occupied*/			(cellValue < CostMapCell::CELL_BLOCKED)
				{
					result = costmap.pixelToPose(nei.x(), nei.y());
					return true;
				}
				/*else*/
				reachable[index(nei, w,h)] = UNREACHABLE;
				next.push_back(nei);
			}
		}
		for(size_t i=0; i<w*h; i++) if(reachable[i]==UNKNOWN) reachable[i]=UNREACHABLE;
	}
	// Failed to find
	result = start;
	return false;
}

