
#include "GoalCalculator.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace goal_calculator;


inline
double len(const cv::Point& p){ return hypot(p.x, p.y); }

inline
double ang(const cv::Point& p){ return atan2(p.y, p.x); }

struct PointCmp
{
	cv::Point p;
	PointCmp(){}
	PointCmp(const PointCmp& p):p(p.p){}
	PointCmp(const cv::Point& p):p(p){}

	bool operator<(const PointCmp& p2)const
	{
		if(p.x==p2.p.x) return p.y<p2.p.y;
		return p.x<p2.p.x;
	}
};


deque<cv::Point> waypoints;
set<PointCmp> obstacles;
cv::Point robot;
bool world_change=true;

int last_event, last_flags;
void mouseHandler(int event, int mouse, int x,int y, int flags,void* param)
{
    if ( event == cv::EVENT_LBUTTONDOWN )
    {
    	robot = cv::Point(x,y);
    	world_change = true;
    }
    else if ( event == cv::EVENT_MBUTTONDOWN )
    {
		if ( flags & cv::EVENT_FLAG_SHIFTKEY )
		{
			for(set<PointCmp>::const_iterator i=obstacles.begin(); i!=obstacles.end(); i++)
				if( len(i->p - cv::Point(x,y))<20 ){ obstacles.erase(i); break; }
		}
		else if( mouse != cv::EVENT_MOUSEMOVE)
		{
			obstacles.insert(PointCmp(cv::Point(x,y)));
		}
    	world_change = true;
    }
    else if ( event == cv::EVENT_RBUTTONDOWN )
    {

		if ( flags & cv::EVENT_FLAG_SHIFTKEY )
		{
			deque<cv::Point> tmp;
			while(waypoints.empty()==false)
			{
				cv::Point p = waypoints.front();
				waypoints.pop_front();
				if( len(p - cv::Point(x,y))<10 ) continue;
				tmp.push_back(p);
			}
			waypoints = tmp;
		}
		else if( mouse != cv::EVENT_MOUSEMOVE)
		{
			waypoints.push_back(cv::Point(x,y));
		}
    	world_change = true;
    }
}
void mouseHandler(int event,int x,int y, int flags,void* param)
{
	if(event == cv::EVENT_MOUSEMOVE)
		mouseHandler(last_event, event, x,y, flags, param);
	else
	{
		mouseHandler(event, event, x,y, flags, param);
		last_event = event;
		last_flags = flags;
	}
}

int main() {
	GoalCalculator gc;

	cv::Mat world(300,500, CV_8UC3, cv::Scalar(255,255,255));
	cv::namedWindow("world", CV_WINDOW_NORMAL);
	cv::setMouseCallback("world", mouseHandler, 0);

	cv::Mat internal(300,500, CV_8UC3, cv::Scalar(255,255,255));
	cv::namedWindow("internal", CV_WINDOW_NORMAL);

	long key=0;
	const long ESC=1048603;
	do{

		if(world_change)
		{
//			cout<<"{";cout.flush();

			cv::rectangle(world, cv::Point(0,0), cv::Point(world.cols, world.rows), cv::Scalar(255,255,255), -1);

//			cout<<"1";cout.flush();
			for( set<PointCmp>::const_iterator i=obstacles.begin(); i!=obstacles.end(); i++)
			{
				cv::circle(world, i->p, 20, cv::Scalar(100,100,50), -1);
			}

//			cout<<"2";cout.flush();
			goal_calculator::Map map(world.cols, world.rows, Point2d(0,0), 0, 1.0);
			for(int y=0;y<world.rows;y++)for(int x=0;x<world.cols;x++)
			{
				 if(world.at<cv::Vec3b>(y,x)[0]<250) 	map.set_occuped_value(map(x,y));
				 else									map.set_free_value(map(x,y));
			}

//			cout<<"3";cout.flush();
			for( deque<cv::Point>::const_iterator i=waypoints.begin(); i!=waypoints.end(); i++)
			{
				cv::circle(world, *i, 5, cv::Scalar(0,100,200), -1);
			}
			for( deque<cv::Point>::const_iterator i=waypoints.begin(); i!=waypoints.end(); i++)
			{
				deque<cv::Point>::const_iterator n = i+1;
				if(n == waypoints.end()) break;
				cv::line(world, *i, *n, cv::Scalar(0,100,200), 2);
			}

//			cout<<"4";cout.flush();
			Path gc_path;
			for( deque<cv::Point>::const_iterator i=waypoints.begin(); i!=waypoints.end(); i++)
			{
				gc_path.push_back(Point2d(i->x, i->y));
			}


//			cout<<"5";cout.flush();
			cv::circle(world, robot, 5, cv::Scalar(0,0,255), -1);
			Point2d gc_robot(robot.x, robot.y);

			map.select_accessable_points(Point2d(robot.x, robot.y));

//			cout<<"6";cout.flush();
			cv::Vec3b blocked(0,0,0), free(255,255,255);
			for(int y=0;y<world.rows;y++)for(int x=0;x<world.cols;x++)
			{
				 if( map.is_accessable_value(map(x,y)) ) 	internal.at<cv::Vec3b>(y,x)=free;
				 else										internal.at<cv::Vec3b>(y,x)=blocked;
			}

//			cout<<"7";cout.flush();
			Point2d goal; Index wp;
			gc.get_goal(map, gc_path, gc_robot, goal, wp);

			cv::circle(world, cv::Point(goal.x, goal.y), 8, cv::Scalar(255,0,255), 2);


			world_change = false;
//			cout<<"}"<<endl;
		}


		cv::imshow("world", world);
		cv::imshow("internal", internal);
		key = cv::waitKey(10);
		if(key>0) cout<<"key = "<<key<<endl;
	}
	while(key!=ESC);


	return 0;
}
