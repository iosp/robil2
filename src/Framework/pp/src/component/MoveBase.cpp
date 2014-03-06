/*
 * MoveBase.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: dan
 */

#include "MoveBase.h"
#include <Geometry.h>


MoveBase::MoveBase()
	:gp_defined(false),gl_defined(false)
{
	ros::NodeHandle node;

	goalPublisher = node.advertise<geometry_msgs::PoseStamped>("/move_base/goal", 5, false);
	mapPublisher = node.advertise<nav_msgs::OccupancyGrid>("/map", 5, false);
}

MoveBase::~MoveBase() {

}

void MoveBase::on_position_update(const config::PP::sub::Location& location){
	gotten_location = location;
	gl_defined=true;
	if(gl_defined and gp_defined) calculate_goal();
}

void MoveBase::on_path(const config::PP::sub::GlobalPath& goal_path){
	gotten_path = goal_path;
	gp_defined=true;
	if(gl_defined and gp_defined) calculate_goal();
}

namespace{

	double calcTriangle_deg(
			const geometry_msgs::Pose& A,
			const geometry_msgs::Pose& C,
			const geometry_msgs::Pose& B
	){
		btVector3 vA = toVector(A);
		btVector3 vB = toVector(B);
		btVector3 vC = toVector(C);
		double a = vC.distance(vB);//CB
		double c = vA.distance(vB);//AB
		double b = vA.distance(vC);//AC
		double gamma;// \_ ACB
		gamma = acos( (a*a + b*b - c*c)/(2*a*b) );
		return gamma;
	}
	geometry_msgs::PoseStamped search_waypoint(const nav_msgs::Path& path, const geometry_msgs::PoseWithCovarianceStamped& pos){
		if(path.poses.size()==0) return getPoseStamped( pos );
		if(path.poses.size()==1) return getPoseStamped( path.poses[0] );
		const geometry_msgs::Pose& c = getPose( pos );
		//FIRST POINT
		{
			const geometry_msgs::Pose& b = getPose( path.poses[0] );
			const geometry_msgs::Pose& d = getPose( path.poses[1] );
			double angle = calcTriangle_deg(d,b,c);
			if(angle>90) return getPoseStamped( path.poses[0] );
		}
		//OTHERS POINTS
		for(size_t i=1;i<path.poses.size();i++){
			const geometry_msgs::Pose& a = getPose( path.poses[i-1] );
			const geometry_msgs::Pose& b = getPose( path.poses[i] );
			double angle = calcTriangle_deg(a,b,c);
			if(angle<90) return getPoseStamped( path.poses[i] );
		}
		//PATH IS FINISHED
		return getPoseStamped( pos );
		//return path.poses[path.poses.size()-1];
	}
}

void MoveBase::calculate_goal(){
	geometry_msgs::PoseStamped goal;
	goal = search_waypoint(gotten_path.waypoints, gotten_location);
	on_goal(goal);
}

namespace{
	void remove_orientation(geometry_msgs::PoseStamped& goal){
		goal.pose.orientation.x = 0;
		goal.pose.orientation.y = 0;
		goal.pose.orientation.z = 0;
		goal.pose.orientation.w = 0;
	}
}

void MoveBase::on_goal(const geometry_msgs::PoseStamped& robil_goal){

	geometry_msgs::PoseStamped goal;

	goal = robil_goal;
	remove_orientation(goal);

	goal.header.frame_id = "/map";
	goal.header.stamp = ros::Time::now();

	goalPublisher.publish(goal);

}

void MoveBase::on_map(const config::PP::sub::Map& robil_map){

	nav_msgs::OccupancyGrid map;

//INFO EXAMPLE
//	map.info.width = 2048;
//	map.info.height = 2048;
//	map.info.resolution = 0.05;
//
//	map.info.origin.position.x = -51.2249984741; // Center map
//	map.info.origin.position.y = -51.2249984741; // Center map
//
//	// No rotation
//	map.info.origin.orientation.x = 0;
//	map.info.origin.orientation.y = 0;
//	map.info.origin.orientation.z = 0;
//	map.info.origin.orientation.w = 1;

	map.info = robil_map.info;

	map.data.resize(map.info.width * map.info.height, 0);
	for(size_t i=0;i<map.data.size();i++){
		if(robil_map.data[i].type==robil_msgs::MapCell::type_obstacle){
			map.data[i] = 100;
		}else
		if(robil_map.data[i].type==robil_msgs::MapCell::type_clear){
			map.data[i] = 0;
		}else
		if(robil_map.data[i].type==robil_msgs::MapCell::type_unscanned){
			map.data[i] = -1;
		}
	}

	map.header.frame_id = "map";//NOTE: check if frame_id not has to be from robil_map.header
	map.header.stamp = robil_map.header.stamp;//ros::Time::now();

	mapPublisher.publish(map);
}
