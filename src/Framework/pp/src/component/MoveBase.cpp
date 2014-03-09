/*
 * MoveBase.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: dan
 */

#include "MoveBase.h"
#include <Geometry.h>

#define SYNCH 	boost::recursive_mutex::scoped_lock locker(mtx);

namespace{
	void remove_orientation(geometry_msgs::PoseStamped& goal){
		goal.pose.orientation.x = 0;
		goal.pose.orientation.y = 0;
		goal.pose.orientation.z = 0;
		goal.pose.orientation.w = 0;
	}
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
		return angles::to_degrees( angles::normalize_angle_positive( gamma ) );
	}
	size_t search_nearest_waypoint_index(const nav_msgs::Path& path, const geometry_msgs::PoseWithCovarianceStamped& pos){
		btVector3 vPose = toVector(getPose(pos));
		double min_idx=0;
		double min_distance=toVector(getPose(path.poses[0])).distance(vPose);
		for(size_t i=0;i<path.poses.size();i++){
			double c_distance=toVector(getPose(path.poses[i])).distance(vPose);
			if(c_distance<min_distance){
				min_distance = c_distance;
				min_idx = i;
			}
		}
		return min_idx;
	}
	geometry_msgs::PoseStamped search_next_waypoint(const nav_msgs::Path& path, const geometry_msgs::PoseWithCovarianceStamped& pos){
		if(path.poses.size()==0) return getPoseStamped( pos );
		if(path.poses.size()==1) return getPoseStamped( path.poses[0] );
		size_t ni = search_nearest_waypoint_index(path , pos);
		const geometry_msgs::Pose& c = getPose( pos );
		//FIRST POINT
		if(ni==0){
			const geometry_msgs::Pose& b = getPose( path.poses[0] );
			const geometry_msgs::Pose& d = getPose( path.poses[1] );
			double angle_deg = calcTriangle_deg(d,b,c);
			if(angle_deg>90) return getPoseStamped( path.poses[0] );
			ni+=1;
		}
		//OTHERS POINTS
		for(size_t i=ni;i<path.poses.size();i++){
			const geometry_msgs::Pose& a = getPose( path.poses[i-1] );
			const geometry_msgs::Pose& b = getPose( path.poses[i] );
			double angle_deg = calcTriangle_deg(a,b,c);
			if(angle_deg<90) return getPoseStamped( path.poses[i] );
		}
		//PATH IS FINISHED
		return getPoseStamped( pos );
		//return getPoseStamped( path.poses[path.poses.size()-1] );
	}
}


MoveBase::MoveBase()
	:gp_defined(false),gnp_defined(false), gl_defined(false)
{
	ros::NodeHandle node;

	goalPublisher = node.advertise<geometry_msgs::PoseStamped>("/move_base/goal", 5, false);
	mapPublisher = node.advertise<nav_msgs::OccupancyGrid>("/map", 5, false);


//	//FOR TEST
	sub_location = node.subscribe("/test/location", 10, &MoveBase::on_sub_loc, this);
	sub_location_cov = node.subscribe("/test/location_cov", 10, &MoveBase::on_sub_loc_cov, this);
	sub_map = node.subscribe("/test/map", 10, &MoveBase::on_sub_map, this);
	sub_path = node.subscribe("/test/path", 10, &MoveBase::on_sub_path, this);
	sub_commands = node.subscribe("/test/command", 10, &MoveBase::on_sub_commands, this);
}

//================ TEST =================
void MoveBase::on_sub_map(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	//ROS_INFO_STREAM("on_sub_map{");
	this->on_map(*msg);
	//ROS_INFO_STREAM("}on_sub_map");
}
void MoveBase::on_sub_path(const nav_msgs::Path::ConstPtr& msg){
	//ROS_INFO_STREAM("on_sub_path{");
	this->on_path(*msg);
	//ROS_INFO_STREAM("}on_sub_path");
}
void MoveBase::on_sub_loc(const geometry_msgs::PoseStamped::ConstPtr& msg){
	//ROS_INFO_STREAM("on_sub_loc{");
	geometry_msgs::PoseWithCovarianceStamped p;
	p.header.frame_id = "map";//NOTE: check if frame_id not has to be from robil_map.header
	p.header.stamp = ros::Time::now();
	p.pose.pose = msg->pose;
	this->on_position_update(p);
	//ROS_INFO_STREAM("}on_sub_loc");
}
void MoveBase::on_sub_loc_cov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	//ROS_INFO_STREAM("on_sub_loc_cov{");
	//std::cout<<"path.poses.push_back(createPose("<< msg->pose.pose.position.x<<","<< msg->pose.pose.position.y<<"));"<<std::endl;
	geometry_msgs::PoseWithCovarianceStamped p = *msg;
	p.header.frame_id = "map";//NOTE: check if frame_id not has to be from robil_map.header
	this->on_position_update(p);
	//ROS_INFO_STREAM("}on_sub_loc_cov");
}
void MoveBase::on_sub_commands(const std_msgs::String::ConstPtr& msg){
	//ROS_INFO_STREAM("on_sub_commands{");
	std::string command = msg->data;
	if(command == "clear"){
		ROS_INFO_STREAM("\t on command CLEAR");
		this->gl_defined = false;
		this->gnp_defined= false;
		this->gp_defined = false;
	}
	//ROS_INFO_STREAM("on_sub_commands{");
}
//=======================================

MoveBase::~MoveBase() {

}

bool MoveBase::all_data_defined()const{
	return gl_defined and (gp_defined or gnp_defined);
}

void MoveBase::on_position_update(const config::PP::sub::Location& location){
SYNCH
	//ROS_INFO_STREAM("\t on_position_update( "<<location<<" )");
	gotten_location = location;
	gl_defined=true;
	if(all_data_defined()) calculate_goal();
}

void MoveBase::on_path(const config::PP::sub::GlobalPath& goal_path){
SYNCH
	gotten_path = goal_path;
	gp_defined=true;
	if(all_data_defined()) calculate_goal();
}
void MoveBase::on_path(const nav_msgs::Path& goal_path){
SYNCH
	gotten_nav_path = goal_path;
	gnp_defined=true;
	if(all_data_defined()) calculate_goal();
}

void MoveBase::calculate_goal(){
	geometry_msgs::PoseStamped goal;
	if(gp_defined){
		goal = search_next_waypoint(gotten_path.waypoints, gotten_location);
	}else{
		goal = search_next_waypoint(gotten_nav_path, gotten_location);
	}
	on_goal(goal);
}

void MoveBase::on_goal(const geometry_msgs::PoseStamped& robil_goal){

	geometry_msgs::PoseStamped goal;

	goal = robil_goal;
	remove_orientation(goal);

	goal.header.frame_id = "/map";
	goal.header.stamp = ros::Time::now();

	std::cout<<"goal : "<<goal.pose.position.x<<","<<goal.pose.position.y<<std::endl;
	goalPublisher.publish(goal);

}

void MoveBase::on_map(const nav_msgs::OccupancyGrid& nav_map){
SYNCH
	nav_msgs::OccupancyGrid map = nav_map;
	mapPublisher.publish(map);
}
void MoveBase::on_map(const config::PP::sub::Map& robil_map){
SYNCH
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
