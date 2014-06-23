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


#define CREATE_MAP_FOR_NAV 0
#define CREATE_POINTCLOUD2_FOR_NAV 0
#define CREATE_POINTCLOUD_FOR_NAV 1

#define TH_NEARBY 0.5 //m

#define SYNCH 	boost::recursive_mutex::scoped_lock locker(mtx);

namespace{
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

	geometry_msgs::PoseStamped search_next_waypoint(const nav_msgs::Path& path, const geometry_msgs::PoseWithCovarianceStamped& pos, bool& path_is_finished){
		//cout<<"search_next_waypoint for: "<<pos.pose.pose.position.x<<","<<pos.pose.pose.position.y<<endl;
		if(path.poses.size()==0){
			path_is_finished = true;
			return getPoseStamped( pos );
		}
		if(path.poses.size()==1){
			geometry_msgs::PoseStamped my_pose = getPoseStamped( pos );
			geometry_msgs::PoseStamped path_pose = getPoseStamped( path.poses[0] );
			path_is_finished = toVector(path_pose.pose).distance(toVector(my_pose.pose)) <= TH_NEARBY;
			return path_pose;
		}
		path_is_finished = false;
		size_t ni = search_nearest_waypoint_index(path , pos);
		//cout<<"[i] nearest index = "<<ni<<endl;
		const geometry_msgs::Pose& c = getPose( pos );
		//FIRST POINT
		if(ni==0){
			const geometry_msgs::Pose& b = getPose( path.poses[0] );
			const geometry_msgs::Pose& d = getPose( path.poses[1] );
			double angle_deg = calcTriangle_deg(d,b,c);
			if(angle_deg>90){
				//cout<<"[i] return index = 0"<<endl;
				return getPoseStamped( path.poses[0] );
			}
			ni+=1;
		}
		//OTHERS POINTS
		for(size_t i=ni;i<path.poses.size();i++){
			const geometry_msgs::Pose& a = getPose( path.poses[i-1] );
			const geometry_msgs::Pose& b = getPose( path.poses[i] );
			double angle_deg = calcTriangle_deg(a,b,c);
			if(angle_deg<90){
				//cout<<"[i] return index = "<< i <<endl;
				if(i==path.poses.size()-1){
					geometry_msgs::PoseStamped my_pose = getPoseStamped( pos );
					geometry_msgs::PoseStamped path_pose = getPoseStamped( path.poses[i] );
					path_is_finished = toVector(path_pose.pose).distance(toVector(my_pose.pose)) <= TH_NEARBY;
				}
				return getPoseStamped( path.poses[i] );
			}
		}
		//PATH IS FINISHED
		path_is_finished = true;
		return getPoseStamped( pos );
		//return getPoseStamped( path.poses[path.poses.size()-1] );
	}
}


MoveBase::MoveBase(ComponentMain* comp)
	:gp_defined(false),gnp_defined(false), gl_defined(false), comp(comp)
{
	ros::NodeHandle node;

	goalPublisher = node.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 5, false);
	pathSubscriber = node.subscribe("/move_base/NavfnROS/plan", 10, &MoveBase::on_nav_path, this);

#if CREATE_MAP_FOR_NAV == 1
	mapPublisher = node.advertise<nav_msgs::OccupancyGrid>("/map", 5, false);
#elif CREATE_POINTCLOUD2_FOR_NAV == 1
	mapPublisher = node.advertise<sensor_msgs::PointCloud2>("/map_cloud", 5, false);
#elif CREATE_POINTCLOUD_FOR_NAV == 1
	mapPublisher = node.advertise<sensor_msgs::PointCloud>("/map_cloud", 5, false);
#endif


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
	p.header.frame_id = "/map";//NOTE: check if frame_id not has to be from robil_map.header
	p.header.stamp = ros::Time::now();
	p.pose.pose = msg->pose;
	this->on_position_update(p);
	//ROS_INFO_STREAM("}on_sub_loc");
}
void MoveBase::on_sub_loc_cov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	//ROS_INFO_STREAM("on_sub_loc_cov{");
	//std::cout<<"path.poses.push_back(createPose("<< msg->pose.pose.position.x<<","<< msg->pose.pose.position.y<<"));"<<std::endl;
	geometry_msgs::PoseWithCovarianceStamped p = *msg;
	p.header.frame_id = "/map";//NOTE: check if frame_id not has to be from robil_map.header
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

void MoveBase::notify_path_is_finished()const{
	comp->rise_taskFinished();
}

void MoveBase::on_position_update(const config::PP::sub::Location& location){
SYNCH
	ROS_INFO_STREAM("\t on_position_update( "<<location.pose.pose.position.x<<","<<location.pose.pose.position.y<<" )");
	gotten_location = location;
	gl_defined=true;
	if(all_data_defined()) calculate_goal();
}

void MoveBase::on_path(const config::PP::sub::GlobalPath& goal_path){
SYNCH
	ROS_INFO_STREAM("\t on_path( "<<goal_path.waypoints.poses.size()<<" )");
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

void MoveBase::on_nav_path(const nav_msgs::Path& nav_path){
	//SYNCH
	config::PP::pub::LocalPath lpath;
	lpath.is_heading_defined=false;
	lpath.is_ip_defined=false;
	lpath.waypoints = nav_path;
	comp->publishLocalPath(lpath);
}

void MoveBase::calculate_goal(){
	geometry_msgs::PoseStamped goal;
	bool is_path_finished;
	if(gp_defined){
		goal = search_next_waypoint(gotten_path.waypoints, gotten_location, is_path_finished);
	}else{
		goal = search_next_waypoint(gotten_nav_path, gotten_location, is_path_finished);
	}
	if(is_path_finished) notify_path_is_finished();
	on_goal(goal);
}

void MoveBase::on_goal(const geometry_msgs::PoseStamped& robil_goal){

	move_base_msgs::MoveBaseActionGoal goal;
	geometry_msgs::PoseStamped ps_goal;

	ps_goal = robil_goal;
	remove_orientation(ps_goal);

	goal.header.frame_id = "/map";
	goal.header.stamp = ros::Time::now();
	ps_goal.header = goal.header;

	std::stringstream sid ; sid<<"[i] goal : "<<ps_goal.pose.position.x<<","<<ps_goal.pose.position.y;
	goal.goal_id.id = sid.str();
	goal.goal_id.stamp = goal.header.stamp;
	
	goal.goal.target_pose = ps_goal;
	
	std::cout<<"goal : "<<ps_goal.pose.position.x<<","<<ps_goal.pose.position.y<<std::endl;
	goalPublisher.publish(goal);

}

void MoveBase::on_map(const nav_msgs::OccupancyGrid& nav_map){
SYNCH
	nav_msgs::OccupancyGrid map = nav_map;
	mapPublisher.publish(map);
}


void MoveBase::on_map(const config::PP::sub::Map& robil_map){
SYNCH

#if CREATE_MAP_FOR_NAV == 1
	nav_msgs::OccupancyGrid map;

	map.info = robil_map.info;
	//PATCH->
	map.info.origin.position.x =0;//+= map.info.width*map.info.resolution/2.0;
	map.info.origin.position.y =0;//+= map.info.height*map.info.resolution/2.0;
//	map.info.origin.position.z =0;
	tf::Quaternion q(0,0,0,1);
//	#define setQ(M) q=tf::Quaternion(M.x,M.y,M.z,M.w);
//	setQ(map.info.origin.orientation);
//	#undef setQ
//	q.setRotation(tf::Vector3(0,0,1),M_PI_2);
	#define setQ(M) M.x=q.x();M.y=q.y();M.z=q.z();M.w=q.w();
	setQ(map.info.origin.orientation);
	#undef setQ

	//<-PATCH

	map.data.resize(map.info.width * map.info.height, 0);
	for(size_t i=0;i<map.data.size();i++){
		if(robil_map.data[i].type==robil_msgs::MapCell::type_obstacle or rand()%5>2){
			map.data[i] = 100;
		}else
		if(robil_map.data[i].type==robil_msgs::MapCell::type_clear){
			map.data[i] = 0;
		}else
		if(robil_map.data[i].type==robil_msgs::MapCell::type_unscanned){
			map.data[i] = -1;
		}
	}

	map.header.frame_id = "/map_fit";//NOTE: check if frame_id not has to be from robil_map.header
	map.header.stamp = robil_map.header.stamp;//ros::Time::now();






	{
#define SET(N,X)\
	tf::Quaternion _q(X.orientation.x, X.orientation.y, X.orientation.z, X.orientation.w);\
	tf::Quaternion q1; \
	q1.setRotation(tf::Vector3(0,0,1),M_PI_2); \
	tf::Vector3 _l(X.position.x-map.info.width*map.info.resolution/3.0*2.0,X.position.y-map.info.height*map.info.resolution/2.0,X.position.z);\
	tf::Transform N(_q,_l);
	SET( tf_map, map.info.origin )
#undef SET
	comp->publishTransform(tf_map,"map_fit_rotation", "map_fit");
	}

	{
#define SET(N,X)\
	tf::Quaternion _q(X.orientation.x, X.orientation.y, X.orientation.z, X.orientation.w);\
	tf::Quaternion q1; \
	q1.setRotation(tf::Vector3(0,0,1),M_PI); _q=q1*_q;\
	tf::Vector3 _l(X.position.x,X.position.y,X.position.z);\
	tf::Transform N(_q,_l);
	SET( tf_map, map.info.origin )
#undef SET
	comp->publishTransform(tf_map,"base_link", "map_fit_rotation");
	}

	mapPublisher.publish(map);

#elif CREATE_POINTCLOUD2_FOR_NAV == 1

	sensor_msgs::PointCloud2 map;

	map.is_dense=true;
	map.is_bigendian=false;

	map.point_step=12;
	map.height=robil_map.info.height;
	map.width=robil_map.info.width;
	map.row_step=robil_map.info.width * map.point_step;

	map.fields.resize(3);
	{
		map.fields[0].name="x";
		map.fields[0].offset=0;
		map.fields[0].datatype=7;
		map.fields[0].count=1;

		map.fields[1].name="y";
		map.fields[1].offset=4;
		map.fields[1].datatype=7;
		map.fields[1].count=1;

		map.fields[2].name="z";
		map.fields[2].offset=8;
		map.fields[2].datatype=7;
		map.fields[2].count=1;
	}

	map.data.clear();
	map.data.resize(map.height*map.row_step);
	unsigned char* map_data = map.data.data();
	float _fz = -10;
	for(
			size_t y=0;
			y<robil_map.info.height;
			y++
	){
		float fy = y*robil_map.info.resolution;
		for(
				size_t x=0;
				x<robil_map.info.width;
				x++,	map_data+=map.point_step
		){
			size_t i = y*robil_map.info.width+x;
			float fx = x*robil_map.info.resolution;
			float fz = 0;

			if(robil_map.data[i].type==robil_msgs::MapCell::type_obstacle /*or rand()%10>8*/){
				fz = 1;
			}else
			if(robil_map.data[i].type==robil_msgs::MapCell::type_clear){
				fz = 0;
			}else
			if(robil_map.data[i].type==robil_msgs::MapCell::type_unscanned){
				fz = -1;
			}

			_fz+=0.1;
			if(_fz>10)_fz=-10;

			memcpy(map_data+map.fields[0].offset, &fx,sizeof(fx));
			memcpy(map_data+map.fields[1].offset, &fy,sizeof(fy));
			memcpy(map_data+map.fields[2].offset, &fz,sizeof(fz));
		}
	}

	map.header.frame_id = "/map_fit";
	map.header.stamp = ros::Time::now();//robil_map.header.stamp;//

	{
#define SET(N,X)\
	tf::Quaternion _q(0,0,0,1);\
	tf::Quaternion q1; \
	q1.setRotation(tf::Vector3(0,0,1),M_PI_2); \
	tf::Vector3 _l(X.position.x-robil_map.info.width*robil_map.info.resolution/3.0*2.0,X.position.y-robil_map.info.height*robil_map.info.resolution/2.0,0);\
	tf::Transform N(_q,_l);
	SET( tf_map, robil_map.info.origin )
#undef SET
	comp->publishTransform(tf_map,"map_fit_rotation", "map_fit");
	}

	{
#define SET(N,X)\
	tf::Quaternion _q(0,0,0,1);\
	tf::Quaternion q1; \
	q1.setRotation(tf::Vector3(0,0,1),M_PI); _q=q1*_q;\
	tf::Vector3 _l(0,0,0);\
	tf::Transform N(_q,_l);
	SET( tf_map, robil_map.info.origin )
#undef SET
	comp->publishTransform(tf_map,"base_link", "map_fit_rotation");
	}

	mapPublisher.publish(map);

#elif CREATE_POINTCLOUD_FOR_NAV == 1

	sensor_msgs::PointCloud map;
	tf::Transform tf_map_fit;
	tf::Transform tf_map_rotation;
	{
#define SET(N,X)\
	tf::Quaternion _q(0,0,0,1);\
	tf::Quaternion q1; \
	q1.setRotation(tf::Vector3(0,0,1),M_PI_2); \
	tf::Vector3 _l(X.position.x-robil_map.info.width*robil_map.info.resolution/3.0*2.0,X.position.y-robil_map.info.height*robil_map.info.resolution/2.0,0);\
	tf::Transform N(_q,_l);
	SET( tf_map, robil_map.info.origin )
#undef SET
	comp->publishTransform(tf_map,"map_fit_rotation", "map_fit");
	tf_map_fit = tf_map;
	}

	{
#define SET(N,X)\
	tf::Quaternion _q(0,0,0,1);\
	tf::Quaternion q1; \
	q1.setRotation(tf::Vector3(0,0,1),M_PI); _q=q1*_q;\
	tf::Vector3 _l(0,0,0);\
	tf::Transform N(_q,_l);
	SET( tf_map, robil_map.info.origin )
#undef SET
	comp->publishTransform(tf_map,"base_link", "map_fit_rotation");
	tf_map_rotation = tf_map;
	}


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
				fz = 0;
			}else
			if(robil_map.data[i].type==robil_msgs::MapCell::type_clear){
				continue;
				fz = 0;
			}else
			if(robil_map.data[i].type==robil_msgs::MapCell::type_unscanned){
				continue;
				fz = -1;
			}
			geometry_msgs::Point32 point;
			point.x = fx;
			point.y = fy;
			point.z = fz;
			map.points.push_back(point);
		}
	}

	map.header.frame_id = "/map_fit";
	map.header.stamp = ros::Time::now();//robil_map.header.stamp;//


	mapPublisher.publish(map);
#endif


}
