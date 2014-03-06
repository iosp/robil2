/*
 * Tester.cpp
 *
 *  Created on: Mar 6, 2014
 *      Author: dan
 */

#include "Tester.h"
using namespace std;
using namespace ros;
using namespace boost;
using namespace boost::posix_time;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace std_msgs;

Tester::Tester() {
	pub_location = node.advertise<PoseStamped>("/test/location", 10);
	pub_map = node.advertise<OccupancyGrid>("/test/map", 10);
	pub_path = node.advertise<Path>("/test/path", 10);
	pub_command = node.advertise<String>("/test/command", 10);
}

Tester::~Tester() {

}

Checker::Checker(){
	sub_on_goal = node.subscribe("/move_base/goal",10, &Checker::on_goal, this);
}
Checker::~Checker(){

}

OccupancyGrid createMap(){
	OccupancyGrid map;
	map.header.frame_id="map";
	map.header.stamp = ros::Time::now();
	map.info.width=360;
	map.info.height=480;
	map.info.resolution=0.25;
	map.info.origin.position.x=-360/2*map.info.resolution;
	map.info.origin.position.y=-480/2*map.info.resolution;
	map.info.origin.orientation.x=0;
	map.info.origin.orientation.y=0;
	map.info.origin.orientation.z=0;
	map.info.origin.orientation.w=1;
	size_t map_len = map.info.width*map.info.height;
	map.data.resize(map_len);
	for(size_t i=0;i<map_len;i++)
		map.data[i]=0;
	for(size_t y=105;y<200;y++)for(size_t x=150;x<230;x++)
		map.data[y*map.info.width+x]=100;
	return map;
}

PoseStamped createPose(double x, double y){
	PoseStamped pose;
	pose.pose.position.x=x;
	pose.pose.position.y=y;
	pose.pose.position.z=0;
	pose.pose.orientation.x=pose.pose.orientation.y=pose.pose.orientation.z=0;
	pose.pose.orientation.w=1;
	return pose;
}
Path createPath(){
	Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id="map";
	path.poses.push_back(createPose(29.7075042725,-49.105682373));
	path.poses.push_back(createPose(19.9376335144,-43.1056747437));
	path.poses.push_back(createPose(20.9225997925,-27.7453804016));
	path.poses.push_back(createPose(25.5489196777,-10.2032852173));
	path.poses.push_back(createPose(22.6079330444,6.34499359131));
	path.poses.push_back(createPose(6.22555160522,9.78065490723));
	path.poses.push_back(createPose(-12.1595077515,14.9751663208));
	path.poses.push_back(createPose(-23.5211334229,28.2392272949));
	path.poses.push_back(createPose(-32.2888793945,44.5931091309));
	return path;
}
Path createMove(){
	Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id="map";
	path.poses.push_back(createPose(37.5724,-50.082));
	path.poses.push_back(createPose(33.6973,-49.2362));
	path.poses.push_back(createPose(32.2638,-48.443));
	path.poses.push_back(createPose(30.4118,-47.3101));
	path.poses.push_back(createPose(28.2873,-45.7237));
	path.poses.push_back(createPose(25.4941,-44.6778));
	path.poses.push_back(createPose(23.4398,-43.2224));
	path.poses.push_back(createPose(22.2063,-41.9555));
	path.poses.push_back(createPose(22.2179,-39.7622));
	path.poses.push_back(createPose(18.6158,-38.86));
	path.poses.push_back(createPose(18.7159,-34.2964));
	path.poses.push_back(createPose(18.0193,-30.1348));
	path.poses.push_back(createPose(18.7056,-26.3986));
	path.poses.push_back(createPose(19.6115,-22.9968));
	path.poses.push_back(createPose(20.5353,-17.3442));
	path.poses.push_back(createPose(21.936,-10.2276));
	path.poses.push_back(createPose(22.3264,-3.26729));
	path.poses.push_back(createPose(20.2399,3.84317));
	path.poses.push_back(createPose(20.6918,10.4314));
	path.poses.push_back(createPose(15.7382,11.3362));
	path.poses.push_back(createPose(11.5314,13.242));
	path.poses.push_back(createPose(3.21422,12.862));
	path.poses.push_back(createPose(-7.3626,16.5558));
	path.poses.push_back(createPose(-12.1242,19.1083));
	path.poses.push_back(createPose(-18.7633,24.5753));
	path.poses.push_back(createPose(-27.7291,32.4412));
	path.poses.push_back(createPose(-28.5251,39.5861));
	path.poses.push_back(createPose(-35.16,46.1298));
	path.poses.push_back(createPose(-38.8151,52.3665));
	return path;
}

void Tester::test1_init(){

	map = createMap();
	path = createPath();
	move = createMove();
}

int move_index=0 ;
void Tester::test1_step(){

	pub_map.publish(map);
	pub_path.publish(path);
	if(move_index==move.poses.size())move_index=0;
	pub_location.publish(move.poses[move_index++]);
}

void Checker::on_goal(const geometry_msgs::PoseStamped::ConstPtr& msg){
	ROS_INFO_STREAM("Nav goal: "<<*msg);
}



int main(int a ,char** aa){
	ros::init(a, aa, "pp_tester");
	Tester tester;
	Checker checker;
	tester.test1_init();
	while(ros::ok()){
		tester.test1_step();
		this_thread::sleep(seconds(1));
	}
	return 0;
}
