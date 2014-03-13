/*
 * MissionTester.h
 *
 *  Created on: Mar 13, 2014
 *      Author: dan
 */

#ifndef MISSIONTESTER_H_
#define MISSIONTESTER_H_

#include <ros/ros.h>
#include <ParameterTypes.h>
#include <ParameterHandler.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <map>
#include <vector>
#include <list>
#include <set>
#include <sstream>
#include <iostream>

using namespace std;
using namespace boost;
using namespace boost::posix_time;
using namespace ros;

typedef config::SMME::sub::AssignNavTask NavTask;
typedef config::SMME::sub::AssignMission Mission;

class MissionTester {
public:
	MissionTester();
	virtual ~MissionTester();

	void test();

	ros::NodeHandle node;
	Publisher p_NavTask;
	Publisher p_Mission;
};

#endif /* MISSIONTESTER_H_ */
