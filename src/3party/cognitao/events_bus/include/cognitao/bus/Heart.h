/*
 * Heart.h
 *
 *  Created on: Nov 9, 2014
 *      Author: dan
 */

#ifndef HEART_H_
#define HEART_H_

#include <ros/ros.h>
#include <boost/thread.hpp>

namespace cognitao{
namespace monitor{

class Heart {
	ros::NodeHandle& node;
	ros::Publisher heartbeat;
	boost::thread_group th;
	double freq;
	bool use_uniq_id;
public:
	std::string uniq_name;
	Heart(ros::NodeHandle& node, bool use_uniq_id=true);
	virtual ~Heart();
	void beating();
};

}
}

#endif /* HEART_H_ */
