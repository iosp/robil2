/*
 * FsmStatesTracker.h
 *
 *  Created on: Mar 9, 2014
 *      Author: dan
 */

#ifndef FSMSTATESTRACKER_H_
#define FSMSTATESTRACKER_H_

#include <ros/ros.h>
#include <ParameterTypes.h>
#include <map>
#include <vector>
#include <set>
#include <deque>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <robil_msgs/FSMState.h>


#include <diagnostic_msgs/DiagnosticArray.h>


using namespace std;
using namespace ros;
using namespace boost;
using namespace boost::posix_time;

class FsmStatesTracker {
public:
	FsmStatesTracker();
	virtual ~FsmStatesTracker();

	void on_diagnostic_msg(const diagnostic_msgs::DiagnosticArray::ConstPtr msg);
	void on_diagnostic(const diagnostic_msgs::DiagnosticStatus& status);
	void on_states_change(const set<string> old_state,const set<string> new_state);
	void clear(){active_items.clear();}
	void print_actives();

	bool on_ss_actives(robil_msgs::FSMState::Request& req, robil_msgs::FSMState::Response& res);
	bool on_ss_remove(robil_msgs::FSMState::Request& req, robil_msgs::FSMState::Response& res);
	bool on_ss_add(robil_msgs::FSMState::Request& req, robil_msgs::FSMState::Response& res);

	recursive_mutex mtx;
	NodeHandle node;
	Subscriber sub_diagnostic;
	map< string,set<string> > active_items;

	ros::ServiceServer ss_actives;
	ros::ServiceServer ss_remove;
	ros::ServiceServer ss_add;
};

#endif /* FSMCOMPONENTTESTER_H_ */
