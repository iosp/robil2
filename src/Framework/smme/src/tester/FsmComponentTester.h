/*
 * FsmComponentTester.h
 *
 *  Created on: Mar 9, 2014
 *      Author: dan
 */

#ifndef FSMCOMPONENTTESTER_H_
#define FSMCOMPONENTTESTER_H_

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


#include <std_msgs/String.h>
#include <diagnostic_msgs/DiagnosticArray.h>


using namespace std;
using namespace ros;
using namespace boost;
using namespace boost::posix_time;

class FsmComponentTester {
public:
	FsmComponentTester();
	virtual ~FsmComponentTester();

	void on_diagnostic_msg(const diagnostic_msgs::DiagnosticArray::ConstPtr msg);
	void on_diagnostic(const diagnostic_msgs::DiagnosticStatus& status);
	void on_states_change(const set<string> old_state,const set<string> new_state);
	void send(std::string e);
	bool test1(bool);
	bool test2(bool);
	bool test3(bool);
	bool test4(bool);
	bool test5(bool);
	bool test(bool);
	void start();
	void clear(){active_items.clear();}
	void print_actives();
	void init();

	recursive_mutex mtx;
	NodeHandle node;
	Subscriber sub_diagnostic;
	Publisher pub_events;
	string target_component;
	bool all;
	map< string,set<string> > active_items;
};

#endif /* FSMCOMPONENTTESTER_H_ */
