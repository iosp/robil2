/*
 * RosStatesMonitorAggregator.h
 *
 *  Created on: Jan 13, 2016
 *      Author: misha
 */

#ifndef INCLUDE_COGNITAO_BUS_ROSSTATESMONITORAGGREGATOR_H_
#define INCLUDE_COGNITAO_BUS_ROSSTATESMONITORAGGREGATOR_H_

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <events_bus/Srv_StatesMonitor_RawStatus.h>
#include <boost/foreach.hpp>

namespace aggregator{

typedef std::map<std::string,ros::ServiceClient> ClientMap;


class RosStatesMonitorAggregator {
private:
	ros::NodeHandle & node;
	std::string pattern;
	std::string client_postfix;
	int agent_pose;
	events_bus::Srv_StatesMonitor_RawStatus::Response aggregated_states;
	std::string aggregated_states_service_name;
	boost::mutex io_mutex;
	ros::ServiceServer aggregated_states_service;
public:
	RosStatesMonitorAggregator (ros::NodeHandle & node);
	bool test_by_pattern (std::string topic, std::string pattern);
	std::string get_agent_id (std::string line, int agent_pos);
	std::set<std::string> get_topic_list();
	std::set<std::string> get_agent_list(const std::set<std::string> & topic_list);
	std::set<std::string> get_client_id_list (const std::set<std::string> & agent_list);
	ClientMap get_client_map (const std::set<std::string> & client_id_list);
	void process ();
	void set_aggregated_states (const events_bus::Srv_StatesMonitor_RawStatus::Response & new_aggregated_states);
	events_bus::Srv_StatesMonitor_RawStatus::Response get_aggregated_states ();
	void reset_aggregated_states();
	void load_param ();
	bool on_ask_aggregated_states(events_bus::Srv_StatesMonitor_RawStatus::Request& req, events_bus::Srv_StatesMonitor_RawStatus::Response& res);
};






}

#endif /* INCLUDE_COGNITAO_BUS_ROSSTATESMONITORAGGREGATOR_H_ */
