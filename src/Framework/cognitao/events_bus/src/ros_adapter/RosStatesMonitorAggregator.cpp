/*
 * RosStatesMonitorAggregator.cpp
 *
 *  Created on: Jan 13, 2016
 *      Author: misha
 */
#include "../../include/cognitao/bus/RosStatesMonitorAggregator.h"


namespace aggregator {

	RosStatesMonitorAggregator::RosStatesMonitorAggregator (ros::NodeHandle & node)
	: node (node)
	, pattern ("/agents/.*/heartbeat.*")
	, agent_pose (3)
	, client_postfix ("states_monitor/raw_status")
	, aggregated_states_service_name ("/get_aggregated_states")
	{
		load_param();
		reset_aggregated_states();
		aggregated_states_service = node.advertiseService(aggregated_states_service_name, &RosStatesMonitorAggregator::on_ask_aggregated_states, this);
		boost::thread process_thread (&RosStatesMonitorAggregator::process, this);
	}


	void RosStatesMonitorAggregator::load_param (){
		if (node.getParam("input_topic_pattern", pattern)){
			ROS_INFO("input_topic_pattern is set to %s", pattern.c_str());
		} else{
			ROS_WARN("input_topic_pattern is set by default to %s", pattern.c_str());
		}

		if (node.getParam("agent_id_pose", agent_pose)){
			ROS_INFO("agent_id_pose is set to %i", agent_pose);
		} else{
			ROS_WARN("agent_id_pose is set by default to %i", agent_pose);
		}

		if (node.getParam("service_client_postfix", client_postfix)){
			ROS_INFO("service_client_postfix is set to %s", client_postfix.c_str());
		} else{
			ROS_WARN("service_client_postfix is set by default to %s", client_postfix.c_str());
		}

		if (node.getParam("aggregated_states_service_name", aggregated_states_service_name)){
			ROS_INFO("aggregated_states_service_name is set to %s", aggregated_states_service_name.c_str());
		} else{
			ROS_WARN("aggregated_states_service_name is set by default to %s", aggregated_states_service_name.c_str());
		}
	}



	bool RosStatesMonitorAggregator::on_ask_aggregated_states(events_bus::Srv_StatesMonitor_RawStatus::Request& req, events_bus::Srv_StatesMonitor_RawStatus::Response& res){
		res = get_aggregated_states();
		return true;
	}


	bool RosStatesMonitorAggregator::test_by_pattern (std::string topic, std::string pattern){
		boost::cmatch what;
		boost::regex expression(pattern);
		return boost::regex_match(topic.c_str(), what, expression);
	}


	std::string RosStatesMonitorAggregator::get_agent_id (std::string data, int agent_pos){
		std::vector<std::string> data_splitted;
		boost::split(data_splitted, data, boost::is_any_of("/"));
		if (agent_pos >= data_splitted.size()){
			ROS_ERROR ("Cannot find the agent id, wrong pose or string");
			return "NULL";
		}

		return data_splitted[agent_pos];
	}

	std::set<std::string> RosStatesMonitorAggregator::get_agent_list(const std::set<std::string> & topic_list){
		std::set<std::string>::iterator it;
		std::set<std::string> agent_list;
		for (it = topic_list.begin(); it != topic_list.end(); ++it)
		{
			std::string topic = *it;
			std::string agent_id = get_agent_id (topic, agent_pose);
			if (agent_id.compare ("NULL") != 0){
				agent_list.insert(agent_id);
			}
		}
		return agent_list;
	}



	std::set<std::string> RosStatesMonitorAggregator::get_topic_list(){
		ros::master::V_TopicInfo master_topics;
		ros::master::getTopics(master_topics);
		std::set<std::string> topic_list;

		for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
			const ros::master::TopicInfo& info = *it;
			if( "std_msgs/Empty" != info.datatype ) continue;
			if(test_by_pattern(info.name, pattern)) {
				topic_list.insert(info.name);
			}
		}

		return topic_list;
	}


	std::set<std::string> RosStatesMonitorAggregator::get_client_id_list (const std::set<std::string> & agent_list){
		std::set<std::string> client_id_list;
		std::set<std::string>::iterator it;
		for (it = agent_list.begin(); it != agent_list.end(); ++it){
			std::string agent_id= *it;
			std::stringstream client_id_ss;
			client_id_ss << "/" << agent_id << "/" << client_postfix;
			client_id_list.insert(client_id_ss.str());
		}
		return client_id_list;
	}


	ClientMap RosStatesMonitorAggregator::get_client_map (const std::set<std::string> & client_id_list){
		ClientMap client_map;
		BOOST_FOREACH (std::string client_id, client_id_list)
		{
			client_map[client_id] = (node.serviceClient<events_bus::Srv_StatesMonitor_RawStatus>(client_id));
		}
		return client_map;
	}

	void RosStatesMonitorAggregator::set_aggregated_states (const events_bus::Srv_StatesMonitor_RawStatus::Response & new_aggregated_states){
		boost::mutex::scoped_lock lock(io_mutex);
		aggregated_states = new_aggregated_states;
	}

	events_bus::Srv_StatesMonitor_RawStatus::Response RosStatesMonitorAggregator::get_aggregated_states (){
		boost::mutex::scoped_lock lock(io_mutex);
		return aggregated_states;
	}

	void RosStatesMonitorAggregator::reset_aggregated_states (){
		boost::mutex::scoped_lock lock(io_mutex);
		aggregated_states.results.clear();
		aggregated_states.states.clear();
	}


	void RosStatesMonitorAggregator::process (){
		while(ros::ok()){
			std::set<std::string> topic_list = get_topic_list();
			std::set<std::string> agent_list = get_agent_list (topic_list);
			std::set<std::string> client_id_list = get_client_id_list (agent_list);
			ClientMap client_map = get_client_map (client_id_list);

			events_bus::Srv_StatesMonitor_RawStatus::Response new_aggregated_states;

			BOOST_FOREACH (std::string client_id, client_id_list)
			{
				events_bus::Srv_StatesMonitor_RawStatus srv;
				std::string current_agent_id = *agent_list.begin();
				agent_list.erase(agent_list.begin());
				if (client_map[client_id].call(srv)){
					for (int i = 0; i < srv.response.states.size(); i++){
						events_bus::Srv_StatesMonitor_RawStatus::Response::_states_type::value_type current_state;
						current_state = srv.response.states[i];
						current_state.name = current_agent_id + srv.response.states[i].name;
						new_aggregated_states.states.push_back(current_state);
					}
					for (int i = 0; i < srv.response.results.size(); i++){
						events_bus::Srv_StatesMonitor_RawStatus::Response::_results_type::value_type current_result;
						current_result = srv.response.results[i];
						current_result.name = current_agent_id + srv.response.results[i].name;
						new_aggregated_states.results.push_back(current_result);
					}
				}
			}

			set_aggregated_states (new_aggregated_states);

			boost::this_thread::sleep(boost::posix_time::millisec(100));
		}
	}


}



