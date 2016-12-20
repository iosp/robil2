/*
 * RosStatesMonitor.h
 *
 *  Created on: Nov 9, 2014
 *      Author: dan
 */

#ifndef ROSSTATESMONITOR_H_
#define ROSSTATESMONITOR_H_


#include <ros/ros.h>
#include <cognitao/bus/StatesMonitor.h>

#include <events_bus/Srv_StatesMonitor_Is.h>
#include <events_bus/Srv_StatesMonitor_Result.h>
#include <events_bus/Srv_StatesMonitor_States.h>
#include <events_bus/Srv_StatesMonitor_RawStatus.h>
#include <events_bus/Srv_StatesMonitor_Status.h>
#include <events_bus/Srv_StatesMonitor_Times.h>
#include <events_bus/Srv_StatesMonitor_EndStates.h>
#include <cognitao/bus/EventQueue.h>
#include <std_srvs/Empty.h>

namespace node{

using namespace cognitao;
using namespace cognitao::bus;
using namespace cognitao::monitor;


	class RosStatesMonitor {
		ros::NodeHandle& node;
		StatesMonitor monitor;
		EventQueue events;

		ros::ServiceServer ss_status;
		ros::ServiceServer ss_raw_status;
		ros::ServiceServer ss_is_active;
		ros::ServiceServer ss_is_exists;
		ros::ServiceServer ss_result;
		ros::ServiceServer ss_times;
		ros::ServiceServer ss_search;
		ros::ServiceServer ss_actives;
		ros::ServiceServer ss_end_states;
		ros::ServiceServer ss_clear_history;

	public:
		RosStatesMonitor(ros::NodeHandle& n, cognitao::bus::EventQueue& events);
		virtual ~RosStatesMonitor();

		void process();

		//get output message with full status of monitor
		string status()const;
		events_bus::Srv_StatesMonitor_RawStatus::Response raw_status()const;

		//check if state is active
		bool is_active(const string& name)const;
		bool is_active(const string& name, string& full_name)const;

		//check if state was active sometime
		bool is_exists(const string& name)const;
		bool is_exists(const string& name, string& full_name)const;

		//get last result of state
		string result(const string& name)const;

		//get times (begin, end, freq, etc) of state
		struct TimesInfo{
			ptime begin;
			ptime end;
			time_duration duration;
			double frequency;
			string result;
			enum MASK{ BEGIN=0x1,END=0x2,DURATION=0x4,FREQUENCY=0x8,RESULT=0xa } mask;
		};
		TimesInfo times(const string& name)const;

		//search states
		set<string> search(const string& state)const;

		//get all active states
		set<string> actives(const string& state)const;

		void end_states(const string& state, const string& result);
		void clear_history(const string& state);

		bool on_ask_status(events_bus::Srv_StatesMonitor_Status::Request& req, events_bus::Srv_StatesMonitor_Status::Response& res);
		bool on_ask_raw_status(events_bus::Srv_StatesMonitor_RawStatus::Request& req, events_bus::Srv_StatesMonitor_RawStatus::Response& res);
		bool on_ask_is_active(events_bus::Srv_StatesMonitor_Is::Request& req, events_bus::Srv_StatesMonitor_Is::Response& res);
		bool on_ask_is_exists(events_bus::Srv_StatesMonitor_Is::Request& req, events_bus::Srv_StatesMonitor_Is::Response& res);
		bool on_ask_result(events_bus::Srv_StatesMonitor_Result::Request& req, events_bus::Srv_StatesMonitor_Result::Response& res);
		bool on_ask_times(events_bus::Srv_StatesMonitor_Times::Request& req, events_bus::Srv_StatesMonitor_Times::Response& res);
		bool on_ask_search(events_bus::Srv_StatesMonitor_States::Request& req, events_bus::Srv_StatesMonitor_States::Response& res);
		bool on_ask_actives(events_bus::Srv_StatesMonitor_States::Request& req, events_bus::Srv_StatesMonitor_States::Response& res);
		bool on_ask_end_states(events_bus::Srv_StatesMonitor_EndStates::Request& req, events_bus::Srv_StatesMonitor_EndStates::Response& res);
		bool on_ask_clear_history(events_bus::Srv_StatesMonitor_States::Request& req, events_bus::Srv_StatesMonitor_States::Response& res);

	};
}

namespace cognitao{
namespace monitor{

	class StatesMonitorClient {
		ros::NodeHandle& node;
		//EventQueue events;

		mutable ros::ServiceClient sc_status;
		mutable ros::ServiceClient sc_is_active;
		mutable ros::ServiceClient sc_is_exists;
		mutable ros::ServiceClient sc_result;
		mutable ros::ServiceClient sc_times;
		mutable ros::ServiceClient sc_search;
		mutable ros::ServiceClient sc_actives;
		mutable ros::ServiceClient sc_end_states;
		mutable ros::ServiceClient ss_clear_history;

		class exception_connection_problem:public std::exception{};

	public:
		StatesMonitorClient(ros::NodeHandle& n/*, EventQueue& events*/);
		virtual ~StatesMonitorClient();


		//get output message with full status of monitor
		string status()const;

		//check if state is active
		bool is_active(const string& name)const;
		bool is_active(const string& name, string& full_name)const;

		//check if state was active sometime
		bool is_exists(const string& name)const;
		bool is_exists(const string& name, string& full_name)const;

		//get last result of state
		string result(const string& name)const;

		//get times (begin, end, freq, etc) of state
		struct TimesInfo{
			ptime begin;
			ptime end;
			time_duration duration;
			double frequency;
			string result;
			enum MASK{ BEGIN=0x1,END=0x2,DURATION=0x4,FREQUENCY=0x8,RESULT=0xa } mask;
		};
		TimesInfo times(const string& name)const;

		//search states
		set<string> search(const string& state)const;

		//get all active states
		set<string> actives(const string& state)const;

		void end_states(const string& state, const string& result);
		void clear_history(const string& state);

	};
}
}

#endif /* ROSSTATESMONITOR_H_ */
