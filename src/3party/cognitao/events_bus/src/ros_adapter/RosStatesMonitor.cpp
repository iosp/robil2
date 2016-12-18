/*
 * RosStatesMonitor.cpp
 *
 *  Created on: Nov 9, 2014
 *      Author: dan
 */

#include "../../include/cognitao/bus/RosStatesMonitor.h"

using namespace cognitao;
using namespace cognitao::bus;


#ifndef ROS_SERVIS_CALL_PATCH1
#define ROS_SERVIS_CALL_PATCH1

template<class MESSAGE>
bool service_call(ros::ServiceClient& ros_service, MESSAGE& msg)
{
	bool result = false;
	{
		typedef boost::shared_ptr<boost::this_thread::disable_interruption> interrupt_switch_t;
		interrupt_switch_t interrupt_switcher( new boost::this_thread::disable_interruption );

		result = ros_service.call(msg);
	}

	boost::this_thread::interruption_point();
	return result;
}

template<class MReq, class MRes>
bool service_call(ros::ServiceClient& ros_service, MReq& req, MRes& res)
{
	bool result = false;
	{
		typedef boost::shared_ptr<boost::this_thread::disable_interruption> interrupt_switch_t;
		interrupt_switch_t interrupt_switcher( new boost::this_thread::disable_interruption );

		result = ros_service.call(req, res);
	}

	boost::this_thread::interruption_point();
	return result;
}

#endif


namespace {
	const string service_name_status="status";
	const string service_name_raw_status="raw_status";
	const string service_name_is_active="is_active";
	const string service_name_is_exists="is_exists";
	const string service_name_result="result";
	const string service_name_times="times";
	const string service_name_actives="actives";
	const string service_name_search="search";
	const string service_name_end_states="end_states";
	const string service_name_clear_history="clear_history";
}

namespace node{

RosStatesMonitor::RosStatesMonitor(ros::NodeHandle& n, EventQueue& events):
		node(n), monitor(events), events(events)
{
	ros::NodeHandle node("states_monitor");
	ss_status = node.advertiseService(service_name_status, &node::RosStatesMonitor::on_ask_status,this);
	ss_raw_status = node.advertiseService(service_name_raw_status, &node::RosStatesMonitor::on_ask_raw_status,this);
	ss_is_active = node.advertiseService(service_name_is_active, &node::RosStatesMonitor::on_ask_is_active,this);
	ss_is_exists = node.advertiseService(service_name_is_exists, &node::RosStatesMonitor::on_ask_is_exists,this);
	ss_result = node.advertiseService(service_name_result, &node::RosStatesMonitor::on_ask_result,this);
	ss_times = node.advertiseService(service_name_times, &node::RosStatesMonitor::on_ask_times,this);
	ss_actives = node.advertiseService(service_name_actives, &node::RosStatesMonitor::on_ask_actives,this);
	ss_search = node.advertiseService(service_name_search, &node::RosStatesMonitor::on_ask_search,this);
	ss_end_states = node.advertiseService(service_name_end_states, &node::RosStatesMonitor::on_ask_end_states,this);
	ss_clear_history = node.advertiseService(service_name_clear_history, &node::RosStatesMonitor::on_ask_clear_history,this);
}

RosStatesMonitor::~RosStatesMonitor() {
}

void RosStatesMonitor::process(){
	monitor.process();
}

//get output message with full status of monitor
string RosStatesMonitor::status()const {
	stringstream s; s<<monitor;
	return s.str();
}
events_bus::Srv_StatesMonitor_RawStatus::Response RosStatesMonitor::raw_status()const {
	events_bus::Srv_StatesMonitor_RawStatus::Response res;

	mutex::scoped_lock l(monitor.m);

	using namespace boost::gregorian;
    using namespace boost::posix_time;
	for(StatesMonitor::States::const_iterator i=monitor.states.begin();i!=monitor.states.end();i++){
		events_bus::Srv_StatesMonitor_RawStatus::Response::_states_type::value_type state;
		state.name = i->str();
		state.begin = ros::Time::fromBoost(monitor.begin_time.at(*i));

		res.states.push_back(state);
	}
	for(StatesMonitor::Results::const_iterator i=monitor.results.begin();i!=monitor.results.end();i++){
		events_bus::Srv_StatesMonitor_RawStatus::Response::_results_type::value_type result;
		result.name = i->first.str();
		TimesInfo info;
		info.mask = (TimesInfo::MASK) monitor._get_statistic(result.name, info.begin, info.end, info.duration, info.frequency, info.result);
		if(info.mask & TimesInfo::BEGIN) 		result.begin 				= ros::Time::fromBoost(info.begin);
		if(info.mask & TimesInfo::END) 			result.end 					= ros::Time::fromBoost(info.end);
		if(info.mask & TimesInfo::DURATION) 	result.duration_microsec 	= info.duration.total_microseconds();
		if(info.mask & TimesInfo::FREQUENCY) 	result.frequency 			= info.frequency;
		if(info.mask & TimesInfo::RESULT) 		result.result 				= info.result;

		res.results.push_back(result);
	}
	return res;
}

//check if state is active
bool RosStatesMonitor::is_active(const string& name)const {
	return monitor.is_active(name);
}
bool RosStatesMonitor::is_active(const string& name, string& full_name)const {
	return monitor.is_active(name, full_name);
}

//check if state was active sometime
bool RosStatesMonitor::is_exists(const string& name)const {
	return monitor.is_exists(name);
}
bool RosStatesMonitor::is_exists(const string& name, string& full_name)const {
	return monitor.is_exists(name, full_name);
}

//get last result of state
string RosStatesMonitor::result(const string& name)const {
	return monitor.get_result(name);
}

//get times (begin, end, freq, etc) of state
RosStatesMonitor::TimesInfo RosStatesMonitor::times(const string& name)const {
	TimesInfo times;
	times.mask = (TimesInfo::MASK) monitor.get_statistic(name, times.begin, times.end, times.duration, times.frequency, times.result);
	return times;
}

//search states
set<string> RosStatesMonitor::search(const string& state)const {
	vector<string> st = monitor.get_all(state);
	set<string> res(st.begin(), st.end());
	return res;
}

//get all active states
set<string> RosStatesMonitor::actives(const string& state)const {
	vector<string> ac = monitor.get_actives(state);
	set<string> res(ac.begin(), ac.end());
	return res;
}

void RosStatesMonitor::end_states(const string& state, const string& res){
	monitor.end_all_states(state, res);
}

void RosStatesMonitor::clear_history(const string& state){
	monitor.clear_history(state);
}

bool RosStatesMonitor::on_ask_status(events_bus::Srv_StatesMonitor_Status::Request& req, events_bus::Srv_StatesMonitor_Status::Response& res){
	res.text = status();
	return true;
}
bool RosStatesMonitor::on_ask_raw_status(events_bus::Srv_StatesMonitor_RawStatus::Request& req, events_bus::Srv_StatesMonitor_RawStatus::Response& res){
	res = raw_status();
	return true;
}
bool RosStatesMonitor::on_ask_is_active(events_bus::Srv_StatesMonitor_Is::Request& req, events_bus::Srv_StatesMonitor_Is::Response& res){
	res.result = is_active(req.name, res.full_name);
	return true;
}
bool RosStatesMonitor::on_ask_is_exists(events_bus::Srv_StatesMonitor_Is::Request& req, events_bus::Srv_StatesMonitor_Is::Response& res){
	res.result = is_exists(req.name, res.full_name);
	return true;
}
bool RosStatesMonitor::on_ask_result(events_bus::Srv_StatesMonitor_Result::Request& req, events_bus::Srv_StatesMonitor_Result::Response& res){
	res.result = result(req.name);
	return true;
}
bool RosStatesMonitor::on_ask_times(events_bus::Srv_StatesMonitor_Times::Request& req, events_bus::Srv_StatesMonitor_Times::Response& res){
	TimesInfo info = times(req.name);
	res.mask = info.mask;
	if(info.mask & TimesInfo::BEGIN) 		res.begin 				= ros::Time::fromBoost(info.begin);
	if(info.mask & TimesInfo::END) 			res.end 				= ros::Time::fromBoost(info.end);
	if(info.mask & TimesInfo::DURATION) 	res.duration_microsec 	= info.duration.total_microseconds();
	if(info.mask & TimesInfo::FREQUENCY) 	res.frequency 			= info.frequency;
	if(info.mask & TimesInfo::RESULT) 		res.result 				= info.result;
	return true;
}
bool RosStatesMonitor::on_ask_search(events_bus::Srv_StatesMonitor_States::Request& req, events_bus::Srv_StatesMonitor_States::Response& res){
	if(req.name == "") req.name="*";
	set<string> s = search(req.name);
	res.states = vector<string>(s.begin(), s.end());
	return true;
}
bool RosStatesMonitor::on_ask_actives(events_bus::Srv_StatesMonitor_States::Request& req, events_bus::Srv_StatesMonitor_States::Response& res){
	if(req.name == "") req.name="*";
	set<string> s = actives(req.name);
	res.states = vector<string>(s.begin(), s.end());
	return true;
}

bool RosStatesMonitor::on_ask_end_states(events_bus::Srv_StatesMonitor_EndStates::Request& req, events_bus::Srv_StatesMonitor_EndStates::Response& res){
	end_states(req.name, req.result);
	return true;
}

bool RosStatesMonitor::on_ask_clear_history(events_bus::Srv_StatesMonitor_States::Request& req, events_bus::Srv_StatesMonitor_States::Response& res){
	clear_history(req.name);
}

}


namespace cognitao{
namespace monitor{

StatesMonitorClient::StatesMonitorClient(ros::NodeHandle& n/*, EventQueue& events*/):
		node(n)/*, events(events)*/
{
	ros::NodeHandle node("states_monitor");
	sc_status = node.serviceClient<events_bus::Srv_StatesMonitor_Status>(service_name_status);
	sc_is_active = node.serviceClient<events_bus::Srv_StatesMonitor_Is>(service_name_is_active);
	sc_is_exists = node.serviceClient<events_bus::Srv_StatesMonitor_Is>(service_name_is_exists);
	sc_result = node.serviceClient<events_bus::Srv_StatesMonitor_Result>(service_name_result);
	sc_times = node.serviceClient<events_bus::Srv_StatesMonitor_Times>(service_name_times);
	sc_actives = node.serviceClient<events_bus::Srv_StatesMonitor_States>(service_name_actives);
	sc_search = node.serviceClient<events_bus::Srv_StatesMonitor_States>(service_name_search);
}

StatesMonitorClient::~StatesMonitorClient() {
}


//get output message with full status of monitor
string StatesMonitorClient::status()const {
	events_bus::Srv_StatesMonitor_Status srv;
	if( service_call(sc_status, srv) ){
		return srv.response.text;
	}else{
		throw exception_connection_problem();
	}
}

//check if state is active
bool StatesMonitorClient::is_active(const string& name)const {
	events_bus::Srv_StatesMonitor_Is srv;
	srv.request.name = name;
	if( service_call(sc_is_active, srv) ){
		return srv.response.result;
	}else{
		throw exception_connection_problem();
	}
}
bool StatesMonitorClient::is_active(const string& name, string& full_name)const {
	events_bus::Srv_StatesMonitor_Is srv;
	srv.request.name = name;
	if( service_call(sc_is_active, srv) ){
		full_name = srv.response.full_name;
		return srv.response.result;
	}else{
		throw exception_connection_problem();
	}
}

//check if state was active sometime
bool StatesMonitorClient::is_exists(const string& name)const {
	events_bus::Srv_StatesMonitor_Is srv;
	srv.request.name = name;
	if( service_call(sc_is_exists, srv) ){
		return srv.response.result;
	}else{
		throw exception_connection_problem();
	}
}
bool StatesMonitorClient::is_exists(const string& name, string& full_name)const {
	events_bus::Srv_StatesMonitor_Is srv;
	srv.request.name = name;
	if( service_call(sc_is_exists, srv) ){
		full_name = srv.response.full_name;
		return srv.response.result;
	}else{
		throw exception_connection_problem();
	}
}

//get last result of state
string StatesMonitorClient::result(const string& name)const {
	events_bus::Srv_StatesMonitor_Result srv;
	srv.request.name = name;
	if( service_call(sc_result, srv) ){
		return srv.response.result;
	}else{
		throw exception_connection_problem();
	}
}

//get times (begin, end, freq, etc) of state
StatesMonitorClient::TimesInfo StatesMonitorClient::times(const string& name)const {
	events_bus::Srv_StatesMonitor_Times srv;
	srv.request.name = name;
	if( service_call(sc_result, srv) ){
		TimesInfo times;
		times.mask = (TimesInfo::MASK) srv.response.mask;
		if(times.mask & TimesInfo::BEGIN)		times.begin 	= srv.response.begin.toBoost();
		if(times.mask & TimesInfo::END)			times.end 		= srv.response.begin.toBoost();
		if(times.mask & TimesInfo::DURATION)	times.duration 	= microseconds(srv.response.duration_microsec);
		if(times.mask & TimesInfo::FREQUENCY)	times.frequency = srv.response.frequency;
		if(times.mask & TimesInfo::RESULT)		times.result 	= srv.response.result;
		return times;
	}else{
		throw exception_connection_problem();
	}
}

//search states
set<string> StatesMonitorClient::search(const string& state)const {
	events_bus::Srv_StatesMonitor_States srv;
	srv.request.name = state;
	if( service_call(sc_actives, srv) ){
		return set<string>(srv.response.states.begin(), srv.response.states.end());
	}else{
		throw exception_connection_problem();
	}
}

//get all active states
set<string> StatesMonitorClient::actives(const string& state)const {
	events_bus::Srv_StatesMonitor_States srv;
	srv.request.name = state;
	if( service_call(sc_search, srv) ){
		return set<string>(srv.response.states.begin(), srv.response.states.end());
	}else{
		throw exception_connection_problem();
	}
}


void StatesMonitorClient::end_states(const string& state, const string& result){
	events_bus::Srv_StatesMonitor_EndStates srv;
	srv.request.name = state;
	srv.request.result = result;
	if( service_call(sc_search, srv) ){
		return;
	}else{
		throw exception_connection_problem();
	}
}
void StatesMonitorClient::clear_history(const string& state){
	events_bus::Srv_StatesMonitor_States srv;
	srv.request.name = state;
	if( service_call(sc_search, srv) ){
		return;
	}else{
		throw exception_connection_problem();
	}
}

}
}


