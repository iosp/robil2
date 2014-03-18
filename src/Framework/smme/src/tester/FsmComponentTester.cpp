/*
 * FsmComponentTester.cpp
 *
 *  Created on: Mar 9, 2014
 *      Author: dan
 */

#include "FsmComponentTester.h"

#define SYNCH recursive_mutex::scoped_lock locker(mtx);

FsmComponentTester::FsmComponentTester():all(0) {
	sub_diagnostic = node.subscribe("/diagnostics", 10, &FsmComponentTester::on_diagnostic_msg, this);
	pub_events = node.advertise<std_msgs::String>("/decision_making/events", 10);
	node.setParam("/decision_making/debug",true);
}

FsmComponentTester::~FsmComponentTester() {
	//send("/stop_fsm_debug");
	node.setParam("/decision_making/debug",false);
}

void FsmComponentTester::start(){
	//send("/start_fsm_debug");
	//node.setParam("/decision_making/debug",true);
}

namespace{

//char blue[] = { 0x1b, '[', '1', ';', '3', '4', 'm', 0 };
//char normal[] = { 0x1b, '[', '0', ';', '3', '9', 'm', 0 };

#include "Colors.hpp"

std::string get(const diagnostic_msgs::DiagnosticStatus& status, std::string name){
	for(size_t i=0;i<status.values.size();i++)
		if(status.values[i].key == name) return status.values[i].value;
	return "";
}

std::vector<std::string> split(const string& s, char d, int start=1){
	stringstream ss;
	vector<std::string> res;
	for(size_t i=start;i<s.size();i++){
		if(s[i]==d){res.push_back(ss.str());ss.str("");continue;}
		ss<<s[i];
	}
	res.push_back(ss.str());
	return res;
}

}

#include <robil_msgs/FSMState.h>
void FsmComponentTester::init(){
	ros::ServiceClient srv = node.serviceClient<robil_msgs::FSMState>("/fsm_tracker/states");
	robil_msgs::FSMState st;
	st.request.fsm="/";
	if( srv.call(st) ){
		SYNCH
		cout<<"Current states:"<<endl;
		for(size_t i=0;i<st.response.states.size();i++){
			string state = st.response.states[i];
			cout<<"\t"<<state<<endl;
			diagnostic_msgs::DiagnosticStatus msg;
			typedef diagnostic_msgs::DiagnosticStatus::_values_type::value_type vtype;
#			define SET_VALUE(K,V) msg.values.push_back(vtype());msg.values.back().key=K;msg.values.back().value=V;
			SET_VALUE("status", "started");
			SET_VALUE("name", state);
#			undef SET_VALUE
			on_diagnostic(msg);
		}
		boost::this_thread::sleep(seconds(1));
	}else{

	}
}

void FsmComponentTester::on_diagnostic_msg(const diagnostic_msgs::DiagnosticArray::ConstPtr msg){
	BOOST_FOREACH(diagnostic_msgs::DiagnosticStatus st, msg->status){
		if( contains(st.name, "decision_making") )
			if( get(st, "type") == "FSM_STATE" )
				on_diagnostic(st);
	}
}

void FsmComponentTester::on_diagnostic(const diagnostic_msgs::DiagnosticStatus& msg){
	SYNCH

	string status = get(msg,"status");
	string name = get(msg,"name");
	string component = "/"+split(name,'/')[0];

	set<string> actives = active_items[component];

	if(status == "stopped"){
		actives.erase(name);
	}
	if(status == "started"){
		actives.insert(name);
	}

	if(actives!=active_items[component]){
		on_states_change(active_items[component], actives);
	}
	active_items[component] = actives;
}

ostream& operator<<(ostream& out, const set<string>& s){
	BOOST_FOREACH(string st, s){
		std::cout<<"   "<<st<<std::endl;
	}
	return out;
}
typedef set<string> SET;
SET operator-(const SET& s1, const SET& s2){
	SET s;
	BOOST_FOREACH(string n, s1)if(s2.find(n)==s2.end())s.insert(n);
	return s;
}
SET operator+(const SET& s1, const SET& s2){
	SET s;
	BOOST_FOREACH(string n, s1)s.insert(n);
	BOOST_FOREACH(string n, s2)s.insert(n);
	return s;
}
SET operator*(const SET& s1, const SET& s2){
	SET s;
	BOOST_FOREACH(string n, s1)if(s2.find(n)!=s2.end())s.insert(n);
	return s;
}
bool operator<(const string& n, const SET& s){ return s.find(n)!=s.end(); }
bool operator>(const SET& s,const string& n){ return s.find(n)!=s.end(); }
SET operator+(const SET& s1, const string& n){
	SET s = s1;
	s.insert(n);
	return s;
}
SET operator-(const SET& s1, const string& n){
	SET s=s1;
	s.erase(n);
	return s;
}

void FsmComponentTester::on_states_change(const set<string> old_state,const set<string> new_state){
	if(old_state<new_state){
		set<string> diff=new_state-old_state;
		//cout<<"STARTED:\n"<<diff<<endl;
	}else{
		set<string> diff=old_state-new_state;
		//cout<<"REMOVED:\n"<<diff<<endl;
	}
}


void sleep(){
	this_thread::sleep(seconds(1));
}


void FsmComponentTester::send(string e){
	std_msgs::String m; m.data = e;
	pub_events.publish(m);
	sleep();
}

void print(const SET& t, const SET& c){
	std::cout<<"Target:"<<endl<<t<<endl;
	std::cout<<"Actual:"<<endl<<c<<endl;
}
void print(const SET& c){
	std::cout<<"State:"<<endl<<c<<endl;
}

void FsmComponentTester::print_actives(){
	SYNCH
	typedef map< string,set<string> > T;
	for(T::const_iterator a=active_items.begin();a!=active_items.end();a++){
		cout<<"# "<<a->first<<endl;
		cout<<a->second<<endl;
	}
}

#define SEND(E) if(ros::ok())send(E);else return false;
#define TEST(S) {SYNCH \
	if(active_items[target_component]!=S){ \
		std::cout<<"TEST " #S <<BOLDRED<< " FAULT" <<RESET<< std::endl; print(S,active_items[target_component]);  return false; }\
		else{ std::cout<<"   test " #S <<std::endl; \
	}}
#define WAIT  std::cout<<"WAIT FOR COMPONENT "<<target_component<<std::endl; \
	int s;{SYNCH s=active_items[target_component].size();}\
	while(ros::ok() and active_items[target_component].size()==0){sleep();{SYNCH s=active_items[target_component].size();}}\
	std::cout<<"START TEST"<<std::endl;\
	start();\
	sleep();

#define PASS std::cout<<"TEST "<<BOLDGREEN<<"PASS"<<RESET<<std::endl; return true;

bool FsmComponentTester::test1(bool wait){
	cout<<"[test type 1]"<<endl;
	SET state_on;
		state_on.insert(target_component+"/ON");
		state_on.insert(target_component+"/ON"+target_component+"_ON/INIT");
	SET state_off;
		state_off.insert(target_component+"/OFF");
	SET state_work;
		state_work.insert(target_component+"/ON");
		state_work.insert(target_component+"/ON"+target_component+"_ON/WORK");
		state_work.insert(target_component+"/ON"+target_component+"_ON/WORK"+target_component+"_WORK/STANDBY");
	SET state_work_ready;
		state_work_ready.insert(target_component+"/ON");
		state_work_ready.insert(target_component+"/ON"+target_component+"_ON/WORK");
		state_work_ready.insert(target_component+"/ON"+target_component+"_ON/WORK"+target_component+"_WORK/READY");

	if(wait){
		WAIT
	}

	TEST(state_on);

	SEND(target_component+"/Shutdown");
	TEST(state_off);

	SEND(target_component+"/Activation");
	TEST(state_on);

	SEND(target_component+"/ON"+target_component+"_ON/INIT/INIT/EndOfInit")
	TEST(state_work);

	SEND(target_component+"/Resume")
	TEST(state_work_ready);

	SEND(target_component+"/Standby")
	TEST(state_work);

	SEND(target_component+"/Shutdown");
	TEST(state_off);

	SEND(target_component+"/Activation");
	SEND(target_component+"/ON"+target_component+"_ON/INIT/INIT/EndOfInit")
	SEND(target_component+"/Resume")
	SEND(target_component+"/Shutdown");
	TEST(state_off);

	SEND(target_component+"/Activation");
	PASS
}

bool FsmComponentTester::test2(bool wait){
	cout<<"[test type 2]"<<endl;
	SET state_on;
		state_on.insert(target_component+"/ON");
		state_on.insert(target_component+"/ON"+target_component+"_ON/INIT");
	SET state_off;
		state_off.insert(target_component+"/OFF");
	SET state_work;
		state_work.insert(target_component+"/ON");
		state_work.insert(target_component+"/ON"+target_component+"_ON/WORK");
		state_work.insert(target_component+"/ON"+target_component+"_ON/WORK"+target_component+"_WORK/STANDBY");
	SET state_work_ready;
		state_work_ready.insert(target_component+"/ON");
		state_work_ready.insert(target_component+"/ON"+target_component+"_ON/WORK");
		state_work_ready.insert(target_component+"/ON"+target_component+"_ON/WORK"+target_component+"_WORK/READY");

	if(wait){
		WAIT
	}

	TEST(state_on);

	SEND(target_component+"/Shutdown");
	TEST(state_off);

	SEND(target_component+"/Activation");
	TEST(state_on);

	SEND(target_component+"/SensorConnected")
	TEST(state_work);

	SEND(target_component+"/SensorNotConnected")
	TEST(state_on);

	SEND(target_component+"/SensorConnected")
	TEST(state_work);

	SEND(target_component+"/Resume")
	TEST(state_work_ready);

	SEND(target_component+"/Standby")
	TEST(state_work);

	SEND(target_component+"/Shutdown");
	TEST(state_off);

	SEND(target_component+"/Activation");
	SEND(target_component+"/SensorConnected")
	SEND(target_component+"/Resume")
	SEND(target_component+"/Shutdown");
	TEST(state_off);

	SEND(target_component+"/Activation");
	PASS
}

bool FsmComponentTester::test3(bool wait){
	cout<<"[test type 3]"<<endl;
	SET state_on;
		state_on.insert(target_component+"/ON");
		state_on.insert(target_component+"/ON"+target_component+"_ON/INIT");
	SET state_off;
		state_off.insert(target_component+"/OFF");
	SET state_work;
		state_work.insert(target_component+"/ON");
		state_work.insert(target_component+"/ON"+target_component+"_ON/READY");

	if(wait){
		WAIT
	}

	TEST(state_on);

	SEND(target_component+"/Shutdown");
	TEST(state_off);

	SEND(target_component+"/Activation");
	TEST(state_on);

	SEND(target_component+"/ON"+target_component+"_ON/INIT/INIT/EndOfInit")
	TEST(state_work);

	SEND(target_component+"/Shutdown");
	TEST(state_off);

	SEND(target_component+"/Activation");
	PASS
}

bool FsmComponentTester::test(bool wait){
	if(target_component=="/wsm"){
		return test2(wait);
	}else
	if(
		target_component=="/llc" or
		target_component=="/per" or
		target_component=="/ssm" or
		target_component=="/iedsim" or
	false){
		return test3(wait);
	}else{
		return test1(wait);
	}
	return false;
}

map<string,string> comp_rename;

void init_comp_rename(){
// 	comp_rename["llc"]="/LLC";
// 	comp_rename["per"]="/Perception";
// 	comp_rename["ssm"]="/Monitoring";
// 	comp_rename["iedsim"]="/IED";
// 	comp_rename["ied"]="/IED";
// 	comp_rename["wsm"]="/WorkSequnceManager";
// 	comp_rename["pp"]="/PathPlanner";
// 	comp_rename["wpd"]="/WaypointDriver";
}

int main(int a, char** aa){

	ros::init(a, aa, "FsmComponentTester");
	init_comp_rename();
	ros::AsyncSpinner spinner(2);
	FsmComponentTester tester;
	spinner.start();

	tester.init();

	int errors = 0;
	if(a==2 and string(aa[1])=="all"){
		tester.all = true;
		tester.target_component="/";
		spinner.start();
		tester.start();
		errors+=tester.test(true)?0:1;
		return 0;
	}
	vector<string> components;
	if(a==1){
		string params; ros::param::param("/testing/components",params,string());
		cout<<"Components: "<<params<<endl;
		components = split(params,' ',0);
	}else{
		for(int i=1;i<a;i++) components.push_back(string(aa[i]));
	}
	for(size_t i=0;i<components.size();i++){
		tester.target_component = components[i];
		cout<<BOLDBLACK<<"---- "<<tester.target_component<<" ----"<<RESET<<endl;
		if(comp_rename.find(tester.target_component)!=comp_rename.end())tester.target_component=comp_rename[tester.target_component];
		if(tester.target_component[0]!='/') tester.target_component="/"+tester.target_component;
		errors+=tester.test(true)?0:1;
	}
	std::cout<<BOLDBLACK<<"SUMMARY: "<<(errors?RED:GREEN)<< errors <<" error(s)"<<RESET<< std::endl;
}
