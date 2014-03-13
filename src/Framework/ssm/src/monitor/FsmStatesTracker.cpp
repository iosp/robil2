/*
 * FsmComponentTester.cpp
 *
 *  Created on: Mar 9, 2014
 *      Author: dan
 */

#include "FsmStatesTracker.h"

#define SYNCH recursive_mutex::scoped_lock locker(mtx);

FsmStatesTracker::FsmStatesTracker() {
	sub_diagnostic = node.subscribe("/diagnostics", 10, &FsmStatesTracker::on_diagnostic_msg, this);
	ss_actives = node.advertiseService("/fsm_tracker/states",&FsmStatesTracker::on_ss_actives, this);
	ss_remove = node.advertiseService("/fsm_tracker/remove",&FsmStatesTracker::on_ss_remove, this);
	ss_add = node.advertiseService("/fsm_tracker/add",&FsmStatesTracker::on_ss_add, this);
}

FsmStatesTracker::~FsmStatesTracker() {
}

namespace{

//char blue[] = { 0x1b, '[', '1', ';', '3', '4', 'm', 0 };
//char normal[] = { 0x1b, '[', '0', ';', '3', '9', 'm', 0 };

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

std::string get(const diagnostic_msgs::DiagnosticStatus& status, std::string name){
	for(size_t i=0;i<status.values.size();i++)
		if(status.values[i].key == name) return status.values[i].value;
	return "";
}

std::vector<std::string> split(const string& s, char d){
	stringstream ss;
	vector<std::string> res;
	for(size_t i=1;i<s.size();i++){
		if(s[i]==d){res.push_back(ss.str());ss.str("");}
		ss<<s[i];
	}
	res.push_back(ss.str());
	return res;
}

}


void FsmStatesTracker::on_diagnostic_msg(const diagnostic_msgs::DiagnosticArray::ConstPtr msg){
	BOOST_FOREACH(diagnostic_msgs::DiagnosticStatus st, msg->status){
		if( contains(st.name, "decision_making") )
			if( get(st, "type") == "FSM_STATE" )
				on_diagnostic(st);
	}
}

void FsmStatesTracker::on_diagnostic(const diagnostic_msgs::DiagnosticStatus& msg){
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

bool FsmStatesTracker::on_ss_add(robil_msgs::FSMState::Request& req, robil_msgs::FSMState::Response& res){
	SYNCH
	string name = req.fsm;
	string component = "/"+split(name,'/')[0];
	set<string> actives = active_items[component];
	actives.insert(name);
	active_items[component] = actives;
	return true;
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

void FsmStatesTracker::on_states_change(const set<string> old_state,const set<string> new_state){
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

void print(const SET& t, const SET& c){
	std::cout<<"Target:"<<endl<<t<<endl;
	std::cout<<"Actual:"<<endl<<c<<endl;
}
void print(const SET& c){
	std::cout<<"State:"<<endl<<c<<endl;
}

void FsmStatesTracker::print_actives(){
	SYNCH
	typedef map< string,set<string> > T;
	for(T::const_iterator a=active_items.begin();a!=active_items.end();a++){
		cout<<"# "<<a->first<<endl;
		cout<<a->second<<endl;
	}
}

bool FsmStatesTracker::on_ss_actives(robil_msgs::FSMState::Request& req, robil_msgs::FSMState::Response& res){
	SYNCH
	typedef map< string,set<string> > T;
	typedef set<string> K;
	for(T::const_iterator a=active_items.begin();a!=active_items.end();a++){
		for(K::const_iterator b=a->second.begin();b!=a->second.end();b++){
			if( starts_with(*b, req.fsm) )
				res.states.push_back(*b);
		}
	}
	return true;
}
bool FsmStatesTracker::on_ss_remove(robil_msgs::FSMState::Request& req, robil_msgs::FSMState::Response& res){
	SYNCH
	typedef map< string,set<string> > T;
	typedef set<string> K;
	for(T::iterator a=active_items.begin();a!=active_items.end();a++){
		set<string> for_remove;
		for(K::iterator b=a->second.begin();b!=a->second.end();b++){
			if( starts_with(*b, req.fsm) ){
				for_remove.insert(*b);
				res.states.push_back(*b);
			}
		}
		set<string> res = a->second - for_remove;
		a->second.clear();
		a->second.insert(res.begin(), res.end());
	}
	return true;
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

	ros::init(a, aa, "FsmStatesTracker");
	init_comp_rename();
	FsmStatesTracker tracker;
	ros::spin();
}
