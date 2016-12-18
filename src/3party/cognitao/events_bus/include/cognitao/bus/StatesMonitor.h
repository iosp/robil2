/*
 * StatesMonitor.h
 *
 *  Created on: Nov 5, 2014
 *      Author: dan
 */

#ifndef STATESMONITOR_H_
#define STATESMONITOR_H_

#include "HierarchicalName.h"
#include "EventQueue.h"

namespace cognitao{
namespace monitor{

using namespace bus;

class Model{
public:
	virtual ~Model(){}
	virtual void on_event(const Event& event, EventRiser& riser)=0;
	virtual void init(EventRiser& riser)=0;
};

class StatesMonitor{
public:
	mutable mutex m;
	EventQueue events;
	typedef set<Model*> Models;
	Models models;
	typedef set<HierarchicalName> States;
	States states;
	typedef map<HierarchicalName, string> Results;
	Results results;
	typedef map<HierarchicalName, posix_time::ptime> Times;
	typedef map<HierarchicalName, posix_time::time_duration> Durations;
	Times begin_time, end_time;
	Durations duration, frequency;

//	static string _begin_event_text(){ return "/state/begin"; }
//	static string _end_event_text(){ return "/state/end"; }
//	static string _endAll_event_text(){ return "/state/endALL"; }
//	static string _clean_event_text(){ return "/state/clean"; }

	static string _states_channel(){ return "states"; }
	static Event::name_t _state_name_begin(){ return Event::name_t("/begin"); }
	static Event::name_t _state_name_end(string result){ return Event::name_t(string("/end/")+result); }
	static Event::name_t _state_name_endAll(string result){ return Event::name_t(string("/end_all/")+result); }
	static Event::name_t _state_name_clean(){ return Event::name_t("/clean"); }

	static Event begin_event(const Event::context_t& context){
		return Event(_state_name_begin(), _states_channel(), context);
	}
	static Event end_event(const Event::context_t& context, string result="*"){
		if(result == "") result="N-S";
		return Event(_state_name_end(result), _states_channel(), context);
	}
	static Event endAll_event(const Event::context_t& context, string result="*"){
		if(result == "") result="N-S";
		return Event(_state_name_endAll(result), _states_channel(), context);
	}
	static Event clean_event(const Event::context_t& context){
		return Event(_state_name_clean(), _states_channel(), context);
	}
	static Event begin_event(const string& context){ return begin_event(Event::context_t(context));	}
	static Event end_event(const string& context, string result){ return end_event(Event::context_t(context), result); }
	static Event endAll_event(const string& context, string result){ return endAll_event(Event::context_t(context), result); }
	static Event clean_event(const string& context){ return clean_event(Event::context_t(context)); }

	static
	bool is_state( const Event& event )
	{
		return event.channel() == _states_channel();
	}
	static
	bool is_state_begin( const Event& event )
	{
		return event.name() == _state_name_begin();
	}
	static
	bool is_state_end( const Event& event, string result = "*" )
	{
		return event.name() == _state_name_end(result);
	}
	static
	bool is_state_endAll( const Event& event, string result = "*" )
	{
		return event.name() == _state_name_endAll(result);
	}
	static
	bool is_state_clean( const Event& event )
	{
		return event.name() == _state_name_clean();
	}
	static
	string extract_result( const Event& event )
	{
		vector<string> s = event.name().split();
		if(s.size()<2) return "";
		return HierarchicalName(s.begin()+1, s.end()).text;
	}


	StatesMonitor(EventQueue& ev):events(ev){}
	void process_models(const Event& e);
	void turn_on(const HierarchicalName& state, const ptime& time);
	void turn_off(const HierarchicalName& state, const string& result, const ptime& time);
	void turn_off_all(const HierarchicalName& state, const string& result, const ptime& time);
	void clean_all(const HierarchicalName& state, const ptime& time);
	void on_event(const Event& event);
	void process();

	void add(Model* model);
	void add(Model& model);
	void remove(Model* model);
	void remove(Model& model);

	friend ostream& operator<<(ostream& out, const StatesMonitor& m);

	template<class T>
	typename T::iterator _find(const std::string& st, T& states)const{
		HierarchicalName est(st);
		if(est.is_template()){
			for(typename T::iterator i=states.begin();i!=states.end();i++){
				if( i->equals(est) ) return i;
			}
			return states.end();
		}else{
			return states.find(est);
		}
	}
	template<class T>
	typename T::const_iterator _const_find(const std::string& st, const T& states)const{
		HierarchicalName est(st);
		if(est.is_template()){
			for(typename T::const_iterator i=states.begin();i!=states.end();i++){
				if( i->equals(est) ) return i;
			}
			return states.end();
		}else{
			return states.find(est);
		}
	}
	template<class T>
	typename T::iterator _find_map(const std::string& st, T& states)const{
		HierarchicalName est(st);
		if(est.is_template()){
			for(typename T::iterator i=states.begin();i!=states.end();i++){
				if( i->firt.equals(est) ) return i;
			}
			return states.end();
		}else{
			return states.find(est);
		}
	}
	template<class T>
	typename T::const_iterator _const_find_map(const std::string& st, const T& states)const{
		HierarchicalName est(st);
		if(est.is_template()){
			for(typename T::const_iterator i=states.begin();i!=states.end();i++){
				if( i->first.equals(est) ) return i;
			}
			return states.end();
		}else{
			return states.find(est);
		}
	}

	bool is_active(const std::string& st)const;
	bool is_active(const std::string& st, std::string& full_name)const;
	bool is_exists(const std::string& st)const;
	bool is_exists(const std::string& st, std::string& full_name)const;
	posix_time::ptime get_begin_time(const std::string& st)const;
	posix_time::ptime get_end_time(const std::string& st)const;
	posix_time::time_duration get_duration(const std::string& st)const;
	double _get_frequency(const std::string& st)const;
	double get_frequency(const std::string& st)const;
	string get_result(const std::string& st)const;
	int get_statistic(const string& st, ptime& b, ptime& e, time_duration& d, double& f, string& result)const;
	int _get_statistic(const string& st, ptime& b, ptime& e, time_duration& d, double& f, string& result)const;
	vector<string> _get_actives()const;
	vector<string> _get_actives(string name)const;
	vector<string> _get_all()const;
	vector<string> _get_all(string name)const;
	vector<string> get_actives()const;
	vector<string> get_actives(string name)const;
	vector<string> get_all()const;
	vector<string> get_all(string name)const;
	void end_all_states(string name, string result);
	void clear_history(string name);
};

ostream& operator<<(ostream& out, const StatesMonitor& m);

namespace debugging{
struct VerbalBool
{
public:
	bool value;
	VerbalBool( bool v ):value(v){ std::cout<<"[d] 1 bool("<<&value<<") = "<<(value?"true":"false")<<std::endl; }
	operator bool()const{ return value; }
	bool& operator=( bool v ){ value=v; std::cout<<"[d] 2 bool("<<&value<<") = "<<(value?"true":"false")<<std::endl; return value; }
	bool& operator=( const VerbalBool& v ){ value=v.value; std::cout<<"[d] 3 bool("<<&value<<") = "<<(value?"true":"false")<<std::endl; return value; }
	VerbalBool( const VerbalBool& v ):value(v.value){ std::cout<<"[d] 4 bool("<<&value<<") = "<<(value?"true":"false")<<std::endl; }
};
}

typedef debugging::VerbalBool BOOL;

#define STATE_DEF_VERB false
class State{
	mutable mutex x;
	EventRiser& events;
public:
	bool verb;
	bool open;
	HierarchicalName state;
	State(const State& other):events(other.events),verb(other.verb),open(other.open),state(other.state){  }
	State(EventRiser& events, const HierarchicalName& init_state):verb(STATE_DEF_VERB),open(false),events(events),state(init_state){ _set_state(state); }
	State(EventRiser& events, const string& init_state):verb(STATE_DEF_VERB),open(false),events(events),state(init_state){ _set_state(state); }
	State(EventRiser::Ptr events, const HierarchicalName& init_state):verb(STATE_DEF_VERB),open(false),events(*events),state(init_state){ _set_state(state); }
	State(EventRiser::Ptr events, const string& init_state):verb(STATE_DEF_VERB),open(false),events(*events),state(init_state){ _set_state(state); }
	State(EventRiser& events):verb(STATE_DEF_VERB),open(false),events(events),state(""){}
	State(EventRiser::Ptr events):verb(STATE_DEF_VERB),open(false),events(*events),state(""){}
protected:
	void set_state(const HierarchicalName& state){mutex::scoped_lock l(x); _set_state(state);}
	void _set_state(const HierarchicalName& state){
		if(open){if(verb)cout<<state.str()<<" is already open, so ignore open"<<endl; return;}
		events << StatesMonitor::begin_event(state);
		this->state = state;
		if(verb)cout<<"open state: "<<state.str()<<endl;
		open=true;
	}
	void close_state(const string& result="success"){mutex::scoped_lock l(x); _close_state(result);}
	void _close_state(const string& result="success"){
		if(not open){ if(verb) cout<<state.str()<<" is not open, so ignore close"<<endl; return; }
		events << StatesMonitor::end_event(state,result);
		if(verb)cout<<"close state: "<<state.str()<<" with "<<result<<endl;
		open = false;
	}
	void change_state(const HierarchicalName& state, const string& result="success"){mutex::scoped_lock l(x);
		if(this->state==state) return;
		_close_state(result);_set_state(state);
	}
public:
	operator HierarchicalName()const{ return state; }
	const State& init(const HierarchicalName& s){set_state(s); return *this;}
	const State& error(const string& result, const HierarchicalName& s){ change_state(s, result); return *this;}
	const State& error(const string& result){ close_state(result);  return *this;}
	const State& success(){ close_state();  return *this;}
	const State& success(const HierarchicalName& s){ change_state(s); return *this;}
	const State& operator=(const HierarchicalName& s){ return success(s);}
	const State& operator=(const string& s){ return success(HierarchicalName(s));}
	~State(){
		close_state();
	}
};
inline bool operator==(const State& s, const HierarchicalName& n){ return s.state == n; }
inline bool operator==(const HierarchicalName& n, const State& s){ return s.state == n; }
inline bool operator!=(const State& s, const HierarchicalName& n){ return s.state != n; }
inline bool operator!=(const HierarchicalName& n, const State& s){ return s.state != n; }


class StatesMap
{
public:
	typedef boost::shared_ptr<State> StatePtr;
	typedef std::map<std::string, StatePtr> Map;
	Map states;

	typedef Map::iterator iterator;
	typedef Map::const_iterator const_iterator;

	iterator begin(){ return states.begin(); }
	const_iterator begin()const{ return states.begin(); }
	iterator end(){ return states.end(); }
	const_iterator end()const{ return states.end(); }
	iterator find(const std::string& key){ return states.find(key); }
	const_iterator find(const std::string& key)const{ return states.find(key); }

	bool contains(const std::string& key)const{ return find(key)!=end(); }

	void erase(const std::string& key){ states.erase(key);	}
	void erase(iterator i){ states.erase(i); }

	const State& operator[](const std::string& key)const{ return *(states.at(key)); }
	State& operator[](const std::string& key){ return *(states.at(key)); }
	const State& at(const std::string& key)const{ return *(states.at(key)); }
	State& at(const std::string& key){ return *(states.at(key)); }

	StatePtr ptr(const std::string& key){ if(contains(key)) return states.at(key); return StatePtr(); }

	void addState(const std::string& key, const State& other)
	{
		states[key] = StatePtr( new State(other) );
	}
	void addState(const std::string& key, EventRiser& events, const HierarchicalName& init_state)
	{
		states[key] = StatePtr( new State(events, init_state) );
	}
	void addState(const std::string& key, EventRiser& events, const string& init_state)
	{
		states[key] = StatePtr( new State(events, init_state) );
	}
	void addState(const std::string& key, EventRiser::Ptr events, const HierarchicalName& init_state)
	{
		states[key] = StatePtr( new State(events, init_state) );
	}
	void addState(const std::string& key, EventRiser::Ptr events, const string& init_state)
	{
		states[key] = StatePtr( new State(events, init_state) );
	}
	void addState(const std::string& key, EventRiser& events)
	{
		states[key] = StatePtr( new State(events) );
	}
	void addState(const std::string& key, EventRiser::Ptr events)
	{
		states[key] = StatePtr( new State(events) );
	}
};

}
}

#endif /* STATESMONITOR_H_ */
