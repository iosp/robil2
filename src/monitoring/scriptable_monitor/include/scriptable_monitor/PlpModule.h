/*
 * PlpModule.h
 *
 *  Created on: May 18, 2014
 *      Author: dan
 */

#ifndef PLPMODULE_H_
#define PLPMODULE_H_

#include <map>
#include <list>
#include <vector>
#include <set>
#include <algorithm>

#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost;
using namespace boost::posix_time;

#include "ScriptParameters.h"

inline double system_time_seconds(){
	ptime time_t_epoch(boost::gregorian::date(1970,1,1));
	ptime now = microsec_clock::local_time();
	time_duration diff = now - time_t_epoch;
	long x = diff.total_milliseconds();
	return (x*0.001);
}

class ScriptHost;

class PlpModule {
private:
	struct Info{
		Info(string s):params(s),source(s){}
		Info():params(""){}
		ScriptParameters params;
		string source;
	};
	string _module_name;
	string _status;
	string _internal_status;
	vector<Info> _scripts;
	double _repeated_time;

public:
	struct Timer{
		string name;
		double start;
		double timeout;
		string action;
		void restart(){
			timeout = (timeout-start)+system_time_seconds();
		}
		Timer(string name="", double duration=-1.0, string action="")
		:name(name),start(system_time_seconds()),timeout(start+duration),action(action)
		{

		}
	};
private:
	static map<string, Timer> _timers;

public:
	static ScriptHost* sh;
public:
	PlpModule();
	virtual ~PlpModule();
	static map<string,PlpModule>& get_all_modules(){
		static map<string,PlpModule> modules;
		return modules;
	}
	static recursive_mutex& mutex(){ static recursive_mutex m; return m; }
	#define PlpModuleLock boost::recursive_mutex::scoped_lock plp_locker(PlpModule::mutex());

	string status()const{ return _status; }
	static string status(string name){
		PlpModuleLock
		string status = "stop";
		if(get_all_modules().count(name))
			status = get_all_modules().at(name).status();
		return status;
	}

	bool is_repeated()const{ return _repeated_time>0; }
	static bool is_repeated(string name){
		PlpModuleLock
		if(get_all_modules().count(name))
			return get_all_modules().at(name).is_repeated();
		return false;
	}

	void set_repeated_time(double t_sec){  _repeated_time=t_sec; }
	static void set_repeated_freq(string name, double t_hz){
		PlpModuleLock
		if(get_all_modules().count(name))
			return get_all_modules().at(name).set_repeated_time(t_hz<=0?-1:1.0/t_hz);
	}

	void set_status(string st){ _status = st; }
	static void status(string name, string st){
		PlpModuleLock
		string status = "stop";
		get_all_modules()[name].set_status(st);
	}

	void add_script(Info info){ _scripts.push_back(info); }
	static void add_script(string source){
		Info info(source);
		string modulename = info.params["module"];
		string scriptname = info.params["name"];
		PlpModuleLock
		get_all_modules()[modulename]._module_name = modulename;
		get_all_modules()[modulename].add_script(info);
	}

	vector<Info>::const_iterator _scripts_find(string name)const{
		vector<Info>::const_iterator e = _scripts.end();
		for(vector<Info>::const_iterator i=_scripts.begin();i!=e;i++){
			if(i->params["name"] == name) return i;
		}
		return e;
	}

	static string search_module(string sc_name){
		PlpModuleLock
		typedef map<string,PlpModule> ModuleCollection;
		ModuleCollection& mc = get_all_modules();
		for(ModuleCollection::const_iterator i=mc.begin();i!=mc.end();i++){
			if(i->second._scripts_find(sc_name) != i->second._scripts.end()){
				return i->first;
			}
		}
		return "";
	}


	static bool contains(string name){
		PlpModuleLock
		return get_all_modules().find(name)!=get_all_modules().end();
	}

	void stop_script(string scr);
	static void stop_script(string name, string scr){
		PlpModuleLock
		//out<<"\t[i] stop script : "<<name<<" , "<<scr<<endl;
		get_all_modules()[name].stop_script(scr);
	}

	void start();
	static void start(string name){
		PlpModuleLock
		cout<<"[i] start module : "<<name<<endl;
		get_all_modules()[name].start();
	}
	void stop();
	static void stop(string name){
		PlpModuleLock
		cout<<"[i] stop module : "<<name<<endl;
		get_all_modules()[name].stop();
		get_all_modules().erase(name);
	}
	void resume();
	static void resume(string name){
		PlpModuleLock
		cout<<"[i] resume module : "<<name<<endl;
		get_all_modules()[name].resume();
	}
	void pause();
	static void pause(string name){
		PlpModuleLock
		cout<<"[i] pause module : "<<name<<endl;
		get_all_modules()[name].pause();
	}

	static void start_timer(string name, double timeout, string action){
		cout<<"\t[i] timer: start "<<name<<" for "<<action<<endl;
		PlpModuleLock
		Timer t(name, timeout, action);
		_timers[name] = t;
	}
	static void stop_timer(string name){
		cout<<"\t[i] timer: stop "<<name<<endl;
		PlpModuleLock
		map<string,Timer>::iterator i = _timers.find(name);
		if(i!=_timers.end()){
			_timers.erase(i);
		}
	}
	static void stop_timer_by_prefix(string name_pref){
		PlpModuleLock
		for(map<string,Timer>::iterator i=_timers.begin();i!=_timers.end();i++){
			if(boost::starts_with(i->second.name, name_pref)){
				cout<<"\t[i] timer: stop "<<i->second.name<<endl;
				_timers.erase(i);
				return;
			}
		}
	}
	static void restart_timer_by_prefix(string name_pref){
		PlpModuleLock
		double now = system_time_seconds();
		for(map<string,Timer>::iterator i=_timers.begin();i!=_timers.end();i++){
			if(boost::starts_with(i->second.name, name_pref)){
				cout<<"\t[i] timer: restart "<<i->second.name<<endl;
				double duration = (i->second.timeout-i->second.start);
				i->second.start = now;
				i->second.timeout = duration+i->second.start;
				return;
			}
		}
	}
	static vector<Timer> check_timers_for_timeout(bool restart){
		vector<Timer> res;
		double now = system_time_seconds();
		for(map<string,Timer>::iterator i=_timers.begin();i!=_timers.end();i++){
			if(now >= i->second.timeout){
				res.push_back(i->second);
				if(restart){
					double duration = (i->second.timeout-i->second.start);
					i->second.start = now;
					i->second.timeout = duration+i->second.start;
				}
			}
		}
		return res;
	}
};



#endif /* PLPMODULE_H_ */
