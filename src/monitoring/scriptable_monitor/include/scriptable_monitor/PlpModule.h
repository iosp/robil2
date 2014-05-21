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

class ScriptHost;

class PlpModule {
private:
	struct Info{
		Info(string s):params(s),source(s){}
		Info():params(""){}
		ScriptParameters params;
		string source;
	};
	string _status;
	vector<Info> _scripts;
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
		cout<<"[i] stop script : "<<name<<" , "<<scr<<endl;
		get_all_modules()[name].stop_script(scr);
	}

	void start();
	static void start(string name){
		PlpModuleLock
		cout<<"[i] Start module : "<<name<<endl;
		get_all_modules()[name].start();
	}
	void stop();
	static void stop(string name){
		PlpModuleLock
		cout<<"[i] stop module : "<<name<<endl;
		get_all_modules()[name].stop();
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
};


inline double system_time_seconds(){
	ptime time_t_epoch(boost::gregorian::date(1970,1,1));
	ptime now = microsec_clock::local_time();
	time_duration diff = now - time_t_epoch;
	long x = diff.total_milliseconds();
	return (x*0.001);
}

#endif /* PLPMODULE_H_ */
